/*
 * Copyright (c) 2017-2019, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "saml21.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "usb.h"
#include "uart.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_BUFFER_SIZE        64
#define UART_WAIT_TIMEOUT      10 // ms

// NOTE: Set both rising and falling edge times to 0 to disable edge detection.
HAL_GPIO_PIN(STATUS,           A, 9);
#define STATUS_INACTIVE_STATE  0 // 0 - Low, 1 - High, 2 - Hi-Z
#define STATUS_ACTIVE_STATE    1 // 0 - Low, 1 - High, 2 - Hi-Z
#define STATUS_RISING_EDGE     0 // ms
#define STATUS_FALLING_EDGE    0 // ms

/*- Variables ---------------------------------------------------------------*/
static alignas(4) uint8_t app_recv_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_send_buffer[USB_BUFFER_SIZE];
static int app_recv_buffer_size = 0;
static int app_recv_buffer_ptr = 0;
static int app_send_buffer_ptr = 0;
static bool app_send_buffer_free = true;
static bool app_send_zlp = false;
static int app_system_time = 0;
static int app_uart_timeout = 0;
static bool app_status = false;
static int app_status_timeout = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t sn = 0;

  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
  PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2;
  while (!PM->INTFLAG.reg);

  MCLK->LPDIV.reg = MCLK_LPDIV_LPDIV_DIV4;
  while (0 == (MCLK->INTFLAG.reg & MCLK_INTFLAG_CKRDY));

  MCLK->BUPDIV.reg = MCLK_LPDIV_LPDIV_DIV8;
  while (0 == (MCLK->INTFLAG.reg & MCLK_INTFLAG_CKRDY));

  OSCCTRL->OSC16MCTRL.reg = OSCCTRL_OSC16MCTRL_ENABLE | OSCCTRL_OSC16MCTRL_FSEL_16;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_RWS(2);

  SUPC->INTFLAG.reg = SUPC_INTFLAG_BOD33RDY | SUPC_INTFLAG_BOD33DET;
  OSCCTRL->INTFLAG.reg = OSCCTRL_INTFLAG_DFLLRDY | OSCCTRL_INTFLAG_OSC16MRDY;

  OSCCTRL->DFLLCTRL.reg = 0;
  while (0 == (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

  OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_MUL(48000) | OSCCTRL_DFLLMUL_CSTEP(1) | OSCCTRL_DFLLMUL_FSTEP(1);
  while (0 == (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

  OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL)) | OSCCTRL_DFLLVAL_FINE(0x200);
  while (0 == (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

  OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE | OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_USBCRM;
  while (0 == (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

  GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) | GCLK_GENCTRL_GENEN;
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0);

  sn ^= *(volatile uint32_t *)0x0080a00c;
  sn ^= *(volatile uint32_t *)0x0080a040;
  sn ^= *(volatile uint32_t *)0x0080a044;
  sn ^= *(volatile uint32_t *)0x0080a048;

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[8] = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->VAL = 0;
  SysTick->LOAD = F_CPU / 1000ul;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  app_system_time = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_task(void)
{
  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    app_system_time++;
}

//-----------------------------------------------------------------------------
static int get_system_time(void)
{
  return app_system_time;
}

//-----------------------------------------------------------------------------
static void set_status_state(bool active)
{
  if (active)
  {
  #if STATUS_ACTIVE_STATE == 0
    HAL_GPIO_STATUS_out();
    HAL_GPIO_STATUS_clr();
  #elif STATUS_ACTIVE_STATE == 1
    HAL_GPIO_STATUS_out();
    HAL_GPIO_STATUS_set();
  #else
    HAL_GPIO_STATUS_in();
  #endif
  }
  else
  {
  #if STATUS_INACTIVE_STATE == 0
    HAL_GPIO_STATUS_out();
    HAL_GPIO_STATUS_clr();
  #elif STATUS_INACTIVE_STATE == 1
    HAL_GPIO_STATUS_out();
    HAL_GPIO_STATUS_set();
  #else
    HAL_GPIO_STATUS_in();
  #endif
  }
}

//-----------------------------------------------------------------------------
static void update_status(bool status)
{
#if STATUS_RISING_EDGE > 0
  if (false == app_status && true == status)
  {
    set_status_state(true);
    app_status_timeout = get_system_time() + STATUS_RISING_EDGE;
  }
#endif

#if STATUS_FALLING_EDGE > 0
  if (true == app_status && false == status)
  {
    set_status_state(true);
    app_status_timeout = get_system_time() + STATUS_FALLING_EDGE;
  }
#endif

#if STATUS_RISING_EDGE == 0 && STATUS_FALLING_EDGE == 0
  set_status_state(status);
#endif

  app_status = status;
}

//-----------------------------------------------------------------------------
static void status_task(void)
{
  if (app_status_timeout && get_system_time() > app_status_timeout)
  {
    set_status_state(false);
    app_status_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_send_callback(void)
{
  app_send_buffer_free = true;
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_send_buffer_free = false;
  app_send_zlp = (USB_BUFFER_SIZE == app_send_buffer_ptr);

  usb_cdc_send(app_send_buffer, app_send_buffer_ptr);

  app_send_buffer_ptr = 0;
}

//-----------------------------------------------------------------------------
void usb_cdc_recv_callback(int size)
{
  app_recv_buffer_ptr = 0;
  app_recv_buffer_size = size;
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  app_send_buffer_free = true;
  (void)config;
}

//-----------------------------------------------------------------------------
void usb_cdc_line_coding_updated(usb_cdc_line_coding_t *line_coding)
{
  uart_init(line_coding);
}

//-----------------------------------------------------------------------------
void usb_cdc_control_line_state_update(int line_state)
{
  update_status(line_state & USB_CDC_CTRL_SIGNAL_DTE_PRESENT);
}

//-----------------------------------------------------------------------------
static void tx_task(void)
{
  while (app_recv_buffer_size)
  {
    if (!uart_write_byte(app_recv_buffer[app_recv_buffer_ptr]))
      break;

    app_recv_buffer_ptr++;
    app_recv_buffer_size--;

    if (0 == app_recv_buffer_size)
      usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  }
}

//-----------------------------------------------------------------------------
static void rx_task(void)
{
  int byte;

  if (!app_send_buffer_free)
    return;

  while (uart_read_byte(&byte))
  {
    app_uart_timeout = get_system_time() + UART_WAIT_TIMEOUT;
    app_send_buffer[app_send_buffer_ptr++] = byte;

    if (USB_BUFFER_SIZE == app_send_buffer_ptr)
    {
      send_buffer();
      break;
    }
  }
}

//-----------------------------------------------------------------------------
static void uart_timer_task(void)
{
  if (app_uart_timeout && get_system_time() > app_uart_timeout)
  {
    if (app_send_zlp || app_send_buffer_ptr)
      send_buffer();

    app_uart_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void uart_serial_state_update(int state)
{
  usb_cdc_set_state(state);
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  usb_init();
  usb_cdc_init();
  set_status_state(false);

  while (1)
  {
    sys_time_task();
    usb_task();
    tx_task();
    rx_task();
    uart_timer_task();
    status_task();
  }

  return 0;
}
