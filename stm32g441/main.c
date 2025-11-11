// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32g441xx.h"
#include "hal_gpio.h"
#include "usb.h"
#include "uart.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(VCP_STATUS,       B, 6)

#define USB_BUFFER_SIZE        64
#define UART_WAIT_TIMEOUT      10 // ms
#define STATUS_TIMEOUT         250 // ms

/*- Variables ---------------------------------------------------------------*/
static uint64_t app_system_time = 0;
static uint64_t app_status_timeout = 0;

static alignas(4) uint8_t app_recv_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_send_buffer[USB_BUFFER_SIZE];
static int app_recv_buffer_size = 0;
static int app_recv_buffer_ptr = 0;
static int app_send_buffer_ptr = 0;
static bool app_send_buffer_free = true;
static bool app_send_zlp = false;
static uint64_t app_uart_timeout = 0;
static uint64_t app_break_timeout = 0;
static bool app_vcp_event = false;
static bool app_vcp_open = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN |
      RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOFEN | RCC_AHB2ENR_GPIOGEN;

  PWR->CR5 = 0; // Boost mode

  FLASH->ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_DBG_SWEN;

  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  while (0 == (RCC->CRRCR & RCC_CRRCR_HSI48RDY));

  RCC->CR = RCC_CR_HSEON | RCC_CR_HSEBYP;
  while (0 == (RCC->CR & RCC_CR_HSERDY));

  RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | RCC_PLLCFGR_PLLREN | (0/*1*/ << RCC_PLLCFGR_PLLM_Pos) |
      (28 << RCC_PLLCFGR_PLLN_Pos) | (0/*2*/ << RCC_PLLCFGR_PLLR_Pos);

  RCC->CR |= RCC_CR_PLLON;

  RCC->CFGR = (3/*PLL*/ << RCC_CFGR_SW_Pos);
}

//-----------------------------------------------------------------------------
static void serial_number_init(void)
{
  volatile uint8_t *uid = (volatile uint8_t *)UID_BASE;
  uint32_t sn = 5381;

  for (int i = 0; i < 12; i++)
    sn = ((sn << 5) + sn) ^ uid[i];

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[8] = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->VAL  = 0;
  SysTick->LOAD = (F_CPU / 8) / 1000ul;
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
static void tx_task(void)
{
  while (app_recv_buffer_size)
  {
    if (!uart_write_byte(app_recv_buffer[app_recv_buffer_ptr]))
      break;

    app_recv_buffer_ptr++;
    app_recv_buffer_size--;
    app_vcp_event = true;

    if (0 == app_recv_buffer_size)
      usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  }
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
static void rx_task(void)
{
  int byte;

  if (!app_send_buffer_free)
    return;

  while (uart_read_byte(&byte))
  {
    int state = (byte >> 8) & 0xff;

    app_uart_timeout = app_system_time + UART_WAIT_TIMEOUT;
    app_vcp_event = true;

    if (state)
    {
      usb_cdc_set_state(state);
    }
    else
    {
      app_send_buffer[app_send_buffer_ptr++] = byte;

      if (USB_BUFFER_SIZE == app_send_buffer_ptr)
      {
        send_buffer();
        break;
      }
    }
  }
}

//-----------------------------------------------------------------------------
static void break_task(void)
{
  if (app_break_timeout && app_system_time > app_break_timeout)
  {
    uart_set_break(false);
    app_break_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
static void uart_timer_task(void)
{
  if (app_uart_timeout && app_system_time > app_uart_timeout)
  {
    if (app_send_zlp || app_send_buffer_ptr)
      send_buffer();

    app_uart_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_line_coding_updated(usb_cdc_line_coding_t *line_coding)
{
  uart_init(line_coding);
}

//-----------------------------------------------------------------------------
void usb_cdc_control_line_state_update(int line_state)
{
  bool status = line_state & USB_CDC_CTRL_SIGNAL_DTE_PRESENT;

  app_vcp_open        = status;
  app_send_buffer_ptr = 0;
  app_uart_timeout    = 0;
  app_break_timeout   = 0;

  if (app_vcp_open)
    uart_init(usb_cdc_get_line_coding());
  else
    uart_close();
}

//-----------------------------------------------------------------------------
void usb_cdc_send_break(int duration)
{
  if (USB_CDC_BREAK_DURATION_DISABLE == duration)
  {
    app_break_timeout = 0;
    uart_set_break(false);
  }
  else if (USB_CDC_BREAK_DURATION_INFINITE == duration)
  {
    app_break_timeout = 0;
    uart_set_break(true);
  }
  else
  {
    app_break_timeout = app_system_time + duration;
    uart_set_break(true);
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_send_callback(void)
{
  app_send_buffer_free = true;
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
  app_send_buffer_ptr = 0;

  (void)config;
}

//-----------------------------------------------------------------------------
static void status_timer_task(void)
{
  if (app_system_time < app_status_timeout)
    return;

  app_status_timeout = app_system_time + STATUS_TIMEOUT;

  if (app_vcp_event)
    HAL_GPIO_VCP_STATUS_toggle();
  else
    HAL_GPIO_VCP_STATUS_write(app_vcp_open);

  app_vcp_event = false;
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  usb_init();
  usb_cdc_init();
  serial_number_init();

  app_status_timeout = STATUS_TIMEOUT;

  HAL_GPIO_VCP_STATUS_out();
  HAL_GPIO_VCP_STATUS_clr();

  while (1)
  {
    sys_time_task();
    status_timer_task();
    usb_task();
    tx_task();
    rx_task();
    break_task();
    uart_timer_task();
  }

  return 0;
}

