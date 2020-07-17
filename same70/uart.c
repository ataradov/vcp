/*
 * Copyright (c) 2020, Alex Taradov <alex@taradov.com>
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "same70.h"
#include "hal_gpio.h"
#include "uart.h"
#include "usb_cdc.h"

/*- Definitions -------------------------------------------------------------*/
#define UART_BUF_SIZE        4096

HAL_GPIO_PIN(UART_TX,        B, 4)
HAL_GPIO_PIN(UART_RX,        A, 21)

#define UART_MODULE          USART1
#define UART_TX_ABCD         3
#define UART_RX_ABCD         0
#define UART_PID             PMC_PCER0_PID14
#define UART_IRQ_INDEX       USART1_IRQn
#define UART_IRQ_HANDLER     irq_handler_usart1

/*- Types ------------------------------------------------------------------*/
typedef struct
{
  int       wr;
  int       rd;
  uint8_t   data[UART_BUF_SIZE];
} fifo_buffer_t;

/*- Variables --------------------------------------------------------------*/
static volatile fifo_buffer_t uart_rx_fifo;
static volatile fifo_buffer_t uart_tx_fifo;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void uart_init(usb_cdc_line_coding_t *line_coding)
{
  int chrl, par, nbstop, cd, fp;

  NVIC_DisableIRQ(UART_IRQ_INDEX);

  uart_tx_fifo.wr = 0;
  uart_tx_fifo.rd = 0;

  uart_rx_fifo.wr = 0;
  uart_rx_fifo.rd = 0;

  if (USB_CDC_5_DATA_BITS == line_coding->bDataBits)
    chrl = US_MR_CHRL_5_BIT;
  else if (USB_CDC_6_DATA_BITS == line_coding->bDataBits)
    chrl = US_MR_CHRL_6_BIT;
  else if (USB_CDC_7_DATA_BITS == line_coding->bDataBits)
    chrl = US_MR_CHRL_7_BIT;
  else if (USB_CDC_8_DATA_BITS == line_coding->bDataBits)
    chrl = US_MR_CHRL_8_BIT;
  else
    chrl = US_MR_CHRL_8_BIT;

  if (USB_CDC_NO_PARITY == line_coding->bParityType)
    par = US_MR_PAR_NO;
  else if (USB_CDC_ODD_PARITY == line_coding->bParityType)
    par = US_MR_PAR_ODD;
  else if (USB_CDC_EVEN_PARITY == line_coding->bParityType)
    par = US_MR_PAR_EVEN;
  else if (USB_CDC_MARK_PARITY == line_coding->bParityType)
    par = US_MR_PAR_MARK;
  else if (USB_CDC_SPACE_PARITY == line_coding->bParityType)
    par = US_MR_PAR_SPACE;
  else
    par = US_MR_PAR_NO;

  if (USB_CDC_1_STOP_BIT == line_coding->bCharFormat)
    nbstop = US_MR_NBSTOP_1_BIT;
  else if (USB_CDC_1_5_STOP_BITS == line_coding->bCharFormat)
    nbstop = US_MR_NBSTOP_1_5_BIT;
  else if (USB_CDC_2_STOP_BITS == line_coding->bCharFormat)
    nbstop = US_MR_NBSTOP_2_BIT;
  else
    nbstop = US_MR_NBSTOP_1_BIT;

  HAL_GPIO_UART_TX_abcd(UART_TX_ABCD);

  HAL_GPIO_UART_RX_abcd(UART_RX_ABCD);
  HAL_GPIO_UART_RX_pullup();

  PMC->PMC_PCER0 = UART_PID;

  cd = F_CPU / (16 * line_coding->dwDTERate);
  fp = (F_CPU / line_coding->dwDTERate - 16 * cd) / 2;

  UART_MODULE->US_CR   = US_CR_RSTTX | US_CR_RSTRX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
  UART_MODULE->US_CR   = US_CR_RXEN | US_CR_TXEN;
  UART_MODULE->US_BRGR = US_BRGR_CD(cd) | US_BRGR_FP(fp);
  UART_MODULE->US_MR   = US_MR_USART_MODE_NORMAL | chrl | par | nbstop;
  UART_MODULE->US_IER  = US_IER_RXRDY;

  NVIC_EnableIRQ(UART_IRQ_INDEX);
}

//-----------------------------------------------------------------------------
bool uart_write_byte(int byte)
{
  int wr = (uart_tx_fifo.wr + 1) % UART_BUF_SIZE;
  bool res = false;

  NVIC_DisableIRQ(UART_IRQ_INDEX);

  if (wr != uart_tx_fifo.rd)
  {
    uart_tx_fifo.data[uart_tx_fifo.wr] = byte;
    uart_tx_fifo.wr = wr;
    res = true;

    UART_MODULE->US_IER = US_IER_TXRDY;
  }

  NVIC_EnableIRQ(UART_IRQ_INDEX);

  return res;
}

//-----------------------------------------------------------------------------
bool uart_read_byte(int *byte)
{
  bool res = false;

  NVIC_DisableIRQ(UART_IRQ_INDEX);

  if (uart_rx_fifo.rd != uart_rx_fifo.wr)
  {
    *byte = uart_rx_fifo.data[uart_rx_fifo.rd];
    uart_rx_fifo.rd = (uart_rx_fifo.rd + 1) % UART_BUF_SIZE;
    res = true;
  }

  NVIC_EnableIRQ(UART_IRQ_INDEX);

  return res;
}

//-----------------------------------------------------------------------------
void UART_IRQ_HANDLER(void)
{
  int csr = UART_MODULE->US_CSR;
  int flags = csr & UART_MODULE->US_IMR;

  if (flags & US_CSR_RXRDY)
  {
    int byte = UART_MODULE->US_RHR;
    int wr = (uart_rx_fifo.wr + 1) % UART_BUF_SIZE;

    UART_MODULE->US_CR = US_CR_RSTSTA;

    if (csr & US_CSR_OVRE)
      uart_serial_state_update(USB_CDC_SERIAL_STATE_OVERRUN);

    if (csr & US_CSR_FRAME)
      uart_serial_state_update(USB_CDC_SERIAL_STATE_FRAMING);

    if (csr & US_CSR_PARE)
      uart_serial_state_update(USB_CDC_SERIAL_STATE_PARITY);

    if (wr == uart_rx_fifo.rd)
    {
      uart_serial_state_update(USB_CDC_SERIAL_STATE_OVERRUN);
    }
    else
    {
      uart_rx_fifo.data[uart_rx_fifo.wr] = byte;
      uart_rx_fifo.wr = wr;
    }
  }

  if (flags & US_CSR_TXRDY)
  {
    if (uart_tx_fifo.rd == uart_tx_fifo.wr)
    {
      UART_MODULE->US_IDR = US_IDR_TXRDY;
    }
    else
    {
      UART_MODULE->US_THR = uart_tx_fifo.data[uart_tx_fifo.rd];
      uart_tx_fifo.rd = (uart_tx_fifo.rd + 1) % UART_BUF_SIZE;
    }
  }
}

