// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2017-2022, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd11.h"
#include "hal_config.h"
#include "uart.h"
#include "usb_cdc.h"

/*- Definitions -------------------------------------------------------------*/
#define UART_BUF_SIZE            256

/*- Types ------------------------------------------------------------------*/
typedef struct
{
  int       wr;
  int       rd;
  uint16_t  data[UART_BUF_SIZE];
} fifo_buffer_t;

#define NEXT(idx)	((idx+1)%UART_BUF_SIZE)
#define PREV(idx)	((idx-1)%UART_BUF_SIZE)

#define EMPTY(F)	(F.rd == F.wr)
#define FULL(F)		(F.rd == NEXT(F.wr))

#define PUSH(F,v)	{	F.data[F.wr] = v; F.wr = NEXT(F.wr);	}
#define POP(F)		(	F.rd = NEXT(F.rd), F.data[PREV(F.rd)]	)

/*- Variables --------------------------------------------------------------*/
static volatile fifo_buffer_t uart_rx_fifo;
static volatile fifo_buffer_t uart_tx_fifo;
static volatile bool uart_fifo_overflow = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void uart_init(usb_cdc_line_coding_t *line_coding)
{
  int chsize, form, pmode, sbmode, baud, fp;

  HAL_GPIO_UART_TX_out();
  HAL_GPIO_UART_TX_clr();
  HAL_GPIO_UART_TX_pmuxen(UART_SERCOM_PMUX_TX);

  HAL_GPIO_UART_RX_pullup();
  HAL_GPIO_UART_RX_pmuxen(UART_SERCOM_PMUX_RX);

  PM->APBCMASK.reg |= UART_SERCOM_APBCMASK;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(UART_SERCOM_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
  while (UART_SERCOM->USART.CTRLA.bit.SWRST);

  uart_tx_fifo.wr = 0;
  uart_tx_fifo.rd = 0;

  uart_rx_fifo.wr = 0;
  uart_rx_fifo.rd = 0;

  uart_fifo_overflow = false;

  if (USB_CDC_8_DATA_BITS == line_coding->bDataBits)	//most propable is first
    chsize = 0;
  else if (USB_CDC_7_DATA_BITS == line_coding->bDataBits)
    chsize = 7;
  else if (USB_CDC_6_DATA_BITS == line_coding->bDataBits)
    chsize = 6;
  else if (USB_CDC_5_DATA_BITS == line_coding->bDataBits)
    chsize = 5;
  else
    chsize = 0;

  if (USB_CDC_NO_PARITY == line_coding->bParityType)
    form = 0;
  else
    form = 1;

  if (USB_CDC_EVEN_PARITY == line_coding->bParityType)
    pmode = 0;
  else
    pmode = SERCOM_USART_CTRLB_PMODE;

  if (USB_CDC_1_STOP_BIT == line_coding->bCharFormat)
    sbmode = 0;
  else
    sbmode = SERCOM_USART_CTRLB_SBMODE;

  baud = F_CPU / (16 * line_coding->dwDTERate);
  fp = (F_CPU / line_coding->dwDTERate - 16 * baud) / 2;

  UART_SERCOM->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_FORM(form) | SERCOM_USART_CTRLA_SAMPR(1) |
      SERCOM_USART_CTRLA_RXPO(UART_SERCOM_RXPO) |
      SERCOM_USART_CTRLA_TXPO(UART_SERCOM_TXPO);

  UART_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(chsize) | pmode | sbmode;

  UART_SERCOM->USART.BAUD.reg =
      SERCOM_USART_BAUD_FRACFP_BAUD(baud) | SERCOM_USART_BAUD_FRACFP_FP(fp);

  UART_SERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;

  UART_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

  NVIC_EnableIRQ(UART_SERCOM_IRQ_INDEX);
}

//-----------------------------------------------------------------------------
void uart_close(void)
{
  UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
  while (UART_SERCOM->USART.CTRLA.bit.SWRST);
}

//-----------------------------------------------------------------------------
bool uart_write_byte(int byte)
{
  bool res = false;

  NVIC_DisableIRQ(UART_SERCOM_IRQ_INDEX);

  if (!FULL(uart_tx_fifo))
  {
    PUSH(uart_tx_fifo, byte);
    res = true;

    UART_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
  }

  NVIC_EnableIRQ(UART_SERCOM_IRQ_INDEX);

  return res;
}

//-----------------------------------------------------------------------------
bool uart_read_byte(int *byte)
{
  bool res = false;

  NVIC_DisableIRQ(UART_SERCOM_IRQ_INDEX);

  if (uart_fifo_overflow)
  {
    *byte = (USB_CDC_SERIAL_STATE_OVERRUN << 8);
    uart_fifo_overflow = false;
    res = true;
  }
  else if (!EMPTY(uart_rx_fifo))
  {
    *byte = POP(uart_rx_fifo);
    res = true;
  }

  NVIC_EnableIRQ(UART_SERCOM_IRQ_INDEX);

  return res;
}

//-----------------------------------------------------------------------------
void uart_set_break(bool brk)
{
  if (brk)
    HAL_GPIO_UART_TX_pmuxdis();
  else
    HAL_GPIO_UART_TX_pmuxen(UART_SERCOM_PMUX_TX);
}

//-----------------------------------------------------------------------------
void UART_SERCOM_IRQ_HANDLER(void)
{
  int flags = UART_SERCOM->USART.INTFLAG.reg;

  if (flags & SERCOM_USART_INTFLAG_RXC)
  {
    int status = UART_SERCOM->USART.STATUS.reg;
    int byte = UART_SERCOM->USART.DATA.reg;
    int state = 0;

    UART_SERCOM->USART.STATUS.reg = status;

    if (status & SERCOM_USART_STATUS_BUFOVF)
      state |= USB_CDC_SERIAL_STATE_OVERRUN;

    if (status & SERCOM_USART_STATUS_FERR)
      state |= USB_CDC_SERIAL_STATE_FRAMING;

    if (status & SERCOM_USART_STATUS_PERR)
      state |= USB_CDC_SERIAL_STATE_PARITY;

    byte |= (state << 8);

    if (FULL(uart_rx_fifo))
    {
      uart_fifo_overflow = true;
    }
    else
    {
      PUSH(uart_rx_fifo, byte);
    }
  }

  if (flags & SERCOM_USART_INTFLAG_DRE)
  {
    if (EMPTY(uart_tx_fifo))
    {
      UART_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
    }
    else
    {
      UART_SERCOM->USART.DATA.reg = POP(uart_tx_fifo);
    }
  }
}
