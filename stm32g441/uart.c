// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32g441xx.h"
#include "uart.h"
#include "usb_cdc.h"
#include "hal_gpio.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(UART_TX,      B, 3)
HAL_GPIO_PIN(UART_RX,      B, 4)

#define UART_BUF_SIZE      256

/*- Types ------------------------------------------------------------------*/
typedef struct
{
  int       wr;
  int       rd;
  uint16_t  data[UART_BUF_SIZE];
} fifo_buffer_t;

/*- Variables --------------------------------------------------------------*/
static volatile fifo_buffer_t uart_rx_fifo;
static volatile fifo_buffer_t uart_tx_fifo;
static volatile bool uart_fifo_overflow = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void uart_init(usb_cdc_line_coding_t *line_coding)
{
  HAL_GPIO_UART_RX_alt(7);
  HAL_GPIO_UART_TX_alt(7);

  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

  uart_tx_fifo.wr = 0;
  uart_tx_fifo.rd = 0;

  uart_rx_fifo.wr = 0;
  uart_rx_fifo.rd = 0;

  uart_fifo_overflow = false;

  int bits, parity, stop;

  if (USB_CDC_7_DATA_BITS == line_coding->bDataBits)
    bits = USART_CR1_M1;
  else
    bits = 0;

  if (USB_CDC_EVEN_PARITY == line_coding->bParityType)
    parity = USART_CR1_PCE;
  else if(USB_CDC_ODD_PARITY == line_coding->bParityType)
    parity = USART_CR1_PCE | USART_CR1_PS;
  else
    parity = 0;

  if (USB_CDC_1_5_STOP_BITS == line_coding->bCharFormat)
    stop = 3;
  else if (USB_CDC_2_STOP_BITS == line_coding->bCharFormat)
    stop = 2;
  else
    stop = 0;

  USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_FIFOEN | bits | parity | stop;
  USART2->CR2 = (stop << USART_CR2_STOP_Pos);
  USART2->BRR = (F_CPU / line_coding->dwDTERate);
  USART2->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE;

  NVIC_EnableIRQ(USART2_IRQn);
}

//-----------------------------------------------------------------------------
void uart_close(void)
{
  RCC->APB1RSTR1 = RCC_APB1RSTR1_USART2RST;
  RCC->APB1RSTR1 = 0;

  HAL_GPIO_UART_RX_in();
  HAL_GPIO_UART_TX_in();
}

//-----------------------------------------------------------------------------
static bool fifo_push(volatile fifo_buffer_t *fifo, int value)
{
  int next_wr = (fifo->wr + 1) % UART_BUF_SIZE;

  if (next_wr == fifo->rd)
    return false;

  fifo->data[fifo->wr] = value;
  fifo->wr = next_wr;

  return true;
}

//-----------------------------------------------------------------------------
static bool fifo_pop(volatile fifo_buffer_t *fifo, int *value)
{
  if (fifo->rd == fifo->wr)
    return false;

  *value = fifo->data[fifo->rd];
  fifo->rd = (fifo->rd + 1) % UART_BUF_SIZE;

  return true;
}

//-----------------------------------------------------------------------------
bool uart_write_byte(int byte)
{
  bool res = false;

  NVIC_DisableIRQ(USART2_IRQn);

  if (fifo_push(&uart_tx_fifo, byte))
  {
    USART2->CR1 |= USART_CR1_TXEIE;
    res = true;
  }

  NVIC_EnableIRQ(USART2_IRQn);

  return res;
}

//-----------------------------------------------------------------------------
bool uart_read_byte(int *byte)
{
  bool res = false;

  NVIC_DisableIRQ(USART2_IRQn);

  if (uart_fifo_overflow)
  {
    *byte = (USB_CDC_SERIAL_STATE_OVERRUN << 8);
    uart_fifo_overflow = false;
    res = true;
  }
  else if (fifo_pop(&uart_rx_fifo, byte))
  {
    res = true;
  }

  NVIC_EnableIRQ(USART2_IRQn);

  return res;
}

//-----------------------------------------------------------------------------
void uart_set_break(bool brk)
{
  if (brk)
  {
    HAL_GPIO_UART_TX_out();
    HAL_GPIO_UART_TX_clr();
  }
  else
  {
    HAL_GPIO_UART_TX_alt(7);
  }
}

//-----------------------------------------------------------------------------
void irq_handler_usart2(void)
{
  int isr = USART2->ISR;
  int cr1 = USART2->CR1;

  if ((cr1 & USART_CR1_RXNEIE) && (isr & USART_ISR_RXNE))
  {
    int byte = USART2->RDR;
    int state = 0;

    if (isr & USART_ISR_FE)
      state |= USB_CDC_SERIAL_STATE_FRAMING;

    if (isr & USART_ISR_PE)
      state |= USB_CDC_SERIAL_STATE_PARITY;

    byte |= (state << 8);

    if (!fifo_push(&uart_rx_fifo, byte))
      uart_fifo_overflow = true;
  }

  if ((cr1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE))
  {
    int byte;

    if (fifo_pop(&uart_tx_fifo, &byte))
      USART2->TDR = byte;
    else
      USART2->CR1 &= ~USART_CR1_TXEIE;
  }

  USART2->ICR = USART_ICR_ORECF | USART_ICR_PECF | USART_ICR_FECF;
}


