// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32g441xx.h"
#include "usb.h"
#include "usb_std.h"
#include "hal_gpio.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(USB_DM, A, 11)
HAL_GPIO_PIN(USB_DP, A, 12)

#define USB_EP_NUM             8
#define USB_PM_SIZE            1024
#define USB_EPnR(n)            (*((volatile uint16_t *)(USB_EP0R + (n)*4)))

#define USB_EP_MEM_RX(n)       (volatile uint8_t *)(USB_PMAADDR + usb_ep_desc[n].ADDR_RX)
#define USB_EP_MEM_TX(n)       (volatile uint8_t *)(USB_PMAADDR + usb_ep_desc[n].ADDR_TX)

#define COUNT_RX_NUM_BLOCK(n)  ((n) << 10)
#define COUNT_RX_BL_SIZE       (1 << 15)
#define COUNT_RX_MASK          0x3ff

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint16_t ADDR_TX;
  uint16_t COUNT_TX;
  uint16_t ADDR_RX;
  uint16_t COUNT_RX;
} UsbEpDesc;

/*- Variables ---------------------------------------------------------------*/
static volatile UsbEpDesc *usb_ep_desc = (volatile UsbEpDesc *)USB_PMAADDR;
static void (*usb_control_recv_callback)(uint8_t *data, int size);
static int usb_pm_ptr;
static int usb_setup_length;
static uint8_t *usb_recv_buffer[USB_EP_NUM];
static int usb_recv_size[USB_EP_NUM];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void usb_hw_init(void)
{
  HAL_GPIO_USB_DM_analog();
  HAL_GPIO_USB_DP_analog();

  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN;

  CRS->CR = CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

  USB->CNTR   = USB_CNTR_FRES;
  USB->CNTR   = 0;
  USB->ISTR   = 0;
  USB->BTABLE = 0;
  USB->BCDR   = USB_BCDR_DPPU;

  usb_setup_length = -1;
}

//-----------------------------------------------------------------------------
void usb_attach(void)
{
  USB->CNTR = 0;
}

//-----------------------------------------------------------------------------
void usb_detach(void)
{
  USB->CNTR = USB_CNTR_FRES;
}

//-----------------------------------------------------------------------------
static void usb_pm_alloc_rx(int ep, int size)
{
  int count = (size - 1) / 32;

  usb_ep_desc[ep].ADDR_RX = usb_pm_ptr;
  usb_ep_desc[ep].COUNT_RX = COUNT_RX_BL_SIZE | COUNT_RX_NUM_BLOCK(count);

  usb_pm_ptr += (count + 1) * 32;

  while (usb_pm_ptr > USB_PM_SIZE);
}

//-----------------------------------------------------------------------------
static void usb_pm_alloc_tx(int ep, int size)
{
  usb_ep_desc[ep].ADDR_TX  = usb_pm_ptr;
  usb_ep_desc[ep].COUNT_TX = 0;

  usb_pm_ptr += size;

  while (usb_pm_ptr > USB_PM_SIZE);
}

//-----------------------------------------------------------------------------
static void usb_set_rx_status(int ep, int status)
{
  int reg = USB_EPnR(ep) & USB_EPRX_DTOGMASK;

  if (status & USB_EPRX_DTOG1)
    reg ^= USB_EPRX_DTOG1;

  if (status & USB_EPRX_DTOG2)
    reg ^= USB_EPRX_DTOG2;

  USB_EPnR(ep) = reg | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

//-----------------------------------------------------------------------------
static void usb_set_tx_status(int ep, int status)
{
  int reg = USB_EPnR(ep) & USB_EPTX_DTOGMASK;

  if (status & USB_EPTX_DTOG1)
    reg ^= USB_EPTX_DTOG1;

  if (status & USB_EPTX_DTOG2)
    reg ^= USB_EPTX_DTOG2;

  USB_EPnR(ep) = reg | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

//-----------------------------------------------------------------------------
static void usb_clear_bit(int ep, int bit)
{
  USB_EPnR(ep) = ((USB_EPnR(ep) & USB_EPREG_MASK) | (USB_EP_CTR_TX | USB_EP_CTR_RX)) & ~bit;
}

//-----------------------------------------------------------------------------
void usb_set_address(int address)
{
  USB->DADDR = USB_DADDR_EF | address;
}

//-----------------------------------------------------------------------------
void usb_send(int ep, uint8_t *data, int size)
{
  volatile uint16_t *tx_buf = (volatile uint16_t *)USB_EP_MEM_TX(ep);
  uint16_t *hdata = (uint16_t *)data;
  int count = (size + 1) / sizeof(uint16_t);

  for (int i = 0; i < count; i++)
    tx_buf[i] = hdata[i];

  usb_ep_desc[ep].COUNT_TX = size;

  usb_set_tx_status(ep, USB_EP_TX_VALID);
}

//-----------------------------------------------------------------------------
void usb_recv(int ep, uint8_t *data, int size)
{
  usb_recv_buffer[ep] = data;
  usb_recv_size[ep]   = size;

  usb_set_rx_status(ep, USB_EP_RX_VALID);
}

//-----------------------------------------------------------------------------
void usb_configure_endpoint(usb_endpoint_descriptor_t *desc)
{
  int ep, dir, type, size;

  ep = desc->bEndpointAddress & USB_INDEX_MASK;
  dir = desc->bEndpointAddress & USB_DIRECTION_MASK;
  type = desc->bmAttributes & 0x03;
  size = desc->wMaxPacketSize & 0x3ff;

  if (USB_BULK_ENDPOINT == type)
    USB_EPnR(ep) = USB_EP_BULK | ep;
  else if (USB_INTERRUPT_ENDPOINT == type)
    USB_EPnR(ep) = USB_EP_INTERRUPT | ep;
  else if (USB_ISOCHRONOUS_ENDPOINT == type)
    USB_EPnR(ep) = USB_EP_ISOCHRONOUS | ep;
  else
    while (1);

  if (USB_IN_ENDPOINT == dir)
  {
    usb_pm_alloc_tx(ep, size);
    usb_set_tx_status(ep, USB_EP_TX_NAK);
  }
  else
  {
    usb_pm_alloc_rx(ep, USB_CTRL_EP_SIZE);
    usb_set_rx_status(ep, USB_EP_RX_NAK);
  }
}

//-----------------------------------------------------------------------------
bool usb_endpoint_configured(int ep, int dir)
{
  if (USB_IN_ENDPOINT == dir)
    return ((USB_EPnR(ep) & USB_EPTX_STAT) != USB_EP_TX_DIS);
  else
    return ((USB_EPnR(ep) & USB_EPRX_STAT) != USB_EP_RX_DIS);
}

//-----------------------------------------------------------------------------
int usb_endpoint_get_status(int ep, int dir)
{
  if (USB_IN_ENDPOINT == dir)
    return ((USB_EPnR(ep) & USB_EPTX_STAT) == USB_EP_TX_STALL);
  else
    return ((USB_EPnR(ep) & USB_EPRX_STAT) == USB_EP_RX_STALL);
}

//-----------------------------------------------------------------------------
void usb_endpoint_set_feature(int ep, int dir)
{
  if (USB_IN_ENDPOINT == dir)
  {
    usb_set_tx_status(ep, USB_EP_TX_STALL);
  }
  else
  {
    usb_set_rx_status(ep, USB_EP_RX_STALL);
  }
}

//-----------------------------------------------------------------------------
void usb_endpoint_clear_feature(int ep, int dir)
{
  if (USB_IN_ENDPOINT == dir)
  {
    usb_set_tx_status(ep, USB_EP_TX_NAK);

    if (USB_EPnR(ep) & USB_EP_DTOG_TX)
      USB_EPnR(ep) |= (USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX);
  }
  else
  {
    usb_set_rx_status(ep, USB_EP_RX_NAK);

    if (USB_EPnR(ep) & USB_EP_DTOG_RX)
      USB_EPnR(ep) |= (USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX);
  }
}

//-----------------------------------------------------------------------------
void usb_control_send_zlp(void)
{
  usb_ep_desc[0].COUNT_TX = 0;
  usb_set_tx_status(0, USB_EP_TX_VALID);

  while (0 == (USB_EPnR(0) & USB_EP_CTR_TX));

  usb_clear_bit(0, USB_EP_CTR_TX);
}

//-----------------------------------------------------------------------------
void usb_control_stall(void)
{
  usb_set_rx_status(0, USB_EP_RX_STALL);
  usb_set_tx_status(0, USB_EP_TX_STALL);
}

//-----------------------------------------------------------------------------
void usb_control_send(uint8_t *data, int size)
{
  bool need_zlp = (size < usb_setup_length) &&
      ((size & (usb_device_descriptor.bMaxPacketSize0-1)) == 0);

  while (size)
  {
    int transfer_size = USB_LIMIT(size, usb_device_descriptor.bMaxPacketSize0);

    usb_send(0, data, transfer_size);

    while (0 == (USB_EPnR(0) & USB_EP_CTR_TX));

    usb_clear_bit(0, USB_EP_CTR_TX);

    size -= transfer_size;
    data += transfer_size;
  }

  if (need_zlp)
    usb_control_send_zlp();
}

//-----------------------------------------------------------------------------
void usb_control_recv(void (*callback)(uint8_t *data, int size))
{
  usb_control_recv_callback = callback;
}

//-----------------------------------------------------------------------------
void usb_task(void)
{
  int istr = USB->ISTR;

  if (istr & USB_ISTR_RESET)
  {
    USB->ISTR &= ~USB_ISTR_RESET;

    usb_set_address(0);

    usb_pm_ptr = USB_EP_NUM * sizeof(UsbEpDesc);
    usb_setup_length = -1;

    usb_pm_alloc_rx(0, USB_CTRL_EP_SIZE);
    usb_pm_alloc_tx(0, USB_CTRL_EP_SIZE);

    USB_EPnR(0) = USB_EP_CONTROL;

    usb_set_rx_status(0, USB_EP_RX_VALID);
    usb_set_tx_status(0, USB_EP_TX_NAK);
  }

  int epnr0 = USB_EPnR(0);

  if (epnr0 & USB_EP_CTR_RX)
  {
    int size = usb_ep_desc[0].COUNT_RX & COUNT_RX_MASK;
    volatile uint8_t *rx_buf = USB_EP_MEM_RX(0);
    uint8_t buffer[USB_CTRL_EP_SIZE];

    for (int i = 0; i < size; i++)
      buffer[i] = rx_buf[i];

    if (epnr0 & USB_EP_SETUP)
    {
      usb_request_t *request = (usb_request_t *)buffer;

      usb_setup_length = request->wLength;

      if (sizeof(usb_request_t) == size)
      {
        if (usb_handle_standard_request(request))
          usb_set_rx_status(0, USB_EP_RX_VALID);
        else
          usb_control_stall();
      }
      else
      {
        usb_control_stall();
      }

      usb_setup_length = -1;
    }
    else
    {
      if (usb_control_recv_callback)
      {
        usb_control_recv_callback(buffer, size);
        usb_control_recv_callback = NULL;
        usb_control_send_zlp();
      }

      usb_set_rx_status(0, USB_EP_RX_VALID);
    }

    usb_clear_bit(0, USB_EP_CTR_RX);
  }

  for (int ep = 1; ep < USB_EP_NUM; ep++)
  {
    int epnr = USB_EPnR(ep);

    if (epnr & USB_EP_CTR_RX)
    {
      usb_clear_bit(ep, USB_EP_CTR_RX);

      volatile uint8_t *rx_buf = USB_EP_MEM_RX(ep);
      int size = usb_ep_desc[ep].COUNT_RX & COUNT_RX_MASK;

      size = USB_LIMIT(size, usb_recv_size[ep]);

      for (int i = 0; i < size; i++)
        usb_recv_buffer[ep][i] = rx_buf[i];

      usb_recv_callback(ep, size);
    }

    if (epnr & USB_EP_CTR_TX)
    {
      usb_clear_bit(ep, USB_EP_CTR_TX);

      usb_send_callback(ep);
    }
  }
}


