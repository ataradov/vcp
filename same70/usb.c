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
#include <string.h>
#include <stdbool.h>
#include <stdalign.h>
#include "same70.h"
#include "hal_gpio.h"
#include "utils.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
#define USBHS_RAM_ADDR    0xa0100000u

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint8_t  *data;
  int      size;
} usb_recv_req_t;

/*- Variables ---------------------------------------------------------------*/
static usb_recv_req_t usb_recv_req[USB_EP_NUM];
static void (*usb_control_recv_callback)(uint8_t *data, int size);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void usb_hw_init(void)
{
  PMC->PMC_PCER1 = PMC_PCER1_PID34; // ID_USBHS

  PMC->CKGR_UCKR = CKGR_UCKR_UPLLEN | CKGR_UCKR_UPLLCOUNT(3);
  while (0 == (PMC->PMC_SR & PMC_SR_LOCKU));

  PMC->PMC_USB = PMC_USB_USBS;

  USBHS->USBHS_CTRL = USBHS_CTRL_UIMOD_DEVICE | USBHS_CTRL_USBE;
  while (0 == (USBHS->USBHS_SR & USBHS_SR_CLKUSABLE));

  for (int i = 0; i < USB_EP_NUM; i++)
    usb_reset_endpoint(i, 0);

  usb_detach();
  usb_attach();

  //NVIC_EnableIRQ(USBHS_IRQn);
}

//-----------------------------------------------------------------------------
void usb_attach(void)
{
  while (0 == (USBHS->USBHS_SR & USBHS_SR_CLKUSABLE));

  USBHS->USBHS_DEVCTRL &= ~USBHS_DEVCTRL_DETACH;
  USBHS->USBHS_DEVIER = USBHS_DEVIER_EORSTES;
  USBHS->USBHS_DEVICR = USBHS_DEVICR_EORSTC | USBHS_DEVICR_WAKEUPC |
      USBHS_DEVICR_MSOFC | USBHS_DEVICR_SOFC;
  USBHS->USBHS_DEVIFR = USBHS_DEVIFR_SUSPS;
}

//-----------------------------------------------------------------------------
void usb_detach(void)
{
  USBHS->USBHS_DEVCTRL |= USBHS_DEVCTRL_DETACH;
}

//-----------------------------------------------------------------------------
void usb_reset_endpoint(int ep, int dir)
{
  usb_recv_req[ep].data = NULL;
  usb_recv_req[ep].size = 0;

  USBHS->USBHS_DEVEPT |= (USBHS_DEVEPT_EPRST0 << ep);
  USBHS->USBHS_DEVEPT &= ~(USBHS_DEVEPT_EPRST0 << ep);
  USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_RSTDTS;

  (void)dir;
}

//-----------------------------------------------------------------------------
void usb_configure_endpoint(usb_endpoint_descriptor_t *desc)
{
  int ep, dir, type, size;

  ep = desc->bEndpointAddress & USB_INDEX_MASK;
  dir = desc->bEndpointAddress & USB_DIRECTION_MASK;
  type = desc->bmAttributes & 0x03;
  size = desc->wMaxPacketSize;

  usb_reset_endpoint(ep, dir);

  if (USB_ISOCHRONOUS_ENDPOINT != type)
    USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_RSTDTS;

  if (size <= 8)
    size = USBHS_DEVEPTCFG_EPSIZE_8_BYTE;
  else if (size <= 16)
    size = USBHS_DEVEPTCFG_EPSIZE_16_BYTE;
  else if (size <= 32)
    size = USBHS_DEVEPTCFG_EPSIZE_32_BYTE;
  else if (size <= 64)
    size = USBHS_DEVEPTCFG_EPSIZE_64_BYTE;
  else if (size <= 128)
    size = USBHS_DEVEPTCFG_EPSIZE_128_BYTE;
  else if (size <= 256)
    size = USBHS_DEVEPTCFG_EPSIZE_256_BYTE;
  else if (size <= 512)
    size = USBHS_DEVEPTCFG_EPSIZE_512_BYTE;
  else if (size <= 1024)
    size = USBHS_DEVEPTCFG_EPSIZE_1024_BYTE;
  else
    while (1);

  if (USB_CONTROL_ENDPOINT == type)
    type = USBHS_DEVEPTCFG_EPTYPE_CTRL;
  else if (USB_ISOCHRONOUS_ENDPOINT == type)
    type = USBHS_DEVEPTCFG_EPTYPE_ISO;
  else if (USB_BULK_ENDPOINT == type)
    type = USBHS_DEVEPTCFG_EPTYPE_BLK;
  else
    type = USBHS_DEVEPTCFG_EPTYPE_INTRPT;

  if (USB_IN_ENDPOINT == dir)
    dir = USBHS_DEVEPTCFG_EPDIR_IN;
  else
    dir = USBHS_DEVEPTCFG_EPDIR_OUT;

  USBHS->USBHS_DEVEPTCFG[ep] = type | size | dir | USBHS_DEVEPTCFG_EPBK_1_BANK | 
      USBHS_DEVEPTCFG_NBTRANS_1_TRANS | USBHS_DEVEPTCFG_ALLOC;

  while (0 == (USBHS->USBHS_DEVEPTISR[ep] & USBHS_DEVEPTISR_CFGOK));

  USBHS->USBHS_DEVEPT |= (USBHS_DEVEPT_EPEN0 << ep);
  USBHS->USBHS_DEVIER |= (USBHS_DEVIER_PEP_0 << ep);
}

//-----------------------------------------------------------------------------
bool usb_endpoint_configured(int ep, int dir)
{
  (void)dir;
  return (0 != (USBHS->USBHS_DEVEPT & (USBHS_DEVEPT_EPEN0 << ep)));
}

//-----------------------------------------------------------------------------
int usb_endpoint_get_status(int ep, int dir)
{
  (void)dir;
  return (0 != (USBHS->USBHS_DEVEPTIMR[ep] & USBHS_DEVEPTIMR_STALLRQ));
}

//-----------------------------------------------------------------------------
void usb_endpoint_set_feature(int ep, int dir)
{
  USBHS->USBHS_DEVEPTICR[ep] = USBHS_DEVEPTICR_STALLEDIC;
  USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_STALLRQS;
  USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_RSTDTS;
  (void)dir;
}

//-----------------------------------------------------------------------------
void usb_endpoint_clear_feature(int ep, int dir)
{
  if (USBHS->USBHS_DEVEPTIMR[ep] & USBHS_DEVEPTIMR_STALLRQ)
  {
    if (USBHS->USBHS_DEVEPTISR[ep] & USBHS_DEVEPTISR_STALLEDI)
    {
      USBHS->USBHS_DEVEPTICR[ep] = USBHS_DEVEPTICR_STALLEDIC;
      USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_RSTDTS;
    }

    USBHS->USBHS_DEVEPTIDR[ep] = USBHS_DEVEPTIDR_STALLRQC;
  }

  (void)dir;
}

//-----------------------------------------------------------------------------
void usb_set_address(int address)
{
  USBHS->USBHS_DEVCTRL &= ~USBHS_DEVCTRL_ADDEN;
  USBHS->USBHS_DEVCTRL &= ~USBHS_DEVCTRL_UADD_Msk;
  USBHS->USBHS_DEVCTRL |= address;
  USBHS->USBHS_DEVCTRL |= USBHS_DEVCTRL_ADDEN;
}

//-----------------------------------------------------------------------------
void usb_send(int ep, uint8_t *data, int size)
{
  int ch = ep-1;

  USBHS->UsbhsDevdma[ch].USBHS_DEVDMANXTDSC  = 0;
  USBHS->UsbhsDevdma[ch].USBHS_DEVDMAADDRESS = (uint32_t)data;
  USBHS->UsbhsDevdma[ch].USBHS_DEVDMACONTROL = USBHS_DEVDMACONTROL_CHANN_ENB | USBHS_DEVDMACONTROL_BUFF_LENGTH(size);
  while (0 == (USBHS->UsbhsDevdma[ch].USBHS_DEVDMASTATUS & USBHS_DEVDMASTATUS_END_BF_ST));

  USBHS->USBHS_DEVEPTICR[ep] = USBHS_DEVEPTICR_TXINIC;
  USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_TXINES;
  USBHS->USBHS_DEVEPTIDR[ep] = USBHS_DEVEPTIDR_FIFOCONC;
}

//-----------------------------------------------------------------------------
void usb_recv(int ep, uint8_t *data, int size)
{
  usb_recv_req[ep].data = data;
  usb_recv_req[ep].size = size;
  USBHS->USBHS_DEVEPTIER[ep] = USBHS_DEVEPTIER_RXOUTES;
}

//-----------------------------------------------------------------------------
void usb_control_send_zlp(void)
{
  USBHS->USBHS_DEVEPTICR[0] = USBHS_DEVEPTICR_TXINIC;
  USBHS->USBHS_DEVEPTIER[0] = USBHS_DEVEPTIER_TXINES;

  while (0 == (USBHS->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_TXINI));
}

//-----------------------------------------------------------------------------
void usb_control_stall(void)
{
  USBHS->USBHS_DEVEPTIER[0] = USBHS_DEVEPTIER_STALLRQS;
}

//-----------------------------------------------------------------------------
void usb_control_send(uint8_t *data, int size)
{
  volatile uint8_t *usb_ram = (uint8_t *)USBHS_RAM_ADDR;

  while (size)
  {
    int transfer_size = LIMIT(size, usb_device_descriptor.bMaxPacketSize0);

    for (int i = 0; i < transfer_size; i++)
      usb_ram[i] = data[i];

    __DSB();
    __DMB();

    USBHS->USBHS_DEVEPTICR[0] = USBHS_DEVEPTICR_TXINIC;
    USBHS->USBHS_DEVEPTIER[0] = USBHS_DEVEPTIER_TXINES;
    while (0 == (USBHS->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_TXINI));

    size -= transfer_size;
    data += transfer_size;
  }
}

//-----------------------------------------------------------------------------
void usb_control_recv(void (*callback)(uint8_t *data, int size))
{
  usb_control_recv_callback = callback;
  USBHS->USBHS_DEVEPTIER[0] = USBHS_DEVEPTIER_RXOUTES;
}

//-----------------------------------------------------------------------------
void usb_task(void)
{
  if (USBHS->USBHS_DEVISR & USBHS_DEVISR_EORST)
  {
    usb_endpoint_descriptor_t desc;

    usb_set_address(0);

    for (int i = 0; i < USB_EP_NUM; i++)
      usb_reset_endpoint(i, 0);

    desc.bEndpointAddress = 0;
    desc.bmAttributes     = USB_CONTROL_ENDPOINT;
    desc.wMaxPacketSize   = USB_CONTROL_ENDPOINT_SIZE;

    usb_configure_endpoint(&desc);

    USBHS->USBHS_DEVEPTIER[0] = USBHS_DEVEPTIER_RXSTPES;

    USBHS->USBHS_DEVICR = USBHS_DEVICR_EORSTC;
  }

  if (USBHS->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_RXSTPI)
  {
    int size = ((USBHS->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_BYCT_Msk) >> USBHS_DEVEPTISR_BYCT_Pos);
    volatile uint8_t *usb_ram = (uint8_t *)USBHS_RAM_ADDR;
    uint8_t setup_data[USB_CONTROL_ENDPOINT_SIZE];

    if (size == sizeof(usb_request_t))
    {
      for (int i = 0; i < size; i++)
        setup_data[i] = usb_ram[i];

      USBHS->USBHS_DEVEPTICR[0] = USBHS_DEVEPTICR_RXSTPIC;

      if (!usb_handle_standard_request((usb_request_t *)setup_data))
        usb_control_stall();
    }
    else
    {
      USBHS->USBHS_DEVEPTICR[0] = USBHS_DEVEPTICR_RXSTPIC;
      usb_control_stall();
    }
  }

  if (USBHS->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_RXOUTI)
  {
    int size = ((USBHS->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_BYCT_Msk) >> USBHS_DEVEPTISR_BYCT_Pos);
    volatile uint8_t *usb_ram = (uint8_t *)USBHS_RAM_ADDR;
    uint8_t data[USB_CONTROL_ENDPOINT_SIZE];

    if (size > USB_CONTROL_ENDPOINT_SIZE)
      size = USB_CONTROL_ENDPOINT_SIZE;

    for (int i = 0; i < size; i++)
      data[i] = usb_ram[i];

    USBHS->USBHS_DEVEPTICR[0] = USBHS_DEVEPTICR_RXOUTIC;
    USBHS->USBHS_DEVEPTIDR[0] = USBHS_DEVEPTIDR_RXOUTEC;

    if (usb_control_recv_callback)
    {
      usb_control_recv_callback(data, size);
      usb_control_recv_callback = NULL;
      usb_control_send_zlp();
    }
  }

  for (int ep = 1; ep < USB_EP_NUM; ep++)
  {
    //if (0 == USBHS->USBHS_DEVISR & (USBHS_DEVISR_PEP_0 << i))
    //  continue;

    if ((USBHS->USBHS_DEVEPTISR[ep] & USBHS_DEVEPTISR_RXOUTI) &&
        (USBHS->USBHS_DEVEPTIMR[ep] & USBHS_DEVEPTIMR_RXOUTE))
    {
      int size = ((USBHS->USBHS_DEVEPTISR[ep] & USBHS_DEVEPTISR_BYCT_Msk) >> USBHS_DEVEPTISR_BYCT_Pos);
      int ch = ep-1;

      if (size > usb_recv_req[ep].size)
        size = usb_recv_req[ep].size;

      USBHS->UsbhsDevdma[ch].USBHS_DEVDMANXTDSC  = 0;
      USBHS->UsbhsDevdma[ch].USBHS_DEVDMAADDRESS = (uint32_t)usb_recv_req[ep].data;
      USBHS->UsbhsDevdma[ch].USBHS_DEVDMACONTROL = USBHS_DEVDMACONTROL_CHANN_ENB | USBHS_DEVDMACONTROL_BUFF_LENGTH(size);
      while (0 == (USBHS->UsbhsDevdma[ch].USBHS_DEVDMASTATUS & USBHS_DEVDMASTATUS_END_BF_ST));

      USBHS->USBHS_DEVEPTICR[ep] = USBHS_DEVEPTICR_RXOUTIC;
      USBHS->USBHS_DEVEPTIDR[ep] = USBHS_DEVEPTIDR_RXOUTEC;

      usb_recv_callback(ep, size);

      USBHS->USBHS_DEVEPTIDR[ep] = USBHS_DEVEPTIDR_FIFOCONC;
    }

    if ((USBHS->USBHS_DEVEPTISR[ep] & USBHS_DEVEPTISR_TXINI) &&
        (USBHS->USBHS_DEVEPTIMR[ep] & USBHS_DEVEPTIMR_TXINE))
    {
      USBHS->USBHS_DEVEPTICR[ep] = USBHS_DEVEPTICR_TXINIC;
      USBHS->USBHS_DEVEPTIDR[ep] = USBHS_DEVEPTIDR_TXINEC;

      usb_send_callback(ep);
    }
  }
}

