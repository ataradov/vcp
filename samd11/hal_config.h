// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2017-2022, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _HAL_CONFIG_H_
#define _HAL_CONFIG_H_

/*- Includes ----------------------------------------------------------------*/
#include "samd11.h"
#include "hal_gpio.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(VCP_STATUS,         A, 4);
HAL_GPIO_PIN(BOOT_ENTER,         A, 31);
HAL_GPIO_PIN(UART_TX,            A, 14);
HAL_GPIO_PIN(UART_RX,            A, 8);

#define UART_SERCOM              SERCOM0
#define UART_SERCOM_PMUX_RX      PORT_PMUX_PMUXE_D_Val
#define UART_SERCOM_PMUX_TX      PORT_PMUX_PMUXE_C_Val
#define UART_SERCOM_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define UART_SERCOM_APBCMASK     PM_APBCMASK_SERCOM0
#define UART_SERCOM_IRQ_INDEX    SERCOM0_IRQn
#define UART_SERCOM_IRQ_HANDLER  irq_handler_sercom0
#define UART_SERCOM_TXPO         0
#define UART_SERCOM_RXPO         2

#endif // _HAL_CONFIG_H_
