/*
 * Copyright (c) 2019, Alex Taradov <alex@taradov.com>
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

#ifndef _NVM_DATA_H_
#define _NVM_DATA_H_

/*- Definitions -------------------------------------------------------------*/
#define NVM_ADC_BIASREFBUF_POS       0
#define NVM_ADC_BIASREFBUF_SIZE      3

#define NVM_ADC_BIASCOMP_POS         3
#define NVM_ADC_BIASCOMP_SIZE        3

#define NVM_OSC32K_CAL_POS           6
#define NVM_OSC32K_CAL_SIZE          7

#define NVM_USB_TRANSN_POS           13
#define NVM_USB_TRANSN_SIZE          5

#define NVM_USB_TRANSP_POS           18
#define NVM_USB_TRANSP_SIZE          5

#define NVM_USB_TRIM_POS             23
#define NVM_USB_TRIM_SIZE            3

#define NVM_DFLL48M_COARSE_CAL_POS   26
#define NVM_DFLL48M_COARSE_CAL_SIZE  6

#define NVM_READ_CAL(cal) \
    ((*((uint32_t *)NVMCTRL_OTP5 + cal##_POS / 32)) >> (cal##_POS % 32)) & ((1 << cal##_SIZE) - 1)

#endif // _NVM_DATA_H_

