/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g032.h"

#define _LPUART1_COM_  

#ifdef _LPUART1_COM_
#define LPUARTz             LPUART1
#define LPUARTz_CLK         RCC_LPUART1CLK
#define LPUARTz_RCC_CLK     RCC_APB1_PERIPH_LPUART1
#define LPUARTz_GPIO        GPIOB
#define LPUARTz_CTS_GPIO    GPIOA
#define LPUARTz_GPIO_CLK    RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOA 
#define LPUARTz_RxPin       GPIO_PIN_7
#define LPUARTz_TxPin       GPIO_PIN_6
#define LPUARTz_RTSPin      GPIO_PIN_1
#define LPUARTz_CTSPin      GPIO_PIN_6
#define LPUARTz_Rx_GPIO_AF  GPIO_AF9_LPUART1
#define LPUARTz_Tx_GPIO_AF  GPIO_AF3_LPUART1
#define LPUARTz_RTS_GPIO_AF GPIO_AF4_LPUART1
#define LPUARTz_CTS_GPIO_AF GPIO_AF5_LPUART1
#endif

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(uint32_t LPUART_CLK_SRC);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Delay(__IO uint32_t nCount);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
