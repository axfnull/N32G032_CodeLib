/*****************************************************************************
 * Copyright (c) 2019, ..\readme.txt Technologies Inc.
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
 * ..\readme.txt' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ..\readme.txt "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ..\readme.txt BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.h
 * @author ..\readme.txt
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, ..\readme.txt Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g032.h"

#define _USART1_UART6_
// #define _USART2_UART5_

#ifdef _USART1_UART6_
#define USARTy              USART1
#define USARTy_CLK          RCC_APB2_PERIPH_USART1
#define USARTy_GPIO         GPIOA
#define USARTy_GPIO_CLK     RCC_APB2_PERIPH_GPIOA
#define USARTy_RxPin        GPIO_PIN_10
#define USARTy_TxPin        GPIO_PIN_9
#define USARTy_Rx_GPIO_AF   GPIO_AF4_USART1
#define USARTy_Tx_GPIO_AF   GPIO_AF4_USART1
#define USARTy_APBxClkCmd   RCC_EnableAPB2PeriphClk

#define USARTz              UART6
#define USARTz_CLK          RCC_APB2_PERIPH_UART6
#define USARTz_GPIO         GPIOA
#define USARTz_GPIO_CLK     RCC_APB2_PERIPH_GPIOA
#define USARTz_RxPin        GPIO_PIN_5
#define USARTz_TxPin        GPIO_PIN_4
#define USARTz_Rx_GPIO_AF   GPIO_AF6_UART6
#define USARTz_Tx_GPIO_AF   GPIO_AF6_UART6
#define USARTz_APBxClkCmd   RCC_EnableAPB2PeriphClk
#endif

#ifdef _USART2_UART5_
#define USARTy              UART5
#define USARTy_CLK          RCC_APB1_PERIPH_UART5
#define USARTy_GPIO         GPIOB
#define USARTy_GPIO_CLK     RCC_APB2_PERIPH_GPIOB
#define USARTy_RxPin        GPIO_PIN_1
#define USARTy_TxPin        GPIO_PIN_0
#define USARTy_Rx_GPIO_AF   GPIO_AF6_UART5
#define USARTy_Tx_GPIO_AF   GPIO_AF5_UART5
#define USARTy_APBxClkCmd   RCC_EnableAPB1PeriphClk

#define USARTz              USART2
#define USARTz_CLK          RCC_APB1_PERIPH_USART2
#define USARTz_GPIO         GPIOA
#define USARTz_GPIO_CLK     RCC_APB2_PERIPH_GPIOA
#define USARTz_RxPin        GPIO_PIN_10
#define USARTz_TxPin        GPIO_PIN_9
#define USARTz_Rx_GPIO_AF   GPIO_AF10_USART2
#define USARTz_Tx_GPIO_AF   GPIO_AF10_USART2
#define USARTz_APBxClkCmd   RCC_EnableAPB1PeriphClk
#endif

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(void);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
