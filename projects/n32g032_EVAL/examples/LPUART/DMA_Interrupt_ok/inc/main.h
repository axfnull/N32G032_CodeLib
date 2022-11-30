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

#define _LPUART1_USART1_

#ifdef _LPUART1_USART1_
#define LPUARTy                 LPUART1
#define LPUARTy_CLK             RCC_LPUART1CLK
#define LPUARTy_RCC_CLK         RCC_APB1_PERIPH_LPUART1
#define LPUARTy_GPIO            GPIOA
#define LPUARTy_GPIO_CLK        RCC_APB2_PERIPH_GPIOA
#define LPUARTy_RxPin           GPIO_PIN_0
#define LPUARTy_TxPin           GPIO_PIN_1
#define LPUARTy_Rx_GPIO_AF      GPIO_AF11_LPUART1
#define LPUARTy_Tx_GPIO_AF      GPIO_AF5_LPUART1
#define LPUARTy_DAT_Base        (LPUART1_BASE + 0x10)
#define LPUARTy_Tx_DMA_Channel  DMA_CH1
#define LPUARTy_Tx_DMA_FLAG     DMA_FLAG_TC1
#define LPUARTy_Rx_DMA_Channel  DMA_CH2
#define LPUARTy_Rx_DMA_FLAG     DMA_FLAG_TC2
#define LPUARTy_Tx_DMA_REMAP    DMA_REMAP_LPUART1_TX
#define LPUARTy_Rx_DMA_REMAP    DMA_REMAP_LPUART1_RX
#define LPUARTy_IRQn            LPUART1_2_IRQn
#define LPUARTy_IRQHandler      LPUART1_2_IRQHandler

#define USARTz                  USART1
#define USARTz_GPIO             GPIOA
#define USARTz_CLK              RCC_APB2_PERIPH_USART1
#define USARTz_GPIO_CLK         RCC_APB2_PERIPH_GPIOA
#define USARTz_RxPin            GPIO_PIN_10
#define USARTz_TxPin            GPIO_PIN_9
#define USARTz_Rx_GPIO_AF       GPIO_AF4_USART1
#define USARTz_Tx_GPIO_AF       GPIO_AF4_USART1
#define USARTz_APBxClkCmd       RCC_EnableAPB2PeriphClk
#define USARTz_DAT_Base         (USART1_BASE + 0x04)
#define USARTz_Tx_DMA_Channel   DMA_CH4
#define USARTz_Tx_DMA_FLAG      DMA_FLAG_TC4
#define USARTz_Rx_DMA_Channel   DMA_CH5
#define USARTz_Rx_DMA_FLAG      DMA_FLAG_TC5
#define USARTz_Tx_DMA_REMAP     DMA_REMAP_USART1_TX
#define USARTz_Rx_DMA_REMAP     DMA_REMAP_USART1_RX
#endif

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(uint32_t LPUART_CLK_SRC);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Delay(__IO uint32_t nCount);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
