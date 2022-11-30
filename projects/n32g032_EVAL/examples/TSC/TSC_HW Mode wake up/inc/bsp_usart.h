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
 * @file bsp_usart.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"

/*************************** Debug uart config*********************************/
/*base*/
#define DEBUG_USARTx                (LPUART1)
#define DEBUG_USART_CLK             (RCC_APB1_PERIPH_LPUART1)
#define DEBUG_USART_APBxClkCmd      RCC_EnableAPB1PeriphClk
#define DEBUG_USART_BAUDRATE        (115200)

/*port*/
#define DEBUG_USART_GPIO_CLK        (RCC_APB2_PERIPH_GPIOC)
#define DEBUG_USART_GPIO_APBxClkCmd RCC_EnableAPB2PeriphClk

#define DEBUG_USART_GPIO_REMAP      (GPIO_AF4_LPUART1)

#define DEBUG_USART_TX_GPIO_PORT GPIOC
#define DEBUG_USART_TX_GPIO_PIN  GPIO_PIN_10
#define DEBUG_USART_RX_GPIO_PORT GPIOC
#define DEBUG_USART_RX_GPIO_PIN  GPIO_PIN_11

/*irq, reserved*/
#define DEBUG_USART_IRQ        LPUART1_2_IRQn
//#define DEBUG_USART_IRQHandler LPUART1_2_IRQHandler

/*************************** Debug option**************************************/
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

#if LOG_ENABLE

#include <stdio.h>

#define LOG_NONE        0
#define LOG_ERROR       10
#define LOG_WARNING     20
#define LOG_INFO        30
#define LOG_DEBUG       40

#ifndef LOG_LEVEL
#define LOG_LEVEL       LOG_INFO
#endif

#if LOG_LEVEL >= LOG_INFO
#define log_info(...)       printf(__VA_ARGS__)
#else
#define log_info(...)
#endif

#if LOG_LEVEL >= LOG_ERROR
#define log_error(...)      printf(__VA_ARGS__)
#else
#define log_error(...)
#endif

#if LOG_LEVEL >= LOG_WARNING
#define log_warning(...)    printf(__VA_ARGS__)
#else
#define log_warning(...)
#endif

#if LOG_LEVEL >= LOG_DEBUG
#define log_debug(...)      printf(__VA_ARGS__)
#else
#define log_debug(...)
#endif

extern uint8_t DebugUartInited;
void log_init(void);

#else /* !LOG_ENABLE */

#define log_info(...)
#define log_warning(...)
#define log_error(...)
#define log_debug(...)
#define log_init()

#endif

#define log_func() log_debug("call %s\r\n", __FUNCTION__)

/*************************** Common function***********************************/
void Debug_USART_Config(void);
void Debug_USART_DeInit(void);

void LPUart_SendByte(LPUART_Module* pUSARTx, uint8_t ch);
void LPUart_SendString(LPUART_Module* pUSARTx, char* str);
void LPUart_SendHalfWord(LPUART_Module* pUSARTx, uint16_t ch);

#endif /* __BSP_USART_H__ */
