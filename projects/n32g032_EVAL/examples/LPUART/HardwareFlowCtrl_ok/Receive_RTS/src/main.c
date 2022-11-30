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
 * @file main.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <stdio.h>
#include "main.h"

/** @addtogroup N32G032_StdPeriph_Examples
 * @{
 */

/** @addtogroup LPUART_Receive_RTS
 * @{
 */


#define TxBufferSize1 (countof(TxBuffer1))

#define countof(a) (sizeof(a) / sizeof(*(a)))

LPUART_InitType LPUART_InitStructure;
uint8_t TxBuffer1[] = "LPUART Flow_Control Mode Example: Board1_LPUARTy -> Board2_LPUARTz using CTS and RTS Flags";
uint8_t RxBuffer1[TxBufferSize1];
__IO uint8_t RxCounter             = 0;
volatile TestStatus TransferStatus = FAILED;

/**
 * @brief  Main program
 */
int main(void)
{
    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* System Clocks Configuration */   
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1); 

    /* LPUARTz configuration ------------------------------------------------------*/    
    LPUART_DeInit(LPUARTz);
    LPUART_InitStructure.BaudRate            = 9600;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_RTS;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX;
    /* Configure LPUARTz */
    LPUART_Init(LPUARTz, &LPUART_InitStructure);

    while (RxCounter < TxBufferSize1)
    {
        /* Loop until the Board2_LPUARTz Receive Data Register is not empty */
        while (LPUART_GetFlagStatus(LPUARTz, LPUART_FLAG_FIFO_NE) == RESET)
        {
        }

        // Delay(0x2ffff);

        /* Store the received byte in RxBuffer */
        RxBuffer1[RxCounter++] = LPUART_ReceiveData(LPUARTz);
    }

    /* Check the received data with the send ones */
    /* TransferStatus = PASSED, if the data transmitted from Board1_LPUARTy and
       received by Board2_LPUARTz are the same */
    /* TransferStatus = FAILED, if the data transmitted from Board1_LPUARTy and
       received by Board2_LPUARTz are different */
    TransferStatus = Buffercmp(TxBuffer1, RxBuffer1, TxBufferSize1);

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 * @param  LPUART_CLK_SRC: specifies the LPUARTz clock source.
 */
void RCC_Configuration(uint32_t LPUART_CLK_SRC)
{
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTz_GPIO_CLK, ENABLE);

    switch(LPUART_CLK_SRC)
    {
        case RCC_LPUARTCLK_SRC_HSI:
        {
            /* Configures the High Speed Internal RC clock (HSI) */
            RCC_EnableHsi(ENABLE);
            while (RCC_WaitHsiStable() != SUCCESS)
            {
            }
            /* Specifies the LPUARTy clock source, HSI selected as LPUART clock */
            RCC_ConfigLPUARTClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_HSI);
        }
        break;
        case RCC_LPUARTCLK_SRC_HSE:
        {
            /* Configures the External High Speed oscillator (HSE) */
            RCC_ConfigHse(RCC_HSE_ENABLE);
            while (RCC_WaitHseStable() != SUCCESS)
            {
            }
            /* Specifies the LPUARTy clock source, HSE selected as LPUARTy clock */
            RCC_ConfigLPUARTClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_HSE);
        }
        break;
        case RCC_LPUARTCLK_SRC_LSI:
        {
            /* Enables the Internal Low Speed oscillator (LSI) */
            RCC_EnableLsi(ENABLE);
            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
            {
            }
            /* Specifies the LPUARTy clock source, LSI selected as LPUARTy clock */
            RCC_ConfigLPUARTClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_LSI);
        }
        break;
        case RCC_LPUARTCLK_SRC_LSE:
        {  
            /* Configures the External Low Speed oscillator (LSE) */
            RCC_ConfigLse(RCC_LSE_ENABLE);
            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSERD) == RESET)
            {
            }
            /* Specifies the LPUARTy clock source, LSE selected as LPUARTy clock */
            RCC_ConfigLPUARTClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_LSE);
        }
        break;        
        case RCC_LPUARTCLK_SRC_SYSCLK:
        {
            /* Specifies the LPUARTy clock source, SYSCLK selected as LPUARTy clock */
            RCC_ConfigLPUARTClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_SYSCLK);
        }
        break;
        default:
        {
            /* Specifies the LPUARTy clock source, APB1 selected as LPUARTy clock */
            RCC_ConfigLPUARTClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }    
    
    /* Enable LPUARTz Clock */
    RCC_EnableAPB1PeriphClk(LPUARTz_RCC_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTz_GPIO_CLK, ENABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure LPUARTz RTS as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTz_RTSPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTz_RTS_GPIO_AF;
    GPIO_InitPeripheral(LPUARTz_GPIO, &GPIO_InitStructure);

    /* Configure LPUARTz Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTz_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTz_Rx_GPIO_AF;    
    GPIO_InitPeripheral(LPUARTz_GPIO, &GPIO_InitStructure);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}

/**
 * @brief  Inserts a delay time.
 * @param nCount specifies the delay time length.
 */
void Delay(__IO uint32_t nCount)
{
    /* Decrement nCount value */
    for (; nCount != 0; nCount--)
        ;
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */
