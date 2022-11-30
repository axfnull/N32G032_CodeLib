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
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
/**
 *  BEEPER_Output
 */

/**
 * @brief  Main program.
 */
int main(void)
{
    /* USART Init */
    USART_Config();
    printf("BEEPER1/2 Output Test Start\r\n");

    /* BEEPER Output */

    /* Enable BEEPER1/2 Clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_BEEP1 | RCC_APB1_PERIPH_BEEP2, ENABLE);

    /* Config BEEPER1/2 output GPIO */
    BEEPER_GPIO_Config();

    /* Init BEEPER1/2 */
    BEEPER_Init(BEEPER1, BEEPER_INV_DISABLE, BEEPER_BUZZER_CLKSEL_L1);
    BEEPER_Init(BEEPER2, BEEPER_INV_ENABLE, BEEPER_BUZZER_CLKSEL_M1);

    /* Enable BEEPER1/2 */
    BEEPER_Config(BEEPER1, ENABLE);
    BEEPER_Config(BEEPER2, ENABLE);    

    printf("BEEPER1/2 Output Test End\r\n");

    while (1)
    {
    }
}

/**
 * @brief  USART_Config.
 */
void USART_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);

    GPIO_InitStructure.Pin        = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin       = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_Init(USART1, &USART_InitStructure);

    USART_Enable(USART1, ENABLE);
}

/**
 * @brief  BEEPER_GPIO_Config.
 */
void BEEPER_GPIO_Config(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);

    /* BEEPER1 out */
    GPIO_InitStructure.Pin             = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF11_BEEPER;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    /* BEEPER2 out */
    GPIO_InitStructure.Pin             = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF11_BEEPER;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    /* BEEPER2 inverter out */
    GPIO_InitStructure.Pin             = GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF11_BEEPER;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @}
 */

/**
 * @brief  Retargets the C library printf function to the USART1.
 * @param
 * @return
 */
int fputc(int ch, FILE* f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) == RESET)
        ;
    return (ch);
}

/*  */
/**
 * @brief  Retargets the C library scanf function to the USART1.
 * @param
 * @return
 */
int fgetc(FILE* f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) == RESET)
        ;
    return (int)USART_ReceiveData(USART1);
}

/**
 * @}
 */
