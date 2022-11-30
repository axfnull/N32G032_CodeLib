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
 * @file bsp_usart.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "bsp_usart.h"

#if LOG_ENABLE
    uint8_t DebugUartInited = 0;
#endif
/**
 * @brief  Configure nested vector interrupt controller NVIC.
 */
//static void NVIC_Configuration(void)
//{
//    NVIC_InitType NVIC_InitStructure;

//    /* Select nested vector interrupt controller group */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

//    /* Configure USART as interrupt source */
//    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
//    /*Set the priority*/
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    /*Set the sub priority */
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    /*Enable interrupt */
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    /* Initialize configuration NVIC */
//    NVIC_Init(&NVIC_InitStructure);
//}

/**
 * @brief  Configure parameters of usart port.
 */
void Debug_USART_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    LPUART_InitType LPUART_InitStructure;

    // Turn on the clock of usart port GPIO
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

    // Turn on the clock of usart peripheral
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

#ifdef  DEBUG_USART_GPIO_REMAP
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO,ENABLE);
#endif

    // Configure GPIO of USART TX as push pull multiplexing mode
    GPIO_InitStructure.Pin              = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed       = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current     = GPIO_DC_HIGH;
    GPIO_InitStructure.GPIO_Alternate   = DEBUG_USART_GPIO_REMAP;
    GPIO_InitStructure.GPIO_Pull        = GPIO_NO_PULL;
    GPIO_InitPeripheral(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    // Configure GPIO of USART RX as floating input mode
    GPIO_InitStructure.Pin              = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitPeripheral(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    RCC_ConfigLPUARTClk(RCC_LPUART1CLK,RCC_LPUARTCLK_SRC_SYSCLK);
    
    // Configure parameters of usart port
    // Configure baud rate
    LPUART_InitStructure.BaudRate = DEBUG_USART_BAUDRATE;
    // Configure the RTS Threshold
    LPUART_InitStructure.RtsThreshold = LPUART_RTSTH_FIFOFU;
    // Configure check bit
    LPUART_InitStructure.Parity = LPUART_PE_NO;
    // Configure hardware flow control
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    // Configure working mode, send and receive together
    LPUART_InitStructure.Mode = LPUART_MODE_TX;
    // Complete initialization configuration of usart port
    LPUART_Init(DEBUG_USARTx, &LPUART_InitStructure);

    // Configuration interrupt priority of the usart port
//    NVIC_Configuration();

    // Enable usart port receive interrupt
//    LPUART_ConfigInt(DEBUG_USARTx, LPUART_INT_FIFO_NE, ENABLE);

    // Enable usart
    LPUART1->CTRL |= LPUART_CTRL_TXEN;

#if LOG_ENABLE
    DebugUartInited = 1;
#endif
}

/**
 * @brief  Disable and reset debug uart.
 */
void Debug_USART_DeInit(void)
{
    GPIO_DeInitPin(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_RX_GPIO_PIN|DEBUG_USART_TX_GPIO_PIN);

    LPUART_DeInit(DEBUG_USARTx);
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK,DISABLE);
}

/*****************  Send one byte data **********************/
void LPUart_SendByte(LPUART_Module* pUSARTx, uint8_t ch)
{
   /* Clear transmission complete flag */
    LPUART_ClrFlag(pUSARTx, LPUART_FLAG_TXC);
    
    /* Send one byte data to usart */
    LPUART_SendData(pUSARTx, ch);

    /* Waiting to send data register is empty */
    while (LPUART_GetFlagStatus(pUSARTx, LPUART_FLAG_TXC) == RESET);
}

/****************** Send an array of 8 bit************************/
void LPUart_SendArray(LPUART_Module* pUSARTx, uint8_t* array, uint16_t num)
{
    uint8_t i;

    for (i = 0; i < num; i++)
    {
        /* Send one byte data to usart */
        LPUart_SendByte(pUSARTx, array[i]);
    }
}

/*****************  Send string **********************/
void LPUart_SendString(LPUART_Module* pUSARTx, char* str)
{
    unsigned int k = 0;
    do
    {
        LPUart_SendByte(pUSARTx, *(str + k));
        k++;
    } while (*(str + k) != '\0');
}

/*****************  Send a 16 bits number **********************/
void LPUart_SendHalfWord(LPUART_Module* pUSARTx, uint16_t ch)
{
    uint8_t temp_h, temp_l;

    /* Take out the high byte */
    temp_h = (ch & 0XFF00) >> 8;
    /* Take out the loe byte */
    temp_l = ch & 0XFF;

    /* Send the high byte */
    LPUart_SendByte(pUSARTx, temp_h);

    /* Send the low byte */
    LPUart_SendByte(pUSARTx, temp_l);
}

// Redirect C library printf function to USART port.After rewriting, printf function can be used.
static int is_lr_sent = 0;
int fputc(int ch, FILE* f)
{
    uint8_t tchar = (uint8_t)ch;
    
    if (tchar == '\r')
    {
        is_lr_sent = 1;
    }
    else if (tchar == '\n')
    {
        if (!is_lr_sent)
        {
            tchar = '\r';
        }

        is_lr_sent = 0;
    }
    else
    {
        is_lr_sent = 0;
    }

    LPUart_SendByte(DEBUG_USARTx, tchar);

    return (ch);
}


// Redirect C library scanf function to USART port.After rewriting, functions such as scanf, getchar can be used.
int fgetc(FILE* f)
{
    /* Waiting for usart port input data */
    while (LPUART_GetFlagStatus(DEBUG_USARTx, LPUART_FLAG_FIFO_NE) == RESET);

    return (int)LPUART_ReceiveData(DEBUG_USARTx);
}

