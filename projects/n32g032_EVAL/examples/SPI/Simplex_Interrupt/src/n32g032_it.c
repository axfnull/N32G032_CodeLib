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
 * @file n32g032_it.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g032_it.h"
#include "main.h"

/** @addtogroup N32G032_StdPeriph_Template
 * @{
 */

#define BufferSize 32
extern __IO uint8_t TxIdx, RxIdx;
extern uint8_t MASTER_Buffer_Tx[BufferSize], SLAVE_Buffer_Rx[BufferSize];

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            N32g032 Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
 * @brief  This function handles SPI1 global interrupt request.
 */

void SPI1_IRQHandler(void)
{
    if (SPI_I2S_GetIntStatus(SPI_MASTER, SPI_I2S_INT_TE) != RESET)
    {
        /* Send SPI_MASTER data */
        SPI_I2S_TransmitData(SPI_MASTER, MASTER_Buffer_Tx[TxIdx++]);

        /* Disable SPI_MASTER TE interrupt */
        if (TxIdx == BufferSize)
        {
            SPI_I2S_EnableInt(SPI_MASTER, SPI_I2S_INT_TE, DISABLE);
        }
    }
}

/**
 * @brief  This function handles SPI2 global interrupt request.
 */
void SPI2_3_IRQHandler(void)
{
    if(SPI_I2S_GetIntStatus(SPI_SLAVE, SPI_I2S_INT_RNE) != RESET)
    {
        /* Store SPI_SLAVE received data */
        SLAVE_Buffer_Rx[RxIdx++] = SPI_I2S_ReceiveData(SPI_SLAVE);//SPI_SLAVE
    }
}

/******************************************************************************/
/*                 N32G032 Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_n32g032.s).                                                 */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
