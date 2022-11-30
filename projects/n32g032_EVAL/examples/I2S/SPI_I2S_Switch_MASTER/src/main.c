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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"

/** @addtogroup N32G032_StdPeriph_Examples
 * @{
 */

/** @addtogroup SPI_I2S_Switch
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define BufferSize 32

I2S_InitType I2S_InitStructure;
SPI_InitType SPI_InitStructure;
uint16_t I2S_Buffer_Tx[BufferSize] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
                                       0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
                                       0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                       0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40};

uint16_t SPI_Buffer_Tx[BufferSize] = {0x5152, 0x5354, 0x5556, 0x5758, 0x595A, 0x5B5C, 0x5D5E, 0x5F60,
                                       0x6162, 0x6364, 0x6566, 0x6768, 0x696A, 0x6B6C, 0x6D6E, 0x6F70,
                                       0x7172, 0x7374, 0x7576, 0x7778, 0x797A, 0x7B7C, 0x7D7E, 0x7F80,
                                       0x8182, 0x8384, 0x8586, 0x8788, 0x898A, 0x8B8C, 0x8D8E, 0x8F90};

__IO uint16_t I2S_Buffer_Rx[BufferSize];
__IO uint16_t TxIdx = 0, RxIdx = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;
volatile TestStatus TransferStatus3 = FAILED;

void RCC_Configuration(void);
void GPIO_Configuration(void);
									   

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_n32g032.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_n32g032.c file
       */

    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* Deinitializes the I2S_MASTER peripheral registers --------------------*/
    SPI_I2S_DeInit(I2S_MASTER);

    /* I2S peripheral configuration */
    I2S_InitStructure.Standard       = I2S_STD_PHILLIPS;
    I2S_InitStructure.DataFormat     = I2S_DATA_FMT_16BITS_EXTENDED;
    I2S_InitStructure.AudioFrequency = I2S_AUDIO_FREQ_48K;
    I2S_InitStructure.CLKPOL         = I2S_CLKPOL_LOW;

    /* I2S Master Transmitter to I2S Slave Receiver communication ------------*/
    /* I2S_MASTER configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_MASTER_TX;
    I2S_Init(I2S_MASTER, &I2S_InitStructure);

    /* Enable the I2S_MASTER */
    I2S_Enable(I2S_MASTER, ENABLE);

    /* Begin the communication in I2S_MASTER mode */
    while (TxIdx < BufferSize)
    {
        /* Wait the Tx buffer to be empty */
        while (SPI_I2S_GetStatus(I2S_MASTER, SPI_I2S_TE_FLAG) == RESET)
        {
        }
        /* Send a data from I2S_MASTER */
        SPI_I2S_TransmitData(I2S_MASTER, I2S_Buffer_Tx[TxIdx++]);

    }

    /* delay sometimes */
    for (RxIdx = 0; RxIdx < 1024; RxIdx++)
    {

    }
     /* Reset TxIdx, RxIdx indexes */
    TxIdx = 0;
    RxIdx = 0;
    /* Switch to SPI_MASTER mode communication ----------------------------------------*/
    /* SPI_MASTER configuration */
    SPI_InitStructure.DataDirection = SPI_DIR_SINGLELINE_TX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_16BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_16;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(I2S_MASTER, &SPI_InitStructure);

    /* Enable SPI_MASTER */
    SPI_Enable(SPI_MASTER, ENABLE);

    /* Begin the communication in SPI_MASTER mode */
    while (TxIdx < BufferSize)
    {
        /* Wait the Tx buffer to be empty */
        while (SPI_I2S_GetStatus(SPI_MASTER, SPI_I2S_TE_FLAG) == RESET)
        {
        }
        /* Send a data from SPI_MASTER */
        SPI_I2S_TransmitData(SPI_MASTER, SPI_Buffer_Tx[TxIdx++]);
    }

    /* Reset TxIdx, RxIdx indexes and receive table values */
    for (TxIdx = 0; TxIdx < BufferSize; TxIdx++)
    {
        I2S_Buffer_Rx[TxIdx] = 0;
    }

    TxIdx = 0;
    RxIdx = 0;

    /* Slave Transmitter to Master Receiver communication ------------*/
    /* I2S1 configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_SlAVE_TX;
    I2S_Init(I2S_MASTER, &I2S_InitStructure);

    /* Wait the Tx buffer to be empty */
    while (SPI_I2S_GetStatus(I2S_MASTER, SPI_I2S_TE_FLAG) == RESET)
    {
    }
    /* Prepare the first data to be sent from the slave */
    SPI_I2S_TransmitData(I2S_MASTER, I2S_Buffer_Tx[TxIdx++]);

    /* Enable the I2S1 */
    I2S_Enable(I2S_MASTER, ENABLE);

    /* Begin the communication in I2S mode */
    while (TxIdx < BufferSize)
    {
        /* Wait the Tx buffer to be empty */
        while (SPI_I2S_GetStatus(I2S_MASTER, SPI_I2S_TE_FLAG) == RESET)
        {
        }
        /* Send a data from I2S1 */
        SPI_I2S_TransmitData(I2S_MASTER, I2S_Buffer_Tx[TxIdx++]);
    }

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks --------------------------------------------------*/
    /* GPIOA and AFIO clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* SPI1 clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1 , ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure I2S_MASTER pins: CK, WS and SD ---------------------------------*/
    GPIO_InitStructure.Pin        = I2S_MASTER_PIN_CK | I2S_MASTER_PIN_SD | I2S_MASTER_PIN_WS;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_I2S1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
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
