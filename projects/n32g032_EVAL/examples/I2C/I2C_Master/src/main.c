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
#include "n32g032.h"
#include "n32g032_i2c.h"
#include "log.h"

/** @addtogroup N32G032_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2C_Master
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;

//#define I2C_MASTER_LOW_LEVEL

#define TEST_BUFFER_SIZE  100
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(20 * I2C_FLAG_TIMOUT))
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    0xA0

uint8_t tx_buf[TEST_BUFFER_SIZE] = {0};
uint8_t rx_buf[TEST_BUFFER_SIZE] = {0};
volatile Status test_status      = FAILED;
uint32_t stsbuf[4];
Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;
void delay(uint32_t count)
{
    uint32_t i;
    for(i = 0; i< count; i++)
    {
    }
}
int i2c_master_init(void)
{
    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C1, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

    GPIO_InitStruct(&i2c1_gpio);
    /*PB6 -- SCL; PB7 -- SDA*/
    i2c1_gpio.Pin        = GPIO_PIN_6 | GPIO_PIN_7;
    i2c1_gpio.GPIO_Speed = GPIO_SPEED_HIGH; 
    i2c1_gpio.GPIO_Mode  = GPIO_MODE_AF_OD;
    i2c1_gpio.GPIO_Alternate = GPIO_AF6_I2C1;
    i2c1_gpio.GPIO_Pull = GPIO_PULL_UP; 
    GPIO_InitPeripheral(GPIOB, &i2c1_gpio);

    I2C_DeInit(I2C1);
    i2c1_master.BusMode     = I2C_BUSMODE_I2C;
    i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_2; //if the spped greater than 400KHz, the FmDutyCycle mast be configured to I2C_FMDUTYCYCLE_2
    i2c1_master.OwnAddr1    = I2C_MASTER_ADDR;
    i2c1_master.AckEnable   = I2C_ACKEN;
    i2c1_master.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed    = 100000;

    I2C_Init(I2C1, &i2c1_master);
#ifdef I2C_MASTER_LOW_LEVEL  //I2C work in low level(for example:1.8V) 
    AFIO_EnableI2CLV(AFIO_I2C1_LV, ENABLE);
#endif
    I2C_Enable(I2C1, ENABLE);
    return 0;
}

int i2c_master_send(uint8_t* data, int len)
{
    uint8_t* sendBufferPtr = data;
    I2CTimeout             = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(I2C1, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
            return 4;
    };

    I2C_ConfigAck(I2C1, ENABLE);

    I2C_GenerateStart(I2C1, ENABLE);
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    {
        if ((I2CTimeout--) == 0)
            return 5;
    };

    I2C_SendAddr7bit(I2C1, I2C_SLAVE_ADDR, I2C_DIRECTION_SEND);
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_TXMODE_FLAG)) // EV6
    {
        if ((I2CTimeout--) == 0)
            return 6;
    };

    // send data
    while (len-- > 0)
    {
        I2C_SendData(I2C1, *sendBufferPtr++);
        I2CTimeout = I2C_FLAG_TIMOUT;
        while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_DATA_SENDING)) // EV8
        {
            if ((I2CTimeout--) == 0)
                return 7;
        };
    };

    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_DATA_SENDED)) // EV8-2
    {
        if ((I2CTimeout--) == 0)
            return 8;
    };
    // Delay_us(8);
    I2C_GenerateStop(I2C1, ENABLE);
    return 0;
}

int i2c_master_recv(uint8_t* data, int len)
{
    uint8_t* recvBufferPtr = data;
    I2CTimeout             = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(I2C1, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
            return 9;
    };

    I2C_ConfigAck(I2C1, ENABLE);

    // send start
    I2C_GenerateStart(I2C1, ENABLE);
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    {
        if ((I2CTimeout--) == 0)
            return 10;
    };

    // send addr
    I2C_SendAddr7bit(I2C1, I2C_SLAVE_ADDR, I2C_DIRECTION_RECV);
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_RXMODE_FLAG)) // EV6
    {
        if ((I2CTimeout--) == 0)
            return 6;
    };

    // recv data
    while (len-- > 0)
    {
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(I2C1, I2C_EVT_MASTER_DATA_RECVD_FLAG)) // EV7
        {
            if ((I2CTimeout--) == 0)
            return 14;
        };
        if (len == 1)
        {
            I2C_ConfigAck(I2C1, DISABLE);
        }
        if (len == 0)
        {
            I2C_GenerateStop(I2C1, ENABLE);
        }
        *recvBufferPtr++ = I2C_RecvData(I2C1);

    };
    return 0;
}

/**
 * @brief   Main program
 */
int main(void)
{
    uint16_t i = 0;

    log_init();
    log_info("this is a i2c master demo\r\n");
    /* Initialize the I2C EEPROM driver ----------------------------------------*/
    i2c_master_init();

    /* Fill the buffer to send */
    for (i = 0; i < TEST_BUFFER_SIZE; i++)
    {
        tx_buf[i] = i;
    }
    /* First write in the memory followed by a read of the written data --------*/
    /* Write data*/
    i2c_master_send(tx_buf, TEST_BUFFER_SIZE);

    /* Read data */
    i2c_master_recv(rx_buf, TEST_BUFFER_SIZE);

    /* Check if the data written to the memory is read correctly */
    test_status = Buffercmp(tx_buf, rx_buf, TEST_BUFFER_SIZE);
    if (test_status == PASSED)
    {
        log_info("write and read data the same, i2c master test pass\r\n");
        log_info("recv = ");
        for (i = 0; i < TEST_BUFFER_SIZE; i++)
        {
            log_info("%02x", rx_buf[i]);
        }
        log_info("\r\n");
    }
    else
    {
        log_info("write and read data are different, i2c master test fail\r\n");
    }
    /* test_status = PASSED, if the write and read data are the same  */
    /* test_status = FAILED, if the write and read data are different */
    while (1)
    {
    }
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
Status Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

/**
 * @}
 */
