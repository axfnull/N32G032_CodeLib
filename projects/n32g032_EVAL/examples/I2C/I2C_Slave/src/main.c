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

/** @addtogroup I2C_Slave
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;
//#define I2C_SLAVE_LOW_LEVEL
#define TEST_BUFFER_SIZE  100
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(10 * I2C_FLAG_TIMOUT))
#define I2C_SLAVE_ADDR    0xA0

uint8_t data_buf[TEST_BUFFER_SIZE] = {0};
volatile Status test_status        = FAILED;

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;

/**
 * @brief  I2C slave init
 *
 * @return 0:init finish
 */
int i2c_slave_init(void)
{
    I2C_InitType i2c1_slave;
    GPIO_InitType i2c1_gpio;
    // enable clk
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C1, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_InitStruct(&i2c1_gpio); 
    /*PB6 -- SCL; PB7 -- SDA*/
    i2c1_gpio.Pin        = GPIO_PIN_6 | GPIO_PIN_7;
    i2c1_gpio.GPIO_Speed = GPIO_SPEED_HIGH;
    i2c1_gpio.GPIO_Mode  = GPIO_MODE_AF_OD;
    i2c1_gpio.GPIO_Alternate = GPIO_AF6_I2C1;
    GPIO_InitPeripheral(GPIOB, &i2c1_gpio);

    I2C_DeInit(I2C1);
    i2c1_slave.BusMode     = I2C_BUSMODE_I2C;
    i2c1_slave.FmDutyCycle = I2C_FMDUTYCYCLE_2;   //if the spped greater than 400KHz, the FmDutyCycle mast be configured to I2C_FMDUTYCYCLE_2
    i2c1_slave.OwnAddr1    = I2C_SLAVE_ADDR;
    i2c1_slave.AckEnable   = I2C_ACKEN;
    i2c1_slave.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_slave.ClkSpeed    = 100000;

    I2C_Init(I2C1, &i2c1_slave);
#ifdef I2C_SLAVE_LOW_LEVEL     //I2C work in low level(for example:1.8V) 
    AFIO_EnableI2CLV(AFIO_I2C1_LV, ENABLE); 
#endif
    I2C_Enable(I2C1, ENABLE);
    return 0;
}

/**
 * @brief  I2C slave send data
 *
 * @param data data buffer
 * @param len send data len
 * @return 0:send finish
 */
int i2c_slave_send(uint8_t* data, int len)
{
    uint8_t* sendBufferPtr = data;
    uint32_t tx_index      = 0;
    I2CTimeout             = I2CT_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_SLAVE_SEND_ADDR_MATCHED))
        ; // send addr matched

    // send data
    while (tx_index < (len - 1))
    {
        I2C_SendData(I2C1, *sendBufferPtr++);
        I2CTimeout = I2C_FLAG_TIMOUT;
        while (!I2C_CheckEvent(I2C1, I2C_EVT_SLAVE_DATA_SENDED))
        {
            if ((I2CTimeout--) == 0)
                return 7;
        };
        tx_index++;
    };

    // send last data and clear EV3
    I2C_SendData(I2C1, *sendBufferPtr);
    while (!I2C_CheckEvent(I2C1, I2C_EVT_SLAVE_ACK_MISS))
        ; // EV3-2
    /*Clear I2C AF flag */
    I2C_ClrFlag(I2C1, I2C_FLAG_ACKFAIL); // EV3-2:AF=1,write 0 to STS1 AF bit to clear AF flag

    return 0;
}

/**
 * @brief  I2C slave receive data
 *
 * @param data data buffer,use to save received data
 * @param rcv_len receive data len
 * @return 0:recv finish
 */
int i2c_slave_recv(uint8_t* data, uint32_t rcv_len)
{
    //uint8_t* recvBufferPtr = data;
    uint32_t rx_index      = 0;
    I2CTimeout             = I2CT_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVT_SLAVE_RECV_ADDR_MATCHED))
        ; // receive addr matched

    // recv data
    while (rx_index < rcv_len)
    {
        while (!I2C_CheckEvent(I2C1, I2C_EVT_SLAVE_DATA_RECVD))
            ;
        *data = I2C_RecvData(I2C1);
        rx_index++;
        data++;
    };

    while (!I2C_CheckEvent(I2C1, I2C_EVT_SLAVE_STOP_RECVD))
        ; // EV4
    return 0;
}

/**
 * @brief   Main program
 */
int main(void)
{
    uint16_t i = 0;

    log_init();
    log_info("this is a i2c slave test demo\r\n");
    /* Initialize the I2C EEPROM driver ----------------------------------------*/
    i2c_slave_init();

    /* Read data */
    log_info("i2c slave recv data start\r\n");
    i2c_slave_recv(data_buf, TEST_BUFFER_SIZE);
    log_info("recv = ");
    for (i = 0; i < TEST_BUFFER_SIZE; i++)
    {
        log_info("%02x", data_buf[i]);
    }
    log_info("\r\n");

    /* Write data*/
    log_info("i2c slave send data start\r\n");
    i2c_slave_send(data_buf, TEST_BUFFER_SIZE);
    log_info("i2c slave test pass\r\n");
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
