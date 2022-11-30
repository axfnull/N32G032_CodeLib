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
#include "n32g032.h"
#include "n32g032_i2c.h"
#include "log.h"
/** @addtogroup N32G032_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2C_Slave_Int
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
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    0xA0
volatile Status test_status = FAILED;

uint8_t transmit_buf[TEST_BUFFER_SIZE] = {0};
uint8_t flag_slave_recv_finish         = 0;
uint8_t flag_slave_send_finish         = 0;
uint8_t flag_trans_direct              = 0; // 0:recv, 1:send
uint8_t flag_overrun                   = 0;
uint8_t rx_num                         = 0;
uint8_t tx_num                         = 0;
uint16_t data_len                      = 0;

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Memset(void* s, uint8_t c, uint32_t count);

static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;

/**
 * @brief  i2c slave Interrupt configuration
 * @param ch I2C channel
 */
void NVIC_ConfigurationSlave(uint8_t ch)
{
    NVIC_InitType NVIC_InitStructure;
    if (ch == 1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
    }
    if (ch == 2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
    }
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  i2c slave Interrupt service function
 */
void I2C1_IRQHandler(void)
{
    uint32_t last_event;

    last_event = I2C_GetLastEvent(I2C1);
    if ((last_event & 0x00010000) != 0x00010000) // MSMODE = 0:I2C slave mode
    {
        switch (last_event)
        {
        case I2C_EVT_SLAVE_RECV_ADDR_MATCHED: //0x00020002.EV1 Rx addr matched
            
            rx_num = 0;
            last_event = 0;
            Memset(transmit_buf, (uint8_t)0, TEST_BUFFER_SIZE); // clear buf, ready to recv data
            break;
        
        case I2C_EVT_SLAVE_SEND_ADDR_MATCHED: //0x00060082.EV1 Tx addr matched
            
            tx_num = 0;
            last_event = 0;
            I2C1->DAT = transmit_buf[tx_num++]; // Send first data
            break;
        
        // SlaveTransmitter    
        case I2C_EVT_SLAVE_DATA_SENDING:  //0x00060080. EV3 Sending data
            if (tx_num < TEST_BUFFER_SIZE)
            {
                I2C1->DAT = transmit_buf[tx_num++];
            }
            else if (tx_num == TEST_BUFFER_SIZE)
            {
                flag_slave_send_finish = 1;
            }
            //last_event = 0;
            break;
            
        case I2C_EVT_SLAVE_DATA_SENDED:
            break;
        
        // SlaveReceiver
        case I2C_EVT_SLAVE_DATA_RECVD: //0x00020040.EV2 one byte recved
        case I2C_EVT_SLAVE_DATA_RECVD_BSF://0x00020044:    EV2  When the I2C communication rate is too high, BSF = 1
            transmit_buf[rx_num++] = I2C1->DAT;
            if(rx_num == TEST_BUFFER_SIZE)
            {
                flag_slave_recv_finish = 1; 
            }
            break;
        
        case I2C_EVT_SLAVE_STOP_RECVD: // 0x00000010 EV4
            I2C_Enable(I2C1, ENABLE);   
            break;
        
        default:
            break;
        }
    }
}

/**
 * @brief  i2c slave init
 * @return 0:init finish
 */
int i2c_slave_init(void)
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
    i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_2;
    i2c1_master.OwnAddr1    = I2C_SLAVE_ADDR;
    i2c1_master.AckEnable   = I2C_ACKEN;
    i2c1_master.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed    = 100000;

    I2C_Init(I2C1, &i2c1_master);
    // int enable
    I2C_ConfigInt(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
    NVIC_ConfigurationSlave((uint8_t)1);
        
#ifdef I2C_SLAVE_LOW_LEVEL  //I2C work in low level(for example:1.8V) 
    AFIO_EnableI2CLV(AFIO_I2C1_LV, ENABLE);
#endif    

    I2C_Enable(I2C1, ENABLE);
    return 0;
}

/**
 * @brief   Main program
 */
int main(void)
{
    log_init();
    log_info("this is a i2c slave int demo\r\n");

    /* Initialize the I2C slave driver ----------------------------------------*/
    i2c_slave_init();
    while (1)
    {
        // recive data
        if (flag_slave_recv_finish == 1) // recv data finish
        {
            flag_slave_recv_finish = 0;
            data_len               = rx_num;
            log_info("recv finish,recv len = %02x:\r\n", data_len);
        }
        // send data
        if (flag_slave_send_finish == 1)
        {
            flag_slave_send_finish = 0;
            data_len = tx_num;
            tx_num = 0;
            log_info("tx finish,tx len = %02x:\r\n", data_len);
        }
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
 * @brief memery set a value
 * @param s source
 * @param c value
 * @param count number
 * @return pointer of the memery
 */
void Memset(void* s, uint8_t c, uint32_t count)
{
    char* xs = (char*)s;

    while (count--) // clear 17byte buffer
    {
        *xs++ = c;
    }

    return;
}

/**
 * @}
 */
