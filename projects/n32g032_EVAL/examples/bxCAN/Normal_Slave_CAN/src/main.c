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
#include "main.h"
#include <stdio.h>
/**
 *  Normal_Slave_CAN
 */

#define DEMO_USART_BAUDRATE ((uint32_t)115200)

#define CAN_BAUDRATE_1M   ((uint32_t)1000)
#define CAN_BAUDRATE_500K ((uint32_t)500)
#define CAN_BAUDRATE_250K ((uint32_t)250)
#define CAN_BAUDRATE_125K ((uint32_t)125)
#define CAN_BAUDRATE_100K ((uint32_t)100)
#define CAN_BAUDRATE_50K  ((uint32_t)50)
#define CAN_BAUDRATE_20K  ((uint32_t)20)
#define CAN_BAUDRATE_10K  ((uint32_t)10)
#define CAN_BTR_CALCULATE ((uint32_t)8000)

#define TEST_BUFFER_SIZE  8
#define CAN_TXDLC_8    TEST_BUFFER_SIZE
#define CAN_FILTERNUM0  0
#define CAN_FILTERNUM1  1

CanTxMessage CAN_TxMessage;
CanRxMessage CAN_RxMessage;
uint8_t rx_flag = 0;


/**
 * @brief  Main program.
 */
int main(void)
{
    uint8_t TransmitMailbox = 0;
    uint16_t Time_out       = 0xFFFF;

    /* USART Init */
    USART_Config();
    printf(" Normal_Slave_CAN Start\r\n");

    /* Configures the NVIC for CAN_RX0 */
    NVIC_Config();

    /* Configures CAN IOs */
    CAN_GPIO_Config();

    /* Configures CAN*/
    CAN_Config(CAN_BAUDRATE_500K);

    /* CAN transmit message */
    
      /* check if receive data */
    while (!rx_flag) 
    {
            
    }
    TransmitMailbox = CANTxMessage(CAN,
                                   &CAN_TxMessage,
                                   0x0000,
                                   0x1311,
                                   CAN_ID_EXT,
                                   CAN_RTRQ_DATA,
                                   CAN_TXDLC_8,
                                   CAN_RxMessage.Data);
    /* Waiting to send successfully */
    while ((CAN_TransmitSTS(CAN, TransmitMailbox) != CANTXSTSOK) && (Time_out != 0))
    {
        Time_out--;
    }

    

    while (1)
    {
    }
}

/**
 * @brief  Configures the NVIC for CAN.
 */
void NVIC_Config(void)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel                   = CAN_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures CAN GPIOs
 */
void CAN_GPIO_Config(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configures CAN IOs */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA, ENABLE);

    /* Configure CAN RX pin */
    GPIO_InitStructure.Pin       = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF9_CAN;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    /* Configure CAN TX pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF9_CAN;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  Configures CAN.
 * @param CAN_BaudRate 10Kbit/s ~ 1Mbit/s
 */
void CAN_Config(uint32_t CAN_BaudRate)
{
    CAN_InitType CAN_InitStructure;
    CAN_FilterInitType CAN_FilterInitStructure;

    /* Configure CAN */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN, ENABLE);

    /* CAN register init */
    CAN_DeInit(CAN);

    /* Struct init*/
    CAN_InitStruct(&CAN_InitStructure);

    /* CAN cell init */
    CAN_InitStructure.TTCM          = DISABLE;
    CAN_InitStructure.ABOM          = DISABLE;
    CAN_InitStructure.AWKUM         = DISABLE;
    CAN_InitStructure.NART          = DISABLE;
    CAN_InitStructure.RFLM          = DISABLE;
    CAN_InitStructure.TXFP          = DISABLE;
    CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
    CAN_InitStructure.RSJW          = CAN_RSJW_1tq;
    CAN_InitStructure.TBS1          = CAN_TBS1_3tq;
    CAN_InitStructure.TBS2          = CAN_TBS2_2tq;

    if ((CAN_BaudRate > CAN_BAUDRATE_1M) || (CAN_BaudRate < CAN_BAUDRATE_10K))
    {
        printf("Please select the CAN Baudrate between 10Kbit/s to 1Mbit/s.\r\n");
    }
    else
    {
        CAN_InitStructure.BaudRatePrescaler = (uint32_t)(CAN_BTR_CALCULATE / CAN_BaudRate);
    }

    /*Initializes the CAN */
    CAN_Init(CAN, &CAN_InitStructure);

    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM1;
    CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId         = 0x0000;
    CAN_FilterInitStructure.Filter_LowId          = (((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTRQ_DATA)&0xFFFF;
    CAN_FilterInitStructure.FilterMask_HighId     = 0xFFFF;
    CAN_FilterInitStructure.FilterMask_LowId      = 0xFFFF;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    CAN_InitFilter(&CAN_FilterInitStructure);

    CAN_INTConfig(CAN, CAN_INT_FMP0, ENABLE);
}

/**
 * @brief  CAN Transmit Message.
 * @param  CAN
 * @param  TxMessage CAN_TxMessage
 * @param  StdId
 * @param  ExtId
 * @param  IDE
 * @param  RTR
 * @param  DLC
 * @param  Data0~7
 * @return The number of the mailbox that is used for transmission or CAN_TxSTS_NoMailBox if there is no empty mailbox.
 */
uint8_t CANTxMessage(CAN_Module* CANx,
                     CanTxMessage* TxMessage,
                     uint32_t StdId,
                     uint32_t ExtId,
                     uint8_t IDE,
                     uint8_t RTR,
                     uint8_t DLC,
                     uint8_t* data)
{
    uint8_t i;
    /* Transmit */
    TxMessage->StdId   = StdId; /* Standard ID or Extended ID(MSBs) */
    TxMessage->ExtId   = ExtId; /* Extended ID(LSBs) */
    TxMessage->IDE     = IDE;   /* CAN_ID_STD / CAN_ID_EXT */
    TxMessage->RTR     = RTR;   /* CAN_RTRQ_DATA / CAN_RTRQ_REMOTE */
    TxMessage->DLC     = DLC;   /* 0 to 8 */
    for (i = 0; i < 8; i++)
    {
        TxMessage->Data[i] = data[i];
    }
    return CAN_TransmitMessage(CANx, TxMessage);
    /* ******** */
}

/**
 * @brief  Check Can Receive Message.
 * @param RxMessage CAN_TxMessage
 * @param  StdId
 * @param  ExtId
 * @param  IDE
 * @param  RTR
 * @param  DLC
 * @param  Data0~7
 * @param FMI Filter match index
 * @return FAILED/PASSED
 */
uint8_t Check_CANRxMessage(CanRxMessage* RxMessage,
                           uint32_t StdId,
                           uint32_t ExtId,
                           uint8_t IDE,
                           uint8_t RTR,
                           uint8_t DLC,
                           uint8_t Data0,
                           uint8_t Data1,
                           uint8_t Data2,
                           uint8_t Data3,
                           uint8_t Data4,
                           uint8_t Data5,
                           uint8_t Data6,
                           uint8_t Data7,
                           uint8_t FMI)
{
    /* ID */
    if (IDE == CAN_ID_EXT)
    {
        if (RxMessage->ExtId != ExtId)
        {
            return FAILED;
        }
    }
    else if (IDE == CAN_ID_STD)
    {
        if (RxMessage->StdId != StdId)
        {
            return FAILED;
        }
    }
    /* **** */

    /* IDE/RTR/DLC */
    if ((RxMessage->IDE != IDE) || /* CAN_ID_STD / CAN_ID_EXT */
        (RxMessage->RTR != RTR) || /* CAN_RTRQ_DATA / CAN_RTRQ_REMOTE */
        (RxMessage->DLC != DLC)    /* 0 to 8 */
    )
    {
        return FAILED;
    }
    /* **** */

    /* DATA */
    if (RTR == CAN_RTRQ_DATA)
    {
        if (DLC >= 1)
        {
            if (RxMessage->Data[0] != Data0)
            {
                return FAILED;
            }
        }
        if (DLC >= 2)
        {
            if (RxMessage->Data[1] != Data1)
            {
                return FAILED;
            }
        }
        if (DLC >= 3)
        {
            if (RxMessage->Data[2] != Data2)
            {
                return FAILED;
            }
        }
        if (DLC >= 4)
        {
            if (RxMessage->Data[3] != Data3)
            {
                return FAILED;
            }
        }
        if (DLC >= 5)
        {
            if (RxMessage->Data[4] != Data4)
            {
                return FAILED;
            }
        }
        if (DLC >= 6)
        {
            if (RxMessage->Data[5] != Data5)
            {
                return FAILED;
            }
        }
        if (DLC >= 7)
        {
            if (RxMessage->Data[6] != Data6)
            {
                return FAILED;
            }
        }
        if (DLC == 8)
        {
            if (RxMessage->Data[7] != Data7)
            {
                return FAILED;
            }
        }
        if (DLC > 8)
        {
            return FAILED;
        }
    }
    else if (RTR == CAN_RTRQ_REMOTE)
    {
    }
    /* **** */

    /* RTR/DLC */
    if (RxMessage->FMI != FMI)
    {
        return FAILED;
    }
    /* **** */
    return PASSED;
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

    USART_InitStructure.BaudRate            = DEMO_USART_BAUDRATE;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_Init(USART1, &USART_InitStructure);

    USART_Enable(USART1, ENABLE);
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
