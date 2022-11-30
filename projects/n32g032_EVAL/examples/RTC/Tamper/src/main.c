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
#include <stdint.h>
#include "log.h"
#include "n32g032_rtc.h"
#include "n32g032.h"
/** @addtogroup RTC_TAMPER
 * @{
 */

RTC_InitType RTC_InitStructure;
uint32_t SynchPrediv, AsynchPrediv;

void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);
void LEDInit(void);
void LedOn(uint16_t Pin);
void LedOff(uint16_t Pin);
void RTC_CLKSourceConfig(uint8_t ClkSrc, uint8_t FirstLastCfg, uint8_t RstBKP);
static void RTC_PrescalerConfig(void);
void TAMPER_INT_Configuration(FunctionalState cmd);
void TamperInputIoInit(void);
/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_n32g032_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_n32g032.c file
       */
    /* Initialize USART,TX: PC10 RX: PC11*/
    log_init();
    log_info("RTC Init");
    /* RTC clock source select 1:HSE/128 2:LSE 3:LSI*/
    RTC_CLKSourceConfig(3, 0, 1);
    RTC_PrescalerConfig();
    /*Tamper GPIO Initialize,PC13*/
    TamperInputIoInit();
    /*Configure Tamper GPIO trigger mode*/
    RTC_TamperTriggerConfig(RTC_TAMPER_1, RTC_TamperTrigger_FallingEdge);
    /*Configure the valib number of scanning times*/
    RTC_TamperFilterConfig(RTC_TamperFilter_Disable);
    /*Configure scanning frequence*/
    RTC_TamperSamplingFreqConfig(RTC_TamperSamplingFreq_RTCCLK_Div256);
    /*Configure precharge duration time*/
    RTC_TamperPinsPrechargeDuration(RTC_TamperPrechargeDuration_1RTCCLK);
    /*Configure tamper interruput,EXTI_LINE19*/
    TAMPER_INT_Configuration(ENABLE);
    /*Enable tamper interruput*/
    RTC_TamperIECmd(RTC_TAMPER1_INT, ENABLE);
    /*Enable tamper1 */
    RTC_TamperCmd(RTC_TAMPER_1, ENABLE);                    
    while (1)
    {

    }
}


/**
 * @brief  Configure the tamper interruput,EXTI19.
 */
 void TAMPER_INT_Configuration(FunctionalState cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    
    EXTI_ClrITPendBit(EXTI_LINE19);
    EXTI_InitStructure.EXTI_Line                          = EXTI_LINE19;
    EXTI_InitStructure.EXTI_Mode                          = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger                       = EXTI_Trigger_Rising; 

    EXTI_InitStructure.EXTI_LineCmd                       = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
    
    /* Enable the RTC Alarm Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                    = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority  = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = cmd;
    NVIC_Init(&NVIC_InitStructure);
}
/**
 * @brief  Toggles the selected Led.
 * @param Led Specifies the Led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}
/**
 * @brief  Turns selected Led on.
 * @param Led Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LedOn(uint16_t Pin)
{
    GPIOB->PBC = Pin;
}
/**
 * @brief  Turns selected Led Off.
 * @param Led Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 */
void LedOff(uint16_t Pin)
{
    GPIOB->PBSC = Pin;
}
/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 */

void LEDInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* Enable the GPIO_LED Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;

    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 */

void TamperInputIoInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* Enable the GPIO_LED Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC|RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_RTC;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
}


/**
 * @brief  RTC prescaler config.
 */
static void RTC_PrescalerConfig(void)
{
    /* Configure the RTC data register and RTC prescaler */
    RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
    RTC_InitStructure.RTC_SynchPrediv  = SynchPrediv;
    RTC_InitStructure.RTC_HourFormat   = RTC_24HOUR_FORMAT;

    /* Check on RTC init */
    if (RTC_Init(&RTC_InitStructure) == ERROR)
    {
        log_info("\r\n //******* RTC Prescaler Config failed **********// \r\n");
    }
}

/**
 * @brief  Configures RTC.
 *   Configure the RTC peripheral by selecting the clock source
 */
void RTC_CLKSourceConfig(uint8_t ClkSrc, uint8_t FirstLastCfg, uint8_t RstBKP)
{
//    assert_param(IS_CLKSRC_VALUE(ClkSrc));
//    assert_param(IS_FLCFG_VALUE(FirstLastCfg));

    /* Enable the PWR clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);


    /* Disable RTC clock */
    RCC_EnableRtcClk(DISABLE);

    if (ClkSrc == 0x01)
    {
        log_info("\r\n RTC_ClkSrc Is Set HSE128! \r\n");
        if (FirstLastCfg == 0)
        {
            /* Enable HSE */
            RCC_EnableLsi(DISABLE);
            RCC_ConfigHse(RCC_HSE_ENABLE);
            while (RCC_WaitHseStable() == ERROR)
            {
            }

            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_HSE_DIV128);
        }
        else
        {
            RCC_EnableLsi(DISABLE);
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_HSE_DIV128);

            /* Enable HSE */
            RCC_ConfigHse(RCC_HSE_ENABLE);

            while (RCC_WaitHseStable() == ERROR)
            {
            }
        }

        SynchPrediv  = 0x1E8; // 8M/128 = 62.5KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else if (ClkSrc == 0x02)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSE! \r\n");

        if (FirstLastCfg == 0)
        {
            /* Enable the LSE OSC32_IN PC14 */
            RCC_EnableLsi(DISABLE); // LSI is turned off here to ensure that only one clock is turned on

#if (_TEST_LSE_BYPASS_)
            RCC_ConfigLse(RCC_LSE_BYPASS);
#else
            RCC_ConfigLse(RCC_LSE_ENABLE);
#endif

            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSERD) == RESET)
            {
            }

            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);
        }
        else
        {
            /* Enable the LSE OSC32_IN PC14 */
            RCC_EnableLsi(DISABLE);
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);

#if (_TEST_LSE_BYPASS_)
            RCC_ConfigLse(RCC_LSE_BYPASS);
#else
            RCC_ConfigLse(RCC_LSE_ENABLE);
#endif

            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSERD) == RESET)
            {
            }
        }

        SynchPrediv  = 0xFF; // 32.768KHz
        AsynchPrediv = 0x7F; // value range: 0-7F
    }
    else if (ClkSrc == 0x03)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSI! \r\n");
        if (FirstLastCfg == 0)
        {
            /* Enable the LSI OSC */
            RCC_EnableLsi(ENABLE);

            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
            {
            }

            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);
        }
        else
        {
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);

            /* Enable the LSI OSC */
            RCC_EnableLsi(ENABLE);

            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
            {
            }
        }

        SynchPrediv  = 0xEA; // 30KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else
    {
        log_info("\r\n RTC_ClkSrc Value is error! \r\n");
    }

    /* Enable the RTC Clock */
    RCC_EnableRtcClk(ENABLE);
    RTC_WaitForSynchro();
}
