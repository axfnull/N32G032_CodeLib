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
/** @addtogroup RTC_Calendar
 * @{
 */
RTC_DateType RTC_DateStructure;
RTC_DateType RTC_DateDefault;
RTC_TimeType RTC_TimeStructure;
RTC_TimeType RTC_TimeDefault;
RTC_InitType RTC_InitStructure;
RTC_AlarmType RTC_AlarmStructure;
uint32_t SynchPrediv, AsynchPrediv;

void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);
void LEDInit(void);
void LedOn(uint16_t Pin);
void LedOff(uint16_t Pin);
void RTC_CLKSourceConfig(uint8_t ClkSrc, uint8_t FirstLastCfg, uint8_t RstBKP);
ErrorStatus RTC_TimeRegulate(void);
ErrorStatus RTC_DateRegulate(void);
void RTC_DateAndTimeDefaultVale(void);
static void RTC_PrescalerConfig(void);
void EXTI20_RTCWKUP_Configuration(FunctionalState Cmd);
void RTC_TimeShow(void);
void RTC_DateShow(void);
void WakeUpClockSelect(uint8_t WKUPClkSrcSel);
void RTCOutputIoInit(void);

/**
 * @brief  Main program.
 */
int main(void)
{
    uint8_t WKUPOutIrq   = 2;

    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_n32g43x_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_n32g43x.c file
       */
    /* Initialize LEDs on n32g43x-EVAL board */
    LEDInit();
    LedOff(LED1_PIN);
    /* Initialize USART,TX: PC10 RX: PC11*/
    log_init();
    log_info("RTC Init");
    /* RTC date time alarm default value*/
    RTC_DateAndTimeDefaultVale();

    /* RTC clock source select 1:HSE/128 2:LSE 3:LSI*/
    RTC_CLKSourceConfig(3, 0, 1);
    RTC_PrescalerConfig();
    log_info("\r\n RTC configured....");

    RTCOutputIoInit();
    RTC_ConfigOutput(RTC_OUTPUT_WKUP, RTC_OUTPOL_HIGH);
    /* Adjust time by values entered by the user on the hyperterminal */
    RTC_DateRegulate();
    RTC_TimeRegulate();
    /* wake up clock select */
    WakeUpClockSelect(5);
    /* wake up timer value */
    RTC_SetWakeUpCounter(4);

    if (WKUPOutIrq == 0x02) // 1.WakeUp interrupt test
    {
        EXTI20_RTCWKUP_Configuration(ENABLE);
        /* Enable the RTC Wakeup Interrupt */
        RTC_ConfigInt(RTC_INT_WUT, ENABLE);
        RTC_EnableWakeUp(ENABLE);
        while (1)
            ;
    }
    else if (WKUPOutIrq == 0x01) // 2.WakeUp output test
    {
        RTC_EnableWakeUp(ENABLE);
        while (1)
        {
            if (RTC_GetFlagStatus(RTC_FLAG_WTF) == SET)
            {
                RTC_ClrFlag(RTC_FLAG_WTF);
            }
        }
    }
}

/**
 * @brief  config rtc output io.
 */
void RTCOutputIoInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* Enable the GPIO_LED Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC|RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull  = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF3_RTC;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
}

/**
 * @brief  Wake up clock config.
 */
void WakeUpClockSelect(uint8_t WKUPClkSrcSel)
{
    /* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
    if (WKUPClkSrcSel == 0x01)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV16);
    else if (WKUPClkSrcSel == 0x02)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV8);
    else if (WKUPClkSrcSel == 0x03)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV4);
    else if (WKUPClkSrcSel == 0x04)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV2);
    else if (WKUPClkSrcSel == 0x05)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_16BITS);
    else if (WKUPClkSrcSel == 0x06)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_17BITS);
}
/**
 * @brief  Config RTC wake up Interrupt.
 */
void EXTI20_RTCWKUP_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    EXTI_ClrITPendBit(EXTI_LINE20);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE20;
#ifdef __TEST_SEVONPEND_WFE_NVIC_DIS__
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#else
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* Enable the RTC WakeUp Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
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
 * @brief  Display the current Date on the Hyperterminal.
 */
void RTC_DateShow(void)
{
    /* Get the current Date */
    RTC_GetDate(RTC_FORMAT_BIN, &RTC_DateStructure);
    log_info("\n\r //=========== Current Date Display ==============// \n\r");
    log_info("\n\r The current date (WeekDay-Date-Month-Year) is :  %0.2d-%0.2d-%0.2d-%0.2d \n\r",
             RTC_DateStructure.WeekDay,
             RTC_DateStructure.Date,
             RTC_DateStructure.Month,
             RTC_DateStructure.Year);
}

/**
 * @brief  Display the current time on the Hyperterminal.
 */
void RTC_TimeShow(void)
{
    /* Get the current Time and Date */
    RTC_GetTime(RTC_FORMAT_BIN, &RTC_TimeStructure);
    log_info("\n\r //============ Current Time Display ===============// \n\r");
    log_info("\n\r The current time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \n\r",
             RTC_TimeStructure.Hours,
             RTC_TimeStructure.Minutes,
             RTC_TimeStructure.Seconds);
    /* Unfreeze the RTC DAT Register */
    (void)RTC->DATE;
}
/**
 * @brief  RTC initalize default value.
 */
void RTC_DateAndTimeDefaultVale(void)
{ // Date
    RTC_DateDefault.WeekDay = 3;
    RTC_DateDefault.Date    = 20;
    RTC_DateDefault.Month   = 11;
    RTC_DateDefault.Year    = 19;
    // Time
    RTC_TimeDefault.H12     = RTC_AM_H12;
    RTC_TimeDefault.Hours   = 4;
    RTC_TimeDefault.Minutes = 5;
    RTC_TimeDefault.Seconds = 1;
}
/**
 * @brief  RTC date regulate with the default value.
 */
ErrorStatus RTC_DateRegulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;
    log_info("\n\r //=============Date Settings================// \n\r");

    log_info("\n\r Please Set WeekDay (01-07) \n\r");
    tmp_hh = RTC_DateDefault.WeekDay;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.WeekDay = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    tmp_hh = 0xFF;
    log_info("\n\r Please Set Date (01-31) \n\r");
    tmp_hh = RTC_DateDefault.Date;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Date = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    log_info("\n\r Please Set Month (01-12)\n\r");
    tmp_mm = RTC_DateDefault.Month;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Month = tmp_mm;
    }
    log_info(": %0.2d\n\r", tmp_mm);

    log_info("\n\r Please Set Year (00-99)\n\r");
    tmp_ss = RTC_DateDefault.Year;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Year = tmp_ss;
    }
    log_info(": %0.2d\n\r", tmp_ss);

    /* Configure the RTC date register */
    if (RTC_SetDate(RTC_FORMAT_BIN, &RTC_DateStructure) == ERROR)
    {
        log_info("\n\r>> !! RTC Set Date failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        log_info("\n\r>> !! RTC Set Date success. !! <<\n\r");
        RTC_DateShow();
        return SUCCESS;
    }
}
/**
 * @brief  RTC time regulate with the default value.
 */
ErrorStatus RTC_TimeRegulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;
    log_info("\n\r //==============Time Settings=================// \n\r");

    RTC_TimeStructure.H12 = RTC_TimeDefault.H12;

    log_info("\n\r Please Set Hours \n\r");
    tmp_hh = RTC_TimeDefault.Hours;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Hours = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    log_info("\n\r Please Set Minutes \n\r");
    tmp_mm = RTC_TimeDefault.Minutes;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Minutes = tmp_mm;
    }
    log_info(": %0.2d\n\r", tmp_mm);

    log_info("\n\r Please Set Seconds \n\r");
    tmp_ss = RTC_TimeDefault.Seconds;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Seconds = tmp_ss;
    }
    log_info(": %0.2d\n\r", tmp_ss);

    /* Configure the RTC time register */
    if (RTC_ConfigTime(RTC_FORMAT_BIN, &RTC_TimeStructure) == ERROR)
    {
        log_info("\n\r>> !! RTC Set Time failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        log_info("\n\r>> !! RTC Set Time success. !! <<\n\r");
        RTC_TimeShow();
        return SUCCESS;
    }
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
