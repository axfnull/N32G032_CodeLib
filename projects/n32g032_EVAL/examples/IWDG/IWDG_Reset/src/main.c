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
#include "log.h"

/** @addtogroup n32g032_StdPeriph_Examples
 * @{
 */

/** @addtogroup
 * @{
 */

//#define LSI_TIM_MEASURE
#define LED1 GPIO_PIN_4
#define LED2 GPIO_PIN_5

__IO uint32_t TimingDelay = 0;
__IO uint32_t LsiFreq     = 30000;
extern __IO uint16_t CaptureNumber;

void Delay(__IO uint32_t nTime);
void TIM4_ConfigForLSI(void);

/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
    else if (GPIOx == GPIOF)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, ENABLE);
    }

    /* Configure the GPIO pin */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    // GPIOx->PBSC = Pin;
}

/**
 * @brief  Turns selected Led on.
 * @param Led Specifies the Led to be set on.
 */
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBSC = Pin;
}
/**
 * @brief  Turns selected Led Off.
 * @param Led Specifies the Led to be set off.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBC = Pin;
}

/**
 * @brief  Toggles the selected Led.
 * @param Led Specifies the Led to be toggled.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

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
    log_init();
    log_info("--- IWDG demo reset ---\n");
    LedInit(GPIOA, LED1 | LED2);
    LedOff(GPIOA, LED1 | LED2);
    /* Enable PWR Clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    DBG_ConfigPeriph(DBG_IWDG_STOP, ENABLE);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1)
            ;
    }

    /* Check if the system has resumed from IWDG reset */
    if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_IWDGRSTF) != RESET)
    {
        /* IWDGRST flag set */
        /* Turn on LED1 */
        LedOn(GPIOA, LED1);
        log_info("reset by IWDG\n");
        /* Clear reset flags */
        RCC_ClrFlag();
    }
    else
    {
        /* IWDGRST flag is not set */
        /* Turn off LED1 */
        LedOff(GPIOA, LED1);
    }

#ifdef LSI_TIM_MEASURE
    /* Enable the LSI OSC */
    RCC_EnableLsi(ENABLE);

    /* Wait till LSI is ready */
    while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
    {
    }

    /* TIM Configuration -------------------------------------------------------*/
    TIM4_ConfigForLSI();

    /* Wait until the TIM4 get 3 LSI edges */
    while (CaptureNumber != 3)
    {
    }

    /* Disable TIM4 CC2 Interrupt Request */
    TIM_ConfigInt(TIM4, TIM_INT_CC2, DISABLE);
#endif

    /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
       dispersion) */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteConfig(IWDG_WRITE_ENABLE);

    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV32);

    /* Set counter reload value to obtain 250ms IWDG TimeOut.
       Counter Reload Value = 250ms/IWDG counter clock period
                            = 250ms /( 1/(LSI/32))
                            = 234.375ms = (LsiFreq/128)
     */
    log_debug("LsiFreq is: %d\n", LsiFreq);
    IWDG_CntReload(LsiFreq/128);
    /* Reload IWDG counter */
    IWDG_ReloadKey();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();

    while (1)
    {
        /* Toggle LED2 */
        LedBlink(GPIOA, LED2);
        /* Insert 249 ms delay */
        Delay(200);

        /* Reload IWDG counter */
        IWDG_ReloadKey();
    }
}

/**
 * @brief  Inserts a delay time.
 * @param nTime specifies the delay time length, in milliseconds.
 */
void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while (TimingDelay != 0)
        ;
}

#ifdef LSI_TIM_MEASURE
/**
 * @brief  Configures TIM4 to measure the LSI oscillator frequency.
 */
void TIM4_ConfigForLSI(void)
{
    NVIC_InitType NVIC_InitStructure;
    TIM_ICInitType TIM_ICInitStructure;

    /* Enable TIM4 clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM4, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Enable the TIM4 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure TIM4 prescaler */
    TIM_ConfigPrescaler(TIM4, 0, TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* Connect internally the TM4_CH2 Input Capture to the LSI clock output */
    GPIO_ConfigPinRemap(AFIO_RMP_TIM4CH2, ENABLE);

    /* TIM4 configuration: Input Capture mode ---------------------
       The LSI oscillator is connected to TIM4 CH2
       The Rising edge is used as active edge,
       The TIM4 CCDAT2 is used to compute the frequency value
    ------------------------------------------------------------ */
    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV8;
    TIM_ICInitStructure.IcFilter    = 0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    /* TIM10 Counter Enable */
    TIM_Enable(TIM4, ENABLE);

    /* Reset the flags */
    TIM4->STS = 0;

    /* Enable the CC4 Interrupt Request */
    TIM_ConfigInt(TIM4, TIM_INT_CC2, ENABLE);
}
#endif

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
