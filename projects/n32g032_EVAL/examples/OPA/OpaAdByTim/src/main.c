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

/** @addtogroup OPAMP_PGA
 * @{
 */

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;
uint8_t gRunFlag[4] = {0};

void RCC_Configuration(void);
void GPIO_Configuration(void);
void OPA_Configuration(void);
void COMP_Configuratoin(void);
void ADC_SampleConfig(void);
void TIM_PwmConfig(TIM_Module* TIMx);
void TIM_AllPwmOpen(TIM_Module* TIMx);
void TIM_AllPwmShut(TIM_Module* TIMx);
void TIM_DutySet(TIM_Module* TIMx, int16_t duty1, int16_t duty2, int16_t duty3);
void TIM_Run(TIM_Module* TIMx, uint8_t* RunFlag);

/**
 * @brief   Main program,test whether the PGA is working, Opa out Pin can view by oscilloscope
 */
uint8_t Gain = 0, BakGain = 0;
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* OPA configuration ------------------------------------------------------*/
    OPA_Configuration();

    /* ADC configuration ------------------------------------------------------*/
    ADC_SampleConfig();

    /* COMP configuration ------------------------------------------------------*/
    COMP_Configuratoin();

    /* TIMx configuration ------------------------------------------------------*/
    TIM_PwmConfig(TIM1);

    /*test*/
    TIM_AllPwmOpen(TIM1);
    while (1)
    {
        //TIM_Run(TIM1, &gRunFlag[0]);
        //TIM_Run(TIM8, &gRunFlag[1]);
        if (BakGain != Gain)
        {
            BakGain = Gain;
            switch (Gain)
            {
            case 2:
                OPAMP_SetPgaGain(OPAMP_CS_PGA_GAIN_2);
                break;
            case 4:
                OPAMP_SetPgaGain( OPAMP_CS_PGA_GAIN_4);
                break;
            case 8:
                OPAMP_SetPgaGain( OPAMP_CS_PGA_GAIN_8);
                break;
            case 16:
                OPAMP_SetPgaGain( OPAMP_CS_PGA_GAIN_16);
                break;
            case 32:
                OPAMP_SetPgaGain( OPAMP_CS_PGA_GAIN_32);
                break;
            default:
                break;
            }
        }
    }
}

/**
 * @brief  Start Demo.
 */
void TIM_Run(TIM_Module* TIMx, uint8_t* RunFlag)
{
    if (*RunFlag == 1)
    {
        TIM_AllPwmOpen(TIMx);
        *RunFlag = 0;
    }
    if (*RunFlag == 2)
    {
        TIM_AllPwmShut(TIMx);
        *RunFlag = 0;
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable GPIOA,GPIOB,GPIOC, GPIOD and GPIOF clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                          | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOF,ENABLE);


    /* Enable COMP OPA clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_OPAMP | RCC_APB1_PERIPH_COMPFILT, ENABLE);

    /* Enable ADC clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC ,ENABLE);
		/* Enable ADC1M clock. selsect HSI as RCC ADC1M CLK Source. 
		 The HSI should be enabled when you want to configure HSE as RCC ADC1M CLK source.*/
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);  
    /* RCC_ADCHCLK_DIV16*/
    RCC_ConfigAdcHclk(RCC_ADCHCLK_DIV16);

    /* Enable TIMx clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1 , ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
static uint8_t swit = 0;
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /*OPA VP     OPA_VP,  
                 PA0                   as analog inputs */
    OPAMP_SetVmSel( OPAMP_CS_VMSEL_FLOATING1);
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    if (swit == 0)
    { // My Demo Need Cfg
        OPAMP_SetVpSel( OPAMP_CS_VPSEL_PA0);
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    }
    else if (swit == 1)
    { //
        OPAMP_SetVpSel( OPAMP_CS_VPSEL_PA1);
        GPIO_InitStructure.Pin = GPIO_PIN_1;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    }
    else if (swit == 2)
    { // 
        OPAMP_SetVpSel( OPAMP_CS_VPSEL_PA4);
        GPIO_InitStructure.Pin = GPIO_PIN_4;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    }
    else if (swit == 3)
    { //
        OPAMP_SetVpSel( OPAMP_CS_VPSEL_PF0);
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitPeripheral(GPIOF, &GPIO_InitStructure);
    }
    
    /*OPA output pin enable pix pin when OPAx En.not to remap or select output pin*/
    /*OPA OUT       OP_out,   
                    PA6                as analog output */
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pin       = GPIO_PIN_6;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    //===========================================================================
    /*COMP Config CP1_INP,  cp1_INM   CP1_OUT
                   PF7       PF6       PA15   */
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.Pin        = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitPeripheral(GPIOF, &GPIO_InitStructure);
    //comp1 out PA15
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin       = GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF8_COMP1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    //===========================================================================
    //double check followin pin
    /*TIM1 for FOC   CH1,   CH1,   CH1,    CH1N,   CH2N,   CH3N
                     PA8    PA9    PA10    PB13    PB14    PB15      */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM1;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_TIM1;
    GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* GPIOB Configuration: BKIN pin ,only  used in IomBreakEn mode*/
    GPIO_InitStructure.Pin       = GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP; // GPIO_Mode_Input;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_DOWN;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM1;  //GPIO_NO_AF;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    //============================================================================
}

/**
 * @brief  Configures the Opa.
 */
void OPA_Configuration(void)
{
    OPAMP_InitType OPAMP_Initial;
    OPAMP_StructInit(&OPAMP_Initial);
    OPAMP_Initial.Mod            = OPAMP_CS_FOLLOW; // OPAMP_CS_FOLLOW;//OPAMP_CS_PGA_EN;
    OPAMP_Initial.Gain           = OPAMP_CS_PGA_GAIN_2;
    OPAMP_Initial.TimeAutoMuxEn  = DISABLE;
    /*configure opa1*/
    OPAMP_Init( &OPAMP_Initial);
    OPAMP_Enable( ENABLE);
}

/**
 * @brief  Configures the comp module,this use comp1 and comp7.
 */
void COMP_Configuratoin(void)
{
    COMP_InitType COMP_Initial;

    /*Set ref1 and ref 2 voltage div*/
    COMP_ConfigVREFx(VREF1, COMP_INVREF_VREF1SEL_32, ENABLE);
    COMP_ConfigVREFx(VREF2, COMP_INVREF_VREF2SEL_32, ENABLE);

    /*Initial comp1 and comp2*/
    COMP_StructInit(&COMP_Initial);
    COMP_Initial.InpSel     = COMP1_CTRL_INPSEL_PF7;
    COMP_Initial.InmSel     = COMP1_CTRL_INMSEL_PF6;
    COMP_Initial.SampWindow = COMPX_FILC_SAMPW_30;
    COMP_Initial.Thresh     = COMPX_FILC_THRESH_28;
    COMP_Init(COMP1, &COMP_Initial);

    /*trig initial as tim1&tim8 break*/
    COMP_SetOutTrig(COMP1, COMP2_CTRL_OUTTRG_TIM1_BKIN_TIM8_BKIN);

    /*enable comp1*/
    COMP_Enable(COMP1, COMP_CTRL_EN_ENABLE);
}

/**
 * @brief  Configures the Adcx.
 */
void ADC_SampleConfig(void)
{
    ADC_InitType ADC_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* ADC registers reenable */
    ADC_DeInit(ADC);

    /*ADC configuration*/
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_INJ_CONV_T1_CC4;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(ADC, &ADC_InitStructure);

    /*ADCx Injected conversions configuration,Config Sampling Time*/
    ADC_ConfigInjectedSequencerLength(ADC, 1);
    ADC_ConfigInjectedChannel(ADC, ADC_CH_6_PA6, 1, ADC_SAMP_TIME_56CYCLES5);//OPA1

    /*ADC TrigInJectConv Enable*/
    ADC_EnableExternalTrigInjectedConv(ADC, ENABLE);

    /*NVIC Initial*/
    /*Enable the ADC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);
    /* Check ADC Ready */
    while(ADC_GetFlagStatusNew(ADC,ADC_FLAG_RDY) == RESET)
        ;

    /*Enable Temp and Vrefint*/
    ADC_EnableTempSensor(ENABLE);
    ADC_EnableVrefint(ENABLE);
	
    /*ADC1 Injected group of conversions end and Analog Watchdog interruptsenabling */
    ADC_ConfigInt(ADC, ADC_INT_JENDC | ADC_INT_AWD, ENABLE);
}

/**
 * @brief  Configures the Tim1 or Tim8.
 */
void TIM_PwmConfig(TIM_Module* TIMx)
{
    TIM_TimeBaseInitType TIMx_TimeBaseStructure;
    OCInitType TIMx_OCInitStructure;
    TIM_BDTRInitType TIMx_BDTRInitStructure;
    NVIC_InitType NVIC_InitStructure;

    uint16_t TimerPeriod = 0;

    TimerPeriod = (SystemCoreClock / 20000) - 1;

    /*Time Base configuration*/
    TIM_DeInit(TIMx);
    TIM_InitTimBaseStruct(&TIMx_TimeBaseStructure);
    TIMx_TimeBaseStructure.Prescaler = 0;
    TIMx_TimeBaseStructure.CntMode   = TIM_CNT_MODE_CENTER_ALIGN2; // 01:/\,irq flag only counter down
    TIMx_TimeBaseStructure.Period    = TimerPeriod;                // PWM_PERIOD;
    TIMx_TimeBaseStructure.ClkDiv    = 0;                          // TIM_CLK_DIV2;
    TIMx_TimeBaseStructure.RepetCnt  = 0; // REP_RATE;// Initial condition is REP=0 to set the UPDATE only on the underflow

    TIM_InitTimeBase(TIMx, &TIMx_TimeBaseStructure);
    /*Channel 1, 2,3 in PWM mode */
    TIM_InitOcStruct(&TIMx_OCInitStructure);
    TIMx_OCInitStructure.OcMode       = TIM_OCMODE_PWM1; // when '<' is active,when '>' is inactive
    TIMx_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
    TIMx_OCInitStructure.OutputNState = TIM_OUTPUT_STATE_ENABLE;
    TIMx_OCInitStructure.Pulse        = (TimerPeriod >> 1); // dummy value
    TIMx_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_HIGH;
    TIMx_OCInitStructure.OcNPolarity  = TIM_OCN_POLARITY_HIGH;
    TIMx_OCInitStructure.OcIdleState  = TIM_OC_IDLE_STATE_RESET;
    TIMx_OCInitStructure.OcNIdleState = TIM_OC_IDLE_STATE_RESET;
    TIM_InitOc1(TIMx, &TIMx_OCInitStructure);
    TIM_InitOc2(TIMx, &TIMx_OCInitStructure);
    TIM_InitOc3(TIMx, &TIMx_OCInitStructure);
    /*Channel 4 Configuration in OC */
    TIMx_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIMx_OCInitStructure.Pulse       = (TimerPeriod -200); // dummy value
    TIM_InitOc4(TIMx, &TIMx_OCInitStructure);

    /*Enables the TIM1 Preload on CC1,CC2,CC3,CC4 Register */
    TIM_ConfigOc1Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc2Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc3Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);

    /*Automatic Output enable, Break, dead time and lock configuration*/
    TIMx_BDTRInitStructure.OssrState = TIM_OSSR_STATE_ENABLE;
    TIMx_BDTRInitStructure.OssiState = TIM_OSSI_STATE_ENABLE;
    TIMx_BDTRInitStructure.LockLevel = TIM_LOCK_LEVEL_OFF;
    TIMx_BDTRInitStructure.DeadTime  = 72;                           // DEADTIME;
    TIMx_BDTRInitStructure.Break = TIM_BREAK_IN_ENABLE;              // TIM_BREAK_IN_ENABLE;TIM_BREAK_IN_DISABLE
    TIMx_BDTRInitStructure.BreakPolarity = TIM_BREAK_POLARITY_HIGH;  // TIM_BREAK_POLARITY_HIGH;//TIM_BREAK_POLARITY_LOW;
    TIMx_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_ENABLE; // TIM_AUTO_OUTPUT_DISABLE;//TIM_AUTO_OUTPUT_DISABLE;
    TIMx_BDTRInitStructure.IomBreakEn = false;
    TIMx_BDTRInitStructure.LockUpBreakEn = false;
    TIMx_BDTRInitStructure.PvdBreakEn = false;
    TIM_ConfigBkdt(TIMx, &TIMx_BDTRInitStructure);
    /*Sel Output Trigger*/
    TIM_SelectOutputTrig(TIMx, TIM_TRGO_SRC_OC4REF); // Master mode select,010:The Update event as trigger output(TRGO)
    /*IT about*/
    TIM_ClrIntPendingBit(TIMx, TIM_INT_BREAK);

    /*Enable the TIMx BRK Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = (TIMx == TIM1) ? TIM1_BRK_UP_TRG_COM_IRQn : TIM8_BRK_UP_TRG_COM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIMx counter enable*/
    TIM_Enable(TIMx, ENABLE);
    TIM_EnableCtrlPwmOutputs(TIMx, ENABLE);
}

/**
 * @brief  Configures the Tim1 or Tim8 cc output enable.
 */
void TIM_AllPwmOpen(TIM_Module* TIMx)
{
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3EN));
}

/**
 * @brief  Configures the Tim1 or Tim8 cc output shut.
 */
void TIM_AllPwmShut(TIM_Module* TIMx)
{
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3NEN));
}

/**
 * @brief  Configures the Tim1 or Tim8 cc brake
 */
void TIM_Brake(TIM_Module* TIMx)
{
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN));
}

/**
 * @brief  Configures the Tim1 or Tim8 set cc
 */
void TIM_DutySet(TIM_Module* TIMx, int16_t duty1, int16_t duty2, int16_t duty3)
{
    TIMx->CCDAT1 = duty1;
    TIMx->CCDAT2 = duty2;
    TIMx->CCDAT3 = duty3;
}

/*Irq Adcx Samp value*/
uint16_t ADC_ConvertedValue[4][5];
#define MASK_AD_BITS 0x0FFF // min 7bits

/**
 * @brief  Irq with adc1 and adc2
 */
void ADC_IRQHandler(void)
{
    if (ADC_GetIntStatus(ADC, ADC_INT_JENDC) == SET)
    {
        ADC_ClearFlag(ADC, ADC_FLAG_JENDC);
        ADC_ConvertedValue[0][0] = ADC_GetInjectedConversionDat(ADC, ADC_INJ_CH_1) & MASK_AD_BITS;

        GPIO_WriteBit(GPIOA,GPIO_PIN_12,(Bit_OperateType)!GPIO_ReadInputDataBit(GPIOA,GPIO_PIN_12));
    }
    else
    {
        if (ADC_GetIntStatus(ADC, ADC_INT_AWD) == SET)
            ADC_ClearFlag(ADC, ADC_FLAG_AWDG);
    }
}

/**
 * @brief  Irq TIM1 break
 */
void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
    if (TIM_GetIntStatus(TIM1, TIM_INT_BREAK) != RESET)
    {
        TIM_ClrIntPendingBit(TIM1, TIM_INT_BREAK);
    }

    TIM_AllPwmShut(TIM1);
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
