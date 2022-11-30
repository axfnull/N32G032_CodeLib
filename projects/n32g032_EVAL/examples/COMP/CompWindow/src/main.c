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
#define BIT0                     ((uint32_t)1 << 0)
#define BIT1                     ((uint32_t)1 << 1)
#define BIT2                     ((uint32_t)1 << 2)
#define BIT3                     ((uint32_t)1 << 3)
#define BIT4                     ((uint32_t)1 << 4)
#define BIT5                     ((uint32_t)1 << 5)
#define BIT6                     ((uint32_t)1 << 6)
#define BIT7                     ((uint32_t)1 << 7)
#define BIT8                     ((uint32_t)1 << 8)
#define BIT9                     ((uint32_t)1 << 9)
#define BIT10                    ((uint32_t)1 << 10)
#define BIT11                    ((uint32_t)1 << 11)
#define BIT12                    ((uint32_t)1 << 12)
#define BIT13                    ((uint32_t)1 << 13)
#define BIT14                    ((uint32_t)1 << 14)
#define BIT15                    ((uint32_t)1 << 15)
#define BIT16                    ((uint32_t)1 << 16)
#define BIT17                    ((uint32_t)1 << 17)
#define BIT18                    ((uint32_t)1 << 18)
#define BIT19                    ((uint32_t)1 << 19)
#define BIT20                    ((uint32_t)1 << 20)
#define BIT21                    ((uint32_t)1 << 21)
#define BIT22                    ((uint32_t)1 << 22)
#define BIT23                    ((uint32_t)1 << 23)
#define BIT24                    ((uint32_t)1 << 24)
#define BIT25                    ((uint32_t)1 << 25)
#define BIT26                    ((uint32_t)1 << 26)
#define BIT27                    ((uint32_t)1 << 27)
#define BIT28                    ((uint32_t)1 << 28)
#define BIT29                    ((uint32_t)1 << 29)
#define BIT30                    ((uint32_t)1 << 30)
#define BIT31                    ((uint32_t)1 << 31)
#define SetBitMsk(reg, bit, msk) ((reg) = ((reg) & ~(msk) | (bit)))
#define ClrBit(reg, bit)         ((reg) &= ~(bit))
#define SetBit(reg, bit)         ((reg) |= (bit))
#define GetBit(reg, bit)         ((reg) & (bit))

#include "main.h"

/** @addtogroup ADC_3ADCs_DMA
 * @{
 */

TIM_TimeBaseInitType TIM_TimeBaseStructure;
OCInitType TIM_OCInitStructure;
TIM_BDTRInitType TIM_BDTRInitStructure;
uint16_t TimerPeriod   = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;

void RCC_Configuration(void);
void GPIO_CompConfiguration(void);
void GPIO_TimConfiguration(void);
void COMP_Configuratoin(void);
void TIM_Intial(TIM_Module* TIMx);
void NVIC_Configuration(void);


/**
 * @brief   Main program
 */
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_CompConfiguration();
  
    /* COMP configuration ------------------------------------------------------*/
    COMP_Configuratoin();
    while (1)
    {
        
    }
}


/**
 * @brief  Configures the comp module.
 */
void COMP_Configuratoin(void)
{
    COMP_InitType COMP_Initial;

    /*Initial comp*/
    COMP_StructInit(&COMP_Initial);
    COMP_Initial.InpSel     = COMPx_CTRL_INPSEL_VREF1;
    COMP_Initial.InmSel     = COMP1_CTRL_INMSEL_PA4;
    COMP_Initial.SampWindow = COMPX_FILC_SAMPW_30;
    COMP_Initial.Thresh     = COMPX_FILC_THRESH_16;
    COMP_Init(COMP1, &COMP_Initial);

    /*Initial comp*/
    COMP_StructInit(&COMP_Initial);
    COMP_Initial.InpSel     = COMPx_CTRL_INPSEL_VREF2;
    COMP_Initial.InmSel     = COMP2_CTRL_INMSEL_PA4;
    COMP_Initial.SampWindow = COMPX_FILC_SAMPW_30;
    COMP_Initial.Thresh     = COMPX_FILC_THRESH_16;
    COMP_Initial.windowout  = COMP_CTRL_WINOUT_WINDOW;
    COMP_Init(COMP2, &COMP_Initial);
    
    COMP_ConfigWindowMode(COMP_WINMODE_WINMDINSEL_PA4,COMP_WINMODE_CMP12MD_ENABLE); 
    
    COMP_ConfigVREFx(VREF1, COMP_INVREF_VREF1SEL_45, ENABLE);
    COMP_ConfigVREFx(VREF2, COMP_INVREF_VREF2SEL_25, ENABLE);
    
    /*enable comp1*/
    COMP_Enable(COMP1, COMP_CTRL_EN_ENABLE);
    /*enable comp2*/
    COMP_Enable(COMP2, COMP_CTRL_EN_ENABLE);
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable COMPE clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_COMPFILT, ENABLE);
    /* Enable GPIOA, GPIOB, GPIOC and GPIOD clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA,ENABLE);
}


/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_CompConfiguration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    GPIO_InitStruct(&GPIO_InitStructure);
    // INM 1 // INM 2 
    GPIO_InitStructure.Pin            = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_LOW;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    // OutSel
    // PA7
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin            = GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF8_COMP2;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}


/**
 * @brief  Configures Vector Table base location.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Configure and enable ADC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = COMP1_2_3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
