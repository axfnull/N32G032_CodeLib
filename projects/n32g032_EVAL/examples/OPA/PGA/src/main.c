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

void RCC_Configuration(void);
void GPIO_Configuration(void);
void OPAMP_Configuration(void);

/**
 * @brief   Main program,Test PGA is work ok? Opa out Pin can view by scope
 */
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* OPAMP configuration ------------------------------------------------------*/
    OPAMP_Configuration();

    while (1)
    {
        ;
    }
}

/**
 * @brief  Configures the Opa.
 */
void OPAMP_Configuration(void)
{
    OPAMP_InitType OPAMP_InitStructure;
    OPAMP_StructInit(&OPAMP_InitStructure);
	
    OPAMP_InitStructure.Gain = OPAMP_CS_PGA_GAIN_2;
    /*configure opamp1*/
    OPAMP_Init(&OPAMP_InitStructure);
	
    OPAMP_SetVpSel(OPAMP_CS_VPSEL_PA1);
    OPAMP_SetVmSel(OPAMP_CS_VMSEL_FLOATING2);
    OPAMP_Enable(ENABLE);
}


/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable COMPE clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_OPAMP | RCC_APB1_PERIPH_COMPFILT, ENABLE);
    /* Enable GPIOA, GPIOB, GPIOC, GPIOD and GPIOF clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                          | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOF, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure OPAMP_VPSEL,PA1 as analog inputs */
    GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_HIGH;
    GPIO_InitStructure.Pin          = GPIO_PIN_1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    // OPAMP output pin enable pix pin when OPAMPx En.not to remap or select output pin
    /* configure OP1_out, PA6 as analog output */
    GPIO_InitStructure.Pin = GPIO_PIN_6;
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
