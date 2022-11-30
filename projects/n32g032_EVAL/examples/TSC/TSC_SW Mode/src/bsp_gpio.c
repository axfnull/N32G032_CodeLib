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
 * @file bsp_gpio.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "bsp_gpio.h"

/***
 * @brief    Configure the ports of TSC channels for sample.
 */
void tsc_gpio_init(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Pull        = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed       = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current     = GPIO_DC_HIGH;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_NO_AF;

    RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOA
                            |RCC_APB2_PERIPH_GPIOB
                            |RCC_APB2_PERIPH_GPIOC
                            |RCC_APB2_PERIPH_GPIOF
                            |RCC_APB2_PERIPH_AFIO,
                            ENABLE);
    /*  tsc GPIOA port*/
    /*  PA0:TSC0    PA1:TSC1    PA2:TSC2    PA3:TSC3
        PA4:TSC4    PA5:TSC5    PA6:TSC6    PA7:TSC7
        PA11:TSC18  PA12:TSC19  */
    GPIO_InitStructure.Pin  = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  
                            | GPIO_PIN_3  | GPIO_PIN_4  | GPIO_PIN_5
                            | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_11
                            | GPIO_PIN_12;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    
    /*  tsc GPIOB port*/
    /*  PB0:TSC8    PB1:TSC9    PB2:TSC10   PB11:TSC11
        PB12:TSC12  PB14:TSC13  PB6:TSC20   PB8:TSC21   */
    GPIO_InitStructure.Pin  = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2
                            | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14
                            | GPIO_PIN_6  | GPIO_PIN_8;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* tsc GPIOC port */
    /*  PC6:TSC14   PC7:TSC15   PC8:TSC16   PC9:TSC17 */
    GPIO_InitStructure.Pin  = GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8
                            | GPIO_PIN_9;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    /* tsc GPIOF port */
    /*  PF0:TSC22   PF1:TSC23 */
    GPIO_InitStructure.Pin  = GPIO_PIN_0  | GPIO_PIN_1;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    
    /*PA9: TSC OUT*/
    GPIO_InitStructure.Pin              = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
    AFIO_EnableTscOut(ENABLE);
}

/***
 * @brief    Configure the ports of led.
 */
void led_gpio_init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);

    GPIO_ResetBits(GPIOB,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
    
    /*PB10*/
    GPIO_InitStructure.Pin              = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull        = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed       = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current     = GPIO_DC_HIGH;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_NO_AF;
    GPIO_InitPeripheral(GPIOC,&GPIO_InitStructure); 
}


/***
 * @brief    disable the port of LED.
  * @param:  None
  * @retval: None
  */
void led_gpio_DeInit(void)
{
    GPIO_DeInitPin(GPIOC,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
}


