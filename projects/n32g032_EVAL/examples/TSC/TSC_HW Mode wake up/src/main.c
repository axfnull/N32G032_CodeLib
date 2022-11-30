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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_tsc.h"

/**
 * @brief  Set system clock again,use HSI as clock source,and enable pll
 * &param  None
 * @retval: None
 */
void SetSysClock_HSI_PLL(void)
{
    __IO uint32_t StartUpCounter = 0, ClkStartUpStatus = 0;

    // It is necessary to initialize the RCC peripheral to the reset state.
    RCC_DeInit();

    // Enable HSE, open external crystal oscillator.
    RCC_EnableHsi(ENABLE);

    // Wait for HSE to be stable.
    ClkStartUpStatus = RCC_WaitHsiStable();

    // Go on until the HSI is stable.
    if (ClkStartUpStatus == SUCCESS)
    {
        //----------------------------------------------------------------------//
        // Enable flash Prefetch buffer
        FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);

        FLASH_SetLatency(FLASH_LATENCY_2);
        //----------------------------------------------------------------------//

        // AHB prescaler factor set to 1,HCLK = SYSCLK = 16M
        RCC_ConfigHclk(RCC_SYSCLK_DIV1);
        // AHB prescaler factor set to 1,PCLK2 = HCLK/1 = 16M
        RCC_ConfigPclk2(RCC_HCLK_DIV1);
        // AHB prescaler factor set to 1,PCLK1 = HCLK/1 = 16M
        RCC_ConfigPclk1(RCC_HCLK_DIV1);

        ////-----------------Set PLL clock source as HSE, set PLL frequency multiplication factor.-------------------//
        // PLLCLK = 8MHz * pllmul
        RCC_ConfigPll(RCC_PLL_SRC_HSI,RCC_PLL_MUL_4,RCC_PLL_PRE_1,RCC_PLLOUT_DIV_2);
        ////------------------------------------------------------------------//

        // Enable PLL
        RCC_EnablePll(ENABLE);
        // Wait for PLL to be stable.
        while (RCC_GetFlagStatus(RCC_CTRL_FLAG_PLLRDF) == RESET)
        {
        }

        // Switch PLL clock to SYSCLK.
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

        // Read the clock switch status bit and make sure pllclk is selected as the system clock.
        while (RCC_GetSysclkSrc() != RCC_CFG_SCLKSTS_PLL)
        {
        }
    }
    else
    {
        // Clock fail
        while (1)
        {
        }
    }

    SystemCoreClockUpdate();
}

/**
 * @brief  simple delay
 * &param  count: count value of delay
 * @retval: None
 */
static void Delay(uint32_t count)
{
    u32 i;
    for(i = 0; i < count; i++)
    {
        i = i;
    }
}

/***
 * @brief    disable the port of LED.
  * @param:  None
  * @retval: None
  */
static void led3_blink(uint32_t cnt)
{
    uint32_t tcnt = cnt*2;

    while(tcnt>0)
    {
        tcnt--;

        if(tcnt & 0x01UL)
            LED3_ON;
        else
            LED3_OFF;

        Delay(SystemCoreClock>>6);
    }
}

/**
 * @brief  Disable debug uart and led before enter low power mode
 * &param   None
 * @retval: None
 */
void DeInitPeripheral(void)
{
    log_debug("Deinit uart and led gpio!\r\n");
    Debug_USART_DeInit();
    led_gpio_DeInit();
}

/**
 * @brief  Init debug uart and led port
 * &param   None
 * @retval: None
 */
void InitPeripheral(void)
{
    /*Initialize USART as 115200 bps, 8 bit length,1 stop bit,no check bit,enable receive by interrupt */
    Debug_USART_Config();
    log_debug("Debug uart port inited\r\n");
    
    led_gpio_init();
    log_debug("led gpio inited\r\n");
}

/**
 * @brief  Main program.
 */
int main(void)
{
    //set SYSCLK = 16M
    SetSysClock_HSI_PLL();
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR,ENABLE);
    
    InitPeripheral();   /*Init debug uart and led port*/
    log_info("Set system clock to 16MHz\r\n");
    
    BSP_TSC_HwInit();   /* Init TSC mode */
    log_debug("TSC inited\r\n");
    
    BSP_TSC_AutoAdjust();   /* Config TSC threshold */
    log_debug("TSC auto adjust done\r\n");
    
    BSP_TSC_ConfigInt();    /* Enable TSC interrupt */
    __TSC_CHN_CONFIG(TSC_CH_USED);      /*Enable channels of TSC*/
    
    log_info("TSC HW mode demo!\r\n");
    Delay(SystemCoreClock>>6);

  #ifdef TSC_LOWPOWER_DEBUG
    DBG_ConfigPeriph(DBG_STOP,ENABLE);
  #endif
  
    while (1)
    {
        log_info("TSC wake up loop start!\r\n");

        __TSC_HW_ENABLE();                  /* Enalbe hw detect*/
        log_info("Enter sleep mode!\r\n");
        DeInitPeripheral();                 /*Deinit debug uart and led port*/
        PWR_EnterSLEEPMode(0,PWR_STOPENTRY_WFI);   /* Enter sleep mode */

        InitPeripheral();   /* Wake up from sleep mode, init debug uart and led port */
        log_info("T%d press,exit sleep mode!\r\n",BSP_TSC_GetKeyNum());
        led3_blink(1);

        __TSC_HW_ENABLE();              /* Enalbe hw detect*/
        log_info("Enter stop mode!\r\n");
        DeInitPeripheral();                 /*Deinit debug uart and led port*/
        PWR_EnterSTOPMode(PWR_STOPPLUSE_DISABLE,PWR_STOPENTRY_WFI);   /* Enter stop mode */

        SetSysClock_HSI_PLL();     /* Config PLL */
        InitPeripheral();   /* Init debug uart and led port */
        log_info("T%d press,exit stop mode!\r\n",BSP_TSC_GetKeyNum());
        led3_blink(2);

        __TSC_HW_ENABLE();              /* Enalbe hw detect*/
        log_info("Enter stop plus mode!\r\n");
        DeInitPeripheral();                 /*Deinit debug uart and led port*/
        PWR_EnterSTOPMode(PWR_STOPPLUSE_DISABLE,PWR_STOPENTRY_WFI);/* Enter stop plus mode */

        SetSysClock_HSI_PLL();
        InitPeripheral();   /* Wake up from stop2 mode, init debug uart and led port */
        log_info("T%d press,exit stop plus mode!\r\n",BSP_TSC_GetKeyNum());
        led3_blink(3);
    }
}

/******************************************************************/

/******************************************************************/

/**
 * @brief Assert failed function by user.
 * @param file The name of the call that failed.
 * @param line The source line number of the call that failed.
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
#if LOG_ENABLE
    if(DebugUartInited)
        log_error("assert failed:%s at %s (line %d)",expr, file, line);
#endif
    
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT
/******************************************************************/


