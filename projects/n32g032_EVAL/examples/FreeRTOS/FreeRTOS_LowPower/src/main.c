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
#include "cmsis_os.h"

/**
 *  FreeRTOS Mail
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_1MS           ((uint32_t)1000)
/* Private macro -------------------------------------------------------------*/
#define blckqSTACK_SIZE   configMINIMAL_STACK_SIZE
#define QUEUE_SIZE        (uint32_t) 1
/* Private variables ---------------------------------------------------------*/
uint32_t Tick_num = 0;
osMessageQId osQueue;

/* The number of items the queue can hold.  This is 1 as the Rx task will
remove items as they are added so the Tx task should always find the queue
empty. */
#define QUEUE_LENGTH             (1)

/* The rate at which the Tx task sends to the queue. */
#define TX_DELAY                 (500)

/* The value that is sent from the Tx task to the Rx task on the queue. */
#define QUEUED_VALUE             (100)

/* The length of time the LED will remain on for.  It is on just long enough
to be able to see with the human eye so as not to distort the power readings too
much. */
#define LED_TOGGLE_DELAY         (20)
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 */
int main(void)
{
    /*Configure the SysTick IRQ priority */
    N32_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0);
    /* Get SystemCoreClock */
    SystemCoreClockUpdate();

    /* Config 1s SysTick 1ms  */
    SysTick_Config(SystemCoreClock/SYSTICK_1MS);

    /* Configure GPIO's to AN to reduce power consumption */
    GPIO_ConfigAN();
  
    /* Initialize Led1 as output pushpull mode */
    LedInit(PORT_GROUP, LED1_PIN);

    /* Turn on Led1 */
    LedOff(PORT_GROUP, LED1_PIN);
  
    /* Create the queue used by the two threads */
    osMessageQDef(osqueue, QUEUE_LENGTH, uint16_t);
    osQueue = osMessageCreate(osMessageQ(osqueue), NULL);

    /* Note the Tx has a lower priority than the Rx when the threads are
    spawned. */
    osThreadDef(RxThread, QueueReceiveThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    osThreadCreate(osThread(RxThread), NULL);

    osThreadDef(TxThread, QueueSendThread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE);
    osThreadCreate(osThread(TxThread), NULL);
  
    /* Start scheduler */
    osKernelStart ();
  
    /* We should never get here as control is now taken by the scheduler */
    for(;;);
}

/**
  * @brief  Message Queue Producer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void QueueSendThread(const void *argument)
{
    for (;;)
    {
        /* Place this thread into the blocked state until it is time to run again.
           The kernel will place the MCU into the Retention low power sleep state
           when the idle thread next runs. */
        osDelay(TX_DELAY);

        /* Send to the queue - causing the queue receive thread to flash its LED.
           It should not be necessary to block on the queue send because the Rx
           thread will already have removed the last queued item. */
        osMessagePut(osQueue, (uint32_t)QUEUED_VALUE, 0);
    }
}

/**
  * @brief  Message Queue Consumer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void QueueReceiveThread(const void *argument)
{
    osEvent event;

    for (;;)
    {
        /* Wait until something arrives in the queue. */
        event = osMessageGet(osQueue, osWaitForever);

        /* To get here something must have arrived, but is it the expected
        value?  If it is, turn the LED on for a short while. */
        if (event.status == osEventMessage)
        {
            if (event.value.v == QUEUED_VALUE)
            {
                LedOn(LED1_PORT,LED1_PIN);
                osDelay(LED_TOGGLE_DELAY);
                LedOff(LED1_PORT,LED1_PIN);
            }
        }
    }
}

/**
  * @brief  Pre Sleep Processing
  * @param  ulExpectedIdleTime: Expected time in idle state
  * @retval None
  */
void PreSleepProcessing(uint32_t * ulExpectedIdleTime)
{
    /* Called by the kernel before it places the MCU into a sleep mode because
    configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().

    NOTE:  Additional actions can be taken here to get the power consumption
    even lower.  For example, peripherals can be turned off here, and then back
    on again in the post sleep processing function.  For maximum power saving
    ensure all unused pins are in their lowest power state. */

    /* 
      (*ulExpectedIdleTime) is set to 0 to indicate that PreSleepProcessing contains
      its own wait for interrupt or wait for event instruction and so the kernel vPortSuppressTicksAndSleep 
      function does not need to execute the wfi instruction  
    */
    *ulExpectedIdleTime = 0;
    
    /*Enter to sleep Mode using the function PWR_EnterSLEEPMode with WFI instruction*/
    PWR_EnterSLEEPMode(DISABLE, PWR_SLEEPENTRY_WFI);
}

/**
  * @brief  Post Sleep Processing
  * @param  ulExpectedIdleTime: Not used
  * @retval None
  */
void PostSleepProcessing(uint32_t * ulExpectedIdleTime)
{
    /* Called by the kernel when the MCU exits a sleep mode because
    configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */

    /* Avoid compiler warnings about the unused parameter. */
    (void) ulExpectedIdleTime;
}

/**
  * @brief  Configure all GPIO's to AN to reduce the power consumption
  * @param  None
  * @retval None
  */
static void GPIO_ConfigAN(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Configure all GPIO as analog to reduce current consumption on non used IOs */
    /* Enable GPIOs clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                          | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOF, ENABLE);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin       = GPIO_PIN_ALL;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
    GPIO_InitPeripheral(GPIOF, &GPIO_InitStructure);

    /* Disable GPIOs clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC
                          | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_GPIOF, DISABLE);
}

/**
  * @brief  Sets the priority of an interrupt.
  * @param  IRQn External interrupt number .
  *         This parameter can be an enumerator of  IRQn_Type enumeration
  * @param  PreemptPriority The pre-emption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 3.
  *         A lower priority value indicates a higher priority 
  * @param  SubPriority the subpriority level for the IRQ channel.
  *         this parameter is a dummy value and it is ignored, because 
  *         no subpriority supported in Cortex M0 based products.   
  * @retval None
  */
void N32_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
    /* Check the parameters */
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));
  NVIC_SetPriority(IRQn,PreemptPriority);
}

/**
 * @brief  Configures LED GPIO.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
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
    else
    {
        return;
    }

    /* Configure the GPIO pin */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief  Turns selected Led on.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedOn(GPIO_Module *GPIOx, uint16_t Pin)
{
    GPIO_SetBits(GPIOx, Pin);
}

/**
 * @brief  Turns selected Led Off.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_ResetBits(GPIOx, Pin);
}

/**
 * @brief  Toggles the selected Led.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_TogglePin(GPIOx, Pin);
}

/**
 * @}
 */

/**
 * @}
 */
