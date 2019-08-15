/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_sctimer.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_BOARD_TEST_LED_PORT 1U
#define APP_BOARD_TEST_LED_PIN 10U
#define APP_SW_PORT BOARD_SW1_GPIO_PORT
#define APP_SW_PIN  BOARD_SW1_GPIO_PIN

#define SP_SW_PORT 0U
#define SP_SW_PIN  20U


#define SP1_SW_PORT 1U
#define SP1_SW_PIN  0U  

#define DIR_SW_PORT 1U

#define DIR_SW_PIN  12U

#define DIR1_SW_PORT 1U
#define DIR1_SW_PIN  13U

#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define DEMO_FIRST_SCTIMER_OUT kSCTIMER_Out_4
#define DEMO_SECOND_SCTIMER_OUT kSCTIMER_Out_5
#define DEMO_THIRD_SCTIMER_OUT kSCTIMER_Out_7
#define DEMO_FOURTH_SCTIMER_OUT kSCTIMER_Out_2

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
	 * Variablesp************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(void);
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 100000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}



    volatile uint8_t updatedDutycycle_m1_p = 50U;
    volatile uint8_t updatedDutycycle_m2_p = 50U;
    volatile uint8_t updatedDutycycle_m1_n = 1U;
    volatile uint8_t updatedDutycycle_m2_n = 1U;


    sctimer_config_t sctimerInfo;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t event1;
    uint32_t event2;
    uint32_t event3;
    uint32_t event4;
    uint32_t event;
    uint32_t sctimerClock;

    uint32_t port_state_SW = 0;
    uint32_t port_state_SW1 = 0;
    uint32_t port_state_SW2 = 0;
    uint32_t port_state_SW3 = 0;
/*!
 * @brief Main function
 */
 int forward()
 {

 }

int main(void)
{


    gpio_pin_config_t led_config = {
         kGPIO_DigitalOutput, 0,
     };

    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    
    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);


    sctimerClock = SCTIMER_CLK_FREQ;


    /* Init SW GPIO PORT. */
       GPIO_PortInit(GPIO, SP_SW_PORT);
       GPIO_PortInit(GPIO, SP1_SW_PORT);
       GPIO_PortInit(GPIO, DIR_SW_PORT);
       GPIO_PortInit(GPIO, DIR1_SW_PORT);

       /* Init output LED GPIO. */
       GPIO_PortInit(GPIO, APP_BOARD_TEST_LED_PORT);
       GPIO_PinInit(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, &led_config);
       GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, 1);

       /* Port masking */
       GPIO_PortMaskedSet(GPIO, APP_BOARD_TEST_LED_PORT, 0x0000FFFF);
       GPIO_PortMaskedWrite(GPIO, APP_BOARD_TEST_LED_PORT, 0xFFFFFFFF);
    /* Print a note to terminal */
    PRINTF("\r\nSCTimer example to output 2 center-aligned PWM signals\r\n");
    PRINTF("\r\nProbe the signal using an oscilloscope");

    SCTIMER_GetDefaultConfig(&sctimerInfo);

    	    /* Initialize SCTimer module */
    	    SCTIMER_Init(SCT0, &sctimerInfo);

    	    /* Configure first PWM with frequency 24kHZ from first output*/
    	    pwmParam.output = DEMO_FIRST_SCTIMER_OUT;
    	    pwmParam.level = kSCTIMER_HighTrue;
    	    pwmParam.dutyCyclePercent = 1;
    	    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 24000U, sctimerClock, &event) == kStatus_Fail)
    	    {
    	        return -1;
    	    }

    	    /* Configure second PWM with different duty cycle but same frequency as before */


    	    pwmParam.output = DEMO_SECOND_SCTIMER_OUT;
    	    pwmParam.level = kSCTIMER_LowTrue;
    	    pwmParam.dutyCyclePercent = 1;
    	    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 24000U, sctimerClock, &event) == kStatus_Fail)
    	      {
    	         return -1;
    	      }

    	    pwmParam.output = DEMO_THIRD_SCTIMER_OUT;
    	    pwmParam.level = kSCTIMER_HighTrue;
    	    pwmParam.dutyCyclePercent = 1;
    	    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 24000U, sctimerClock, &event) == kStatus_Fail)
    	      {
    	         return -1;
    	      }

    	    pwmParam.output = DEMO_FOURTH_SCTIMER_OUT;
    	    pwmParam.level = kSCTIMER_LowTrue;
    	    pwmParam.dutyCyclePercent = 1;
    	    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 24000U, sctimerClock, &event) == kStatus_Fail)
    	      {
    	         return -1;
    	      }

    	    /* Start the timer */
    	    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);

while (1)
    {
    	 port_state_SW = GPIO_PortRead(GPIO, SP_SW_PORT);
    	        if (!(port_state_SW & (1 << SP_SW_PIN)))
    	              {
    	        		// Forward
    	                  //PRINTF("\r\n Port state FOR EXTERNAL SWITCH: %x\r\n", port_state_SW);
    	                  PRINTF("\n Switch pressed FOR Forward Motion ");

    	                updatedDutycycle_m1_p= 99;
    	                  updatedDutycycle_m1_n= 1;
    	                  updatedDutycycle_m2_p= 1;
    	                  updatedDutycycle_m2_n= 99;


    	      	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_FIRST_SCTIMER_OUT, updatedDutycycle_m1_p, event1);
    	      	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_SECOND_SCTIMER_OUT, updatedDutycycle_m1_n, event2);
    	      	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_THIRD_SCTIMER_OUT, updatedDutycycle_m2_p, event3);
    	      	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_FOURTH_SCTIMER_OUT, updatedDutycycle_m2_n, event4);

    	                  //         PRINTF("\r\n dutyCycle = %x\r", updatedDutycycle );
    	           //       GPIO_PortToggle(GPIO, APP_BOARD_TEST_LED_PORT, 1u << APP_BOARD_TEST_LED_PIN);
    	              }
    	/*
    	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_FIRST_SCTIMER_OUT, updatedDutycycle_m1_p, event1);
    	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_SECOND_SCTIMER_OUT, updatedDutycycle_m1_n, event2);
    	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_THIRD_SCTIMER_OUT, updatedDutycycle_m2_p, event3);
    	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_FOURTH_SCTIMER_OUT, updatedDutycycle_m2_n, event4);

    	        port_state_SW1 = GPIO_PortRead(GPIO, SP1_SW_PORT);
    	        if (!(port_state_SW1 & (1 << SP1_SW_PIN)))
    	              {
    	        	//Backward
    	                  //PRINTF("\r\n Port state FOR EXTERNAL SWITCH: %x\r\n", port_state_SW1);
    	                  PRINTF("\n Switch pressed FOR SPEED OF MOTOR 1 decrease");
    	                  if(updatedDutycycle > 5)
    	                  {
    	                  updatedDutycycle = updatedDutycycle - 2U;
    	                  }
    	                  PRINTF("\r\n dutyCycle = %x\r",updatedDutycycle );
    	                  GPIO_PortToggle(GPIO, APP_BOARD_TEST_LED_PORT, 1u << APP_BOARD_TEST_LED_PIN);
    	              }



    	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_SECOND_SCTIMER_OUT, updatedDutycycle, event1);


    	        port_state_SW2 = GPIO_PortRead(GPIO, DIR_SW_PORT);
    	        if (!(port_state_SW2 & (1 << DIR_SW_PIN)))
    	              {
    	        			// turn Left
    	                  //PRINTF("\r\n Port state FOR EXTERNAL SWITCH: %x\r\n", port_state_SW2);
    	                  PRINTF("\n Switch pressed FOR speeed OF MOTOR 2 inc ");
    	                  if(updatedDutycycle > 98)
    	                  {
    	                  updatedDutycycle = updatedDutycycle + 2U;
    	                  }
    	                  PRINTF("\r\n dutyCycle = %x\r",updatedDutycycle );
    	                  GPIO_PortToggle(GPIO, APP_BOARD_TEST_LED_PORT, 1u << APP_BOARD_TEST_LED_PIN);
    	              }


    	        port_state_SW3 = GPIO_PortRead(GPIO, DIR1_SW_PORT);
    	        if (!(port_state_SW3 & (1 << DIR1_SW_PIN)))
    	              {

    	        	// Turn Right
    	        	   PRINTF("\n Switch pressed FOR SPEED OF MOTOR 2 decrease");
    	        	    	                  if(updatedDutycycle > 5)
    	        	    	                  {
    	        	    	                  updatedDutycycle = updatedDutycycle - 2U;
    	        	    	                  }
    	        	    	                  PRINTF("\r\n dutyCycle = %x\r",updatedDutycycle );
    	        	    	                  GPIO_PortToggle(GPIO, APP_BOARD_TEST_LED_PORT, 1u << APP_BOARD_TEST_LED_PIN);
    	              }

    	        SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_THIRD_SCTIMER_OUT, updatedDutycycle, event2);
*/
    	        delay();

    }

}
