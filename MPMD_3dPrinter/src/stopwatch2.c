/*
 * @brief LPC17xx_40xx specific stopwatch implementation
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

//#include "chip.h"
#include <stopwatch2.h>
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Precompute these to optimize runtime */
static uint32_t ticksPerSecond;
static uint32_t ticksPerMs;
static uint32_t ticksPerUs;
static TIM_HandleTypeDef TimHandle;

extern void Error_Handler(uint16_t errorcode);
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize stopwatch */
void StopWatch_Init(void)
{
	  /*##-1- Configure the TIM peripheral #######################################*/
	  /* -----------------------------------------------------------------------
	    In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1),
	    since APB1 prescaler is different from 1.
	      TIM2CLK = 2 * PCLK1
	      PCLK1 = HCLK / 4
	      => TIM2CLK = HCLK / 2 = SystemCoreClock /2
	    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
	    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
	    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1

	    Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	     variable value. Otherwise, any configuration based on this variable will be incorrect.
	     This variable is updated in three ways:
	      1) by calling CMSIS function SystemCoreClockUpdate()
	      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	  ----------------------------------------------------------------------- */

	  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	  //uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
	  uint32_t uwPrescalerValue = 10000; //divide by 1
	  /* Set TIMx instance */
	  TimHandle.Instance = TIMx;

	  /* Initialize TIM3 peripheral as follow:
	       + Period = 10000 - 1
	       + Prescaler = ((SystemCoreClock/2)/10000) - 1
	       + ClockDivision = 0
	       + Counter direction = Up
	  */

	  //Initialize at 	160Mhz ~=6.25nSec
	  //				Period=2^32 ~=26.9sec

	  TimHandle.Init.Period = 0xFFFFFFFF;//10000 - 1;
	  TimHandle.Init.Prescaler = uwPrescalerValue;
	  TimHandle.Init.ClockDivision = 0;
	  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	  {
	    /* Initialization Error */
//	    Error_Handler(10); //Hardcoded to 10 because I'm too lazy to include enumerations properly
	  }

	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
	  /* Start Channel1 */
	  if(HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
	  {
		//Starting Error
//	    Error_Handler(10);
	  }



//	    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
//	    since APB1 prescaler is different from 1.
//	      TIM3CLK = 2 * PCLK1
//	      PCLK1 = HCLK / 4
//	      => TIM3CLK = HCLK / 2 = SystemCoreClock /2

	/* Use timer 1. Set prescaler to divide by 8, should give ticks at 3.75 MHz.
	   That gives a useable stopwatch measurement range of about 19 minutes
	   (if system clock is running at 120 MHz). */
	//const uint32_t prescaleDivisor = 1;
	//Chip_TIMER_Init(STOPWATCH_TIMER);
	//Chip_TIMER_PrescaleSet(STOPWATCH_TIMER, prescaleDivisor - 1);
	//Chip_TIMER_Enable(STOPWATCH_TIMER);

	/* Pre-compute tick rate. Note that peripheral clock supplied to the
	   timer includes a fixed divide by 4. */
	ticksPerSecond = HAL_RCC_GetPCLK1Freq() *2 / (uwPrescalerValue+1) / 1;
	ticksPerMs = ticksPerSecond / 1000;
	ticksPerUs = ticksPerSecond / 1000000;
}

/* Start a stopwatch */
uint32_t StopWatch_Start(void)
{
	/* Return the current timer count. */
	return __HAL_TIM_GET_COUNTER(&TimHandle);
}

/* Returns number of ticks per second of the stopwatch timer */
uint32_t StopWatch_TicksPerSecond(void)
{
	return ticksPerSecond;
}

/* Converts from stopwatch ticks to mS. */
uint32_t StopWatch_TicksToMs(uint32_t ticks)
{
	return ticks / ticksPerMs;
}

/* Converts from stopwatch ticks to uS. */
uint32_t StopWatch_TicksToUs(uint32_t ticks)
{
	return ticks / ticksPerUs;
}

/* Converts from mS to stopwatch ticks. */
uint32_t StopWatch_MsToTicks(uint32_t mS)
{
	return mS * ticksPerMs;
}

/* Converts from uS to stopwatch ticks. */
uint32_t StopWatch_UsToTicks(uint32_t uS)
{
	return uS * ticksPerUs;
}

