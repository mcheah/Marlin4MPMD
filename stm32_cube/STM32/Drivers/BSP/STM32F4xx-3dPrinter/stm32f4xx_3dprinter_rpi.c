/**
  ******************************************************************************
  * @file    stm32f4xx_3dPrinter_rpi.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    April 13, 2016
  * @brief   Functions dedicated to Raspberry Pi mngt of 3D Printer BSP driver
  * @note    (C) COPYRIGHT 2016 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_3dprinter_rpi.h"
#include "stm32f4xx_3dprinter_misc.h"
#include "string.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/

/* Private constant ----------------------------------------------------------*/

/* Global variables ---------------------------------------------------------*/

/* Private function -----------------------------------------------------------*/


/******************************************************//**
 * @brief  Initialisation of the GPIOs for RPi mngt
 * @param None
 * @retval None
 **********************************************************/
void BSP_RPiGpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
    
  GPIO_InitStruct.Pin = BSP_RPI_READY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_RPI_READY_PORT, &GPIO_InitStruct);

  HAL_Delay(100);

}


 /******************************************************//**
 * @brief Wait until Raspberry Pi is ready
 * @param None
 * @retval None
 **********************************************************/
void BSP_RPiWaitUntilReady(void)
{
	uint8_t exit_loop = 0;

	while( !exit_loop) {

		if( GPIO_PIN_RESET == HAL_GPIO_ReadPin(BSP_RPI_READY_PORT, BSP_RPI_READY_PIN))
		{
			HAL_Delay(100);

			if( GPIO_PIN_RESET == HAL_GPIO_ReadPin(BSP_RPI_READY_PORT, BSP_RPI_READY_PIN))
				exit_loop = 1;
		}
	}

	// Wait 5 second more
	HAL_Delay(5000);
}        
 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

