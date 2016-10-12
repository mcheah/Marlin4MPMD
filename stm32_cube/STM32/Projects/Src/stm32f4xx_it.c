/**
  ******************************************************************************
  * @file    STM32F4xx-3dPrinter/Demonstrations/Src/stm32f4xx_it.c 
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "stm32f4xx_it.h"
#include "motorcontrol.h"
#include "stm32f4xx_3dprinter_uart.h"
#include "stm32f4xx_3dprinter_sd.h"
    
/** @addtogroup Interrupt_Handlers
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef hTimPwmX;
extern TIM_HandleTypeDef hTimPwmY;
extern TIM_HandleTypeDef hTimPwmZ;
extern TIM_HandleTypeDef hTimPwmE1;
extern TIM_HandleTypeDef hTimPwmE2;
extern TIM_HandleTypeDef hTimPwmE3;
extern TIM_HandleTypeDef hTimPwmE4;
extern TIM_HandleTypeDef hTimTick;
extern TIM_HandleTypeDef hTimTick2;
extern TIM_HandleTypeDef hTimServo;

extern BspAdcDataType gBspAdcData;
extern BspWifiDataType gBspWifiData;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/


/**
  * @brief  This function handles interrupt for External line 0
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{

}


/**
  * @brief  This function handles interrupt for External line 1
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BSP_MOTOR_CONTROL_BOARD_FLAG_PIN);
}

/**
  * @brief  This function handles interrupt for External lines 10 to 15
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	/* PA15 : SD_CARD_DETECT */
	HAL_GPIO_EXTI_IRQHandler(BSP_SD_DETECT_PIN);
}

/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimPwmX);
}
/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimPwmY);
}
/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimPwmZ);
}
/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimPwmE1);
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
  HAL_TIM_IRQHandler(&hTimTick);
  
}


/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimPwmE2);
}
/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_UP_TIM10_IRQHandler(void)  
{
#ifdef MARLIN
  HAL_TIM_IRQHandler(&hTimServo);
#else  
  HAL_TIM_IRQHandler(&hTimPwmE3);
#endif  
}
/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
#ifdef MARLIN
  HAL_NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_IRQHandler(&hTimTick2);  
#else  
  HAL_TIM_IRQHandler(&hTimPwmE4);
#endif  
  
}


/**
  * @brief  This function handles UART interrupt request for debug.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA 
  *         used for USART data transmission     
  */
void BSP_UART_DEBUG_IRQHandler(void)
{
  HAL_UART_IRQHandler(&gBspUartData.handle);
}

/**
  * @brief  This function handles UART interrupt request for wifi module.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA 
  *         used for USART data transmission     
  */
void BSP_WIFI_UART_IRQHandler(void)
{
  HAL_UART_IRQHandler(&(gBspWifiData.uartHandle));
}

/**
* @brief This function handles DMA global interrupt.
*/
void BSP_DMA_IRQHandler(void)
{
  //HAL_NVIC_ClearPendingIRQ(BSP_DMA_IRQn);
  HAL_DMA_IRQHandler(&(gBspAdcData.dmaHandle));
}

/**
* @brief This function handles ADC global interrupts.
*/
void BSP_ADC_IRQHandler(void)
{
  //HAL_NVIC_ClearPendingIRQ(BSP_ADC_IRQn);
  HAL_ADC_IRQHandler(&(gBspAdcData.adcHandle));
}

/**
  * @brief  This function handles DMA2 Stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
  BSP_SD_DMA_Rx_IRQHandler();
}

/**
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream6_IRQHandler(void)
{
  BSP_SD_DMA_Tx_IRQHandler(); 
}

/**
  * @brief  This function handles DMA RX interrupt request.  
  * @param  None
  * @retval None    
  */
void BSP_WIFI_UART_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(gBspWifiData.uartHandle.hdmarx);
}

/**
  * @brief  This function handles DMA TX interrupt request.
  * @param  None
  * @retval None  
  */
void BSP_WIFI_UART_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(gBspWifiData.uartHandle.hdmatx);
}

/**
  * @brief  This function handles SDIO interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler(void)
{
  BSP_SD_IRQHandler();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
