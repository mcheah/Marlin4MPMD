/**
  ******************************************************************************
  * @file    stm32f0xx_it.h 
  * @author  MCD Application Team
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0xx_IT_H
#define __STM32F0xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

//void NMI_Handler(void);
//void HardFault_Handler(void);
//void SVC_Handler(void);
//void PendSV_Handler(void);
void SysTick_Handler(void);

void BSP_STOP_W_IRQHandler(void);

//void TIM1_CC_IRQHandler(void);
//void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM6_IRQHandler(void);
//void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
//void TIM1_UP_TIM10_IRQHandler(void);
//void TIM1_UP_TIM10_IRQHandler(void);
void TIM14_IRQHandler(void);
void BSP_UART_DEBUG_IRQHandler(void);
void BSP_UART_LCD_IRQHandler(void);
//void BSP_WIFI_UART_IRQHandler(void);
void BSP_DMA_IRQHandler(void);
void BSP_ADC_IRQHandler(void);
//void DMA1_CH1_IRQHandler(void);
//void DMA1_CH2_3_IRQHandler(void);
//void SDIO_IRQHandler(void);
//void BSP_WIFI_UART_DMA_TX_IRQHandler(void);
//void BSP_WIFI_UART_DMA_RX_IRQHandler(void);
void USB_IRQHandler(void);
void USARTx_IRQHandler(void);
void USARTx_DMA_TX_RX_IRQHandler(void);
void TIMx_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
