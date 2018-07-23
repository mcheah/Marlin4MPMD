/** 
  ******************************************************************************
  * @file    mpmd_3dPrinter_adc.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   Header for adc functions of 3D Printer BSP driver 
  *  (based on L6474)
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPMD_3DPRINTER_ADC_H
#define __MPMD_3DPRINTER_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "Configuration_STM.h"
   
/* Exported macros ------------------------------------------------------------*/

/* Definition for ADC resources *********************************************/
#define BSP_ADC                 (ADC1)
#define __BSP_ADC_CLK_ENABLE()    __ADC1_CLK_ENABLE()
#define __BSP_ADC_CLK_DISABLE()   __ADC1_CLK_DISABLE()   
   
#define __BSP_ADC_FORCE_RESET()      __ADC_FORCE_RESET()
#define __BSP_ADC_RELEASE_RESET()    __ADC_RELEASE_RESET()   
   
#define BSP_ADC_CHANNEL_THERM_BED1   (ADC_CHANNEL_4)
#ifdef E1_ADC_PA2
#define BSP_ADC_CHANNEL_THERM_E1    (ADC_CHANNEL_2)
#else
#define BSP_ADC_CHANNEL_THERM_E1    (ADC_CHANNEL_0)
#endif
//TODO: remove unused thermistor values, hardcode to 0 for now
//#define BSP_ADC_CHANNEL_THERM_E2    (ADC_CHANNEL_0)
//#define BSP_ADC_CHANNEL_THERM_E3    (ADC_CHANNEL_0)
//#define BSP_ADC_CHANNEL_THERM_BED2  (ADC_CHANNEL_0)
//#define BSP_ADC_CHANNEL_THERM_BED3 (ADC_CHANNEL_0)

//Order ranks to have growing channels for compatibility with F0, L0...
#define BSP_ADC_RANK_THERM_BED1   (2)
#define BSP_ADC_RANK_THERM_E1    (1)
//#define BSP_ADC_RANK_THERM_E2    (2)
//#define BSP_ADC_RANK_THERM_E3    (3)
//#define BSP_ADC_RANK_THERM_BED2   (6)
//#define BSP_ADC_RANK_THERM_BED3   (4)

/* Definition for  thermistor Pins */
#define BSP_THERM_BED1_PIN              (GPIO_PIN_4)
#define BSP_THERM_BED1_PORT             (GPIOA)
//#define BSP_THERM_BED2_PIN             (GPIO_PIN_4)
//#define BSP_THERM_BED2_PORT            (GPIOA)
#ifdef E1_ADC_PA2
#define BSP_THERM_E1_PIN               (GPIO_PIN_2)
#else
#define BSP_THERM_E1_PIN               (GPIO_PIN_0)
#endif
#define BSP_THERM_E1_PORT              (GPIOA)
//#define BSP_THERM_E2_PIN               (GPIO_PIN_0)
//#define BSP_THERM_E2_PORT              (GPIOA)
//#define BSP_THERM_E3_PIN               (GPIO_PIN_0)
//#define BSP_THERM_E3_PORT              (GPIOA)
//#define BSP_THERM_BED3_PIN             (GPIO_PIN_0)
//#define BSP_THERM_BED3_PORT            (GPIOA)

   /* Definition for IR Out Pin used for Z probing*/

/* Definition of ADC NVIC resources */
#define BSP_ADC_IRQn                   (ADC1_IRQn)
#define BSP_ADC_IRQHandler             (ADC_IRQHandler)
   
/* Definition for DMA resources used by ADC************************************/

#define BSP_DMA                 (DMA1_Channel1_BASE)
//DMA channels not re-mappable on F0
//#define BSP_DMA_CHANNEL         (DMA_CHANNEL_0)
#define __BSP_DMA_CLK_ENABLE()    __DMA1_CLK_ENABLE()

#define BSP_DMA_IRQn            (DMA1_Channel1_IRQn)
#define BSP_DMA_IRQHandler      (DMA1_Ch1_IRQHandler)
   
/* Exported types --- --------------------------------------------------------*/
typedef struct BspAdcDataTag
{
  ADC_HandleTypeDef adcHandle;
  DMA_HandleTypeDef dmaHandle;
  uint8_t acquisitionDone;
}BspAdcDataType;

/* Exported variables  --------------------------------------------------------*/
extern BspAdcDataType gBspAdcData;

/* Exported functions --------------------------------------------------------*/
void BSP_AdcHwInit(void);
uint16_t BSP_AdcGetValue(uint8_t rankId);

#ifdef __cplusplus
}
#endif

#endif /* __MPMD_3DPRINTER_ADC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
