/**
  ******************************************************************************
  * @file    stm32f0xx_3dPrinter_adc.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   adc and dma functions of 3D Printer BSP driver 
  *  (based on L6474)
  * @note    (C) COPYRIGHT 2015 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stm32f0xx_3dprinter_adc.h"
#include "stm32f0xx_3dprinter_misc.h"

/* Private defines ----------------------------------------------------------*/
/* Private constant ----------------------------------------------------------*/
#define ADC_ERROR_TAG        (0x2000)
#define ADC_ERROR(error)     BSP_MiscErrorHandler(error|ADC_ERROR_TAG)
//TODO: update this to correspond to the correct number of
#define BSP_ADC_NUM_CHANNELS (2)
#define BSP_ADC_OS_RATIO	 (16)
//#define BSP_ADC_CONVERTED_VALUES_BUFFER_SIZE (6)
#define BSP_ADC_CONVERTED_VALUES_BUFFER_SIZE (BSP_ADC_NUM_CHANNELS*BSP_ADC_OS_RATIO)
    
/* Global variables ---------------------------------------------------------*/
BspAdcDataType gBspAdcData;

/* Private variables */
__IO uint16_t   aBspAdcConvertedValues[BSP_ADC_CONVERTED_VALUES_BUFFER_SIZE];

/* Extern function -----------------------------------------------------------*/

/******************************************************//**
 * @brief  ADC Hw initialisation
 * @param None
 * @retval None
 **********************************************************/
void BSP_AdcHwInit(void)
{
  BspAdcDataType *pAdc = &gBspAdcData;
  
  ADC_ChannelConfTypeDef sConfig;
  pAdc->acquisitionDone = RESET;
    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  pAdc->adcHandle.Instance = BSP_ADC;
  pAdc->adcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  pAdc->adcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  pAdc->adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  pAdc->adcHandle.Init.ScanConvMode = ENABLE;
  pAdc->adcHandle.Init.EOCSelection = 0X0000;
  //TODO: check these lowpower registers to see if this is valid
  pAdc->adcHandle.Init.LowPowerAutoWait = DISABLE;
  pAdc->adcHandle.Init.LowPowerAutoPowerOff = DISABLE;
  pAdc->adcHandle.Init.ContinuousConvMode = ENABLE;
  //TODO: there's no NbrOfConversion register so we will have to loop manually
//  pAdc->adcHandle.Init.NbrOfConversion = BSP_ADC_CONVERTED_VALUES_BUFFER_SIZE;
  pAdc->adcHandle.Init.DiscontinuousConvMode = DISABLE;
//  pAdc->adcHandle.Init.NbrOfDiscConversion = 1;
  //TODO: check eternalTrigConv value
  pAdc->adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  pAdc->adcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  pAdc->adcHandle.Init.DMAContinuousRequests = ENABLE;
  //TODO: verify this
  pAdc->adcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
//  pAdc->adcHandle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  pAdc->adcHandle.Init.SamplingTimeCommon =
  
  if (HAL_ADC_Init(&pAdc->adcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    ADC_ERROR(1);
  }

  /* Configure ADC for bed thermistor */
  sConfig.Channel = BSP_ADC_CHANNEL_THERM_BED1;
  sConfig.Rank = BSP_ADC_RANK_THERM_BED1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&pAdc->adcHandle, &sConfig) != HAL_OK)
  {
    /* Channel configuration error */
    ADC_ERROR(2);
  }

  /* Configure ADC for E1 thermistor */
  sConfig.Channel = BSP_ADC_CHANNEL_THERM_E1;
  sConfig.Rank = BSP_ADC_RANK_THERM_E1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&pAdc->adcHandle, &sConfig) != HAL_OK)
  {
    /* Channel configuration error */
    ADC_ERROR(3);
  }
#ifdef BSP_THERM_E2_PIN
  /* Configure ADC for E2 thermistor */
  sConfig.Channel = BSP_ADC_CHANNEL_THERM_E2;
  sConfig.Rank = BSP_ADC_RANK_THERM_E2;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&pAdc->adcHandle, &sConfig) != HAL_OK)
  {
    /* Channel configuration error */
    ADC_ERROR(4);
  }

   /* Configure ADC for E3 thermistor */
  sConfig.Channel = BSP_ADC_CHANNEL_THERM_E3;
  sConfig.Rank = BSP_ADC_RANK_THERM_E3;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&pAdc->adcHandle, &sConfig) != HAL_OK)
  {
    /* Channel configuration error */
    ADC_ERROR(5);
  }
#endif
#ifdef BSP_THERM_BED2_PIN
   /* Configure ADC for BED2 thermistor */
  sConfig.Channel = BSP_ADC_CHANNEL_THERM_BED2;
  sConfig.Rank = BSP_ADC_RANK_THERM_BED2;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&pAdc->adcHandle, &sConfig) != HAL_OK)
  {
    /* Channel configuration error */
    ADC_ERROR(6);
  }
  
   /* Configure ADC for BED3*/
  sConfig.Channel = BSP_ADC_CHANNEL_THERM_BED3;
  sConfig.Rank = BSP_ADC_RANK_THERM_BED3;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&pAdc->adcHandle, &sConfig) != HAL_OK)
  {
    /* Channel configuration error */
    ADC_ERROR(7);
  }  
#endif
  HAL_ADCEx_Calibration_Start(&pAdc->adcHandle);

  /* Start conversion */
  if (HAL_ADC_Start_DMA(&pAdc->adcHandle,
                        (uint32_t *)aBspAdcConvertedValues, 
                         BSP_ADC_CONVERTED_VALUES_BUFFER_SIZE)   != HAL_OK)  
  {
      ADC_ERROR(8);
  }
    
}


/******************************************************//**
 * @brief  Conversion complete callback in non blocking mode
 * @param hadc: ADC handle
 * @retval None
 **********************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  BspAdcDataType *pAdc = &gBspAdcData;
  /* Report to main program that ADC sequencer has reached its end */
  pAdc->acquisitionDone = SET;
}

/******************************************************//**
 * @brief  Conversion DMA half-transfer callback in non blocking mode 
 * @param hadc: ADC handle
 * @retval None
 **********************************************************/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  
}

/******************************************************//**
 * @brief  DC error callback in non blocking mode
 *        (ADC conversion with interruption or transfer by DMA)
 * @param hadc: ADC handle
 * @retval None
 **********************************************************/
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  ADC_ERROR(9);
}

/******************************************************//**
 * @brief  ADC Hw initialisation
 * @param rankId: rank ok the analog pin to read 
 * (see BSP_ADC_RANK_... values)
 * @retval None
 **********************************************************/
uint16_t BSP_AdcGetValue(uint8_t rankId)
{
  BspAdcDataType *pAdc = &gBspAdcData;
  if (pAdc->acquisitionDone == RESET)
  {
      ADC_ERROR(10);
  }
  uint32_t adcSum = 0;
  for(int i=0;i<BSP_ADC_CONVERTED_VALUES_BUFFER_SIZE;i+=BSP_ADC_NUM_CHANNELS)
	  adcSum += aBspAdcConvertedValues[rankId+i];
  return (adcSum/BSP_ADC_OS_RATIO);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


