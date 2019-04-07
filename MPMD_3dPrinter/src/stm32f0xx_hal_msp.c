/**
  ******************************************************************************
  * @file    stm32f0xx_hal_msp.c
  * @author  mcheah
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied 
  *          to the user folder.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 MCheah</center></h2>
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
#include "main.h"
#include "Marlin_export.h"

/** @defgroup MSP_module
  * @brief HAL MSP module.
  * @{
  */

/* Imported variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef hTimPwmX;
extern TIM_HandleTypeDef hTimPwmY;
extern TIM_HandleTypeDef hTimPwmZ;
extern TIM_HandleTypeDef hTimPwmE1;
extern TIM_HandleTypeDef hTimPwmE2;
extern TIM_HandleTypeDef hTimPwmE3;

extern TIM_HandleTypeDef hTimPwmHeatBed;
extern TIM_HandleTypeDef hTimPwmHeatBed2;
extern TIM_HandleTypeDef hTimPwmHeatBed3;
extern TIM_HandleTypeDef hTimPwmHeatE1;
extern TIM_HandleTypeDef hTimPwmHeatE2;
extern TIM_HandleTypeDef hTimPwmHeatE3;

//extern BspAdcDataType gBspAdcData;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    
/* Private function prototypes -----------------------------------------------*/
extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId); 
extern void BSP_MotorControl_FlagInterruptHandler(void);
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param[in] hspi SPI handle pointer
  * @retval None
  */
#if defined(SPIx) || defined(SPI_USER)
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(hspi->Instance == SPIx)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI clock */
    SPIx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;

    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;

    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;

    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
  }
#ifdef SPI_USER
  else if (hspi->Instance == SPI_USER)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPI_USER_SCK_GPIO_CLK_ENABLE();
    SPI_USER_MISO_GPIO_CLK_ENABLE();
    SPI_USER_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI clock */
    SPI_USER_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPI_USER_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = SPI_USER_SCK_AF;

    HAL_GPIO_Init(SPI_USER_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPI_USER_MISO_PIN;
    GPIO_InitStruct.Alternate = SPI_USER_MISO_AF;

    HAL_GPIO_Init(SPI_USER_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPI_USER_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPI_USER_MOSI_AF;

    HAL_GPIO_Init(SPI_USER_MOSI_GPIO_PORT, &GPIO_InitStruct);
  }
#endif//SPI_USER
}

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param[in] hspi SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPIx)
  {
    /*##-1- Reset peripherals ##################################################*/
    SPIx_FORCE_RESET();
    SPIx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
  }
#ifdef SPI_USER
  else if (hspi->Instance == SPI_USER)
  {
    /*##-1- Reset peripherals ##################################################*/
    SPI_USER_FORCE_RESET();
    SPI_USER_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(SPI_USER_SCK_GPIO_PORT, SPI_USER_SCK_PIN);
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(SPI_USER_MISO_GPIO_PORT, SPI_USER_MISO_PIN);
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(SPI_USER_MOSI_GPIO_PORT, SPI_USER_MOSI_PIN);
  }
#endif
}
#endif//SPIx || SPI_USER
/**
  * @brief UArt MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param[in] huart UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
#if !defined(STM32_USE_USB_CDC)
  if(huart->Instance == BSP_UART_DEBUG)
  {
    /* Peripheral clock enable */
    __BSP_UART_DEBUG_CLK_ENABLE();
  
    /* Enable GPIO TX/RX clock */
    __BSP_UART_DEBUG_TX_GPIO_CLK_ENABLE();
    __BSP_UART_DEBUG_RX_GPIO_CLK_ENABLE();
    
    /* USART TX GPIO Configuration */
    GPIO_InitStruct.Pin = BSP_UART_DEBUG_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_UART_DEBUG_TX_AF;
    HAL_GPIO_Init(BSP_UART_DEBUG_TX_PORT, &GPIO_InitStruct);

    /* USART RX GPIO Configuration     */    
    GPIO_InitStruct.Pin = BSP_UART_DEBUG_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_UART_DEBUG_RX_AF;
    HAL_GPIO_Init(BSP_UART_DEBUG_RX_PORT, &GPIO_InitStruct);
    
    /* Configure the NVIC for UART */
	//No hardware FIFO for UART, so must be highest priority to prevent missing characters
    HAL_NVIC_SetPriority(BSP_UART_DEBUG_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BSP_UART_DEBUG_IRQn);    
  }
#else
  if(0) { }
#endif
  else if(huart->Instance == BSP_UART_LCD)
  {
    /* Peripheral clock enable */
    __BSP_UART_LCD_CLK_ENABLE();
    /* Enable GPIO TX/RX clock */
    __BSP_UART_LCD_TX_GPIO_CLK_ENABLE();
    __BSP_UART_LCD_RX_GPIO_CLK_ENABLE();

    /* USART TX GPIO Configuration */
    GPIO_InitStruct.Pin = BSP_UART_LCD_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_UART_LCD_TX_AF;
    HAL_GPIO_Init(BSP_UART_LCD_TX_PORT, &GPIO_InitStruct);

    /* USART RX GPIO Configuration     */
    GPIO_InitStruct.Pin = BSP_UART_LCD_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_UART_LCD_RX_AF;
    HAL_GPIO_Init(BSP_UART_LCD_RX_PORT, &GPIO_InitStruct);

    /* Configure the NVIC for UART */
	//No hardware FIFO for UART, so must be highest priority to prevent missing characters
    HAL_NVIC_SetPriority(BSP_UART_LCD_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BSP_UART_LCD_IRQn);
  }
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param[in] huart UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
#if !defined(STM32_USE_USB_CDC)
  if(huart->Instance == BSP_UART_DEBUG)
  {
   /* Reset peripherals */
    __BSP_UART_DEBUG_FORCE_RESET();
    __BSP_UART_DEBUG_RELEASE_RESET();

    /* Disable peripherals and GPIO Clocks */
    HAL_GPIO_DeInit(BSP_UART_DEBUG_TX_PORT, BSP_UART_DEBUG_TX_PIN);
    HAL_GPIO_DeInit(BSP_UART_DEBUG_RX_PORT, BSP_UART_DEBUG_RX_PIN);
    
    /* Disable the NVIC for UART */
    HAL_NVIC_DisableIRQ(BSP_UART_DEBUG_IRQn);

    /* Peripheral clock disable */
    __BSP_UART_DEBUG_CLK_DISABLE();
  }
#else
  if(0) { }
#endif
  else if(huart->Instance == BSP_UART_LCD)
  {
   /* Reset peripherals */
    __BSP_UART_LCD_FORCE_RESET();
    __BSP_UART_LCD_RELEASE_RESET();

    /* Disable peripherals and GPIO Clocks */
    HAL_GPIO_DeInit(BSP_UART_LCD_TX_PORT, BSP_UART_LCD_TX_PIN);
    HAL_GPIO_DeInit(BSP_UART_LCD_RX_PORT, BSP_UART_LCD_RX_PIN);

    /* Disable the NVIC for UART */
    HAL_NVIC_DisableIRQ(BSP_UART_LCD_IRQn);

    /* Peripheral clock disable */
    __BSP_UART_LCD_CLK_DISABLE();
  }
}


/**
  * @brief PWM MSP Initialization 
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */

/**
  * @brief PWM Callback
  * @param[in] htim PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
#ifdef BSP_HEAT_E2_PIN
  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2)&& (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_E2))
  {
    HAL_GPIO_TogglePin(BSP_MOTOR_CONTROL_BOARD_PWM_E2_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_E2_PIN);
    if ((BSP_MotorControl_GetDeviceState(4) != INACTIVE)&& 
        (HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PWM_E2_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_E2_PIN) == GPIO_PIN_SET))
    {
      BSP_MotorControl_StepClockHandler(4);
    }
  }
#endif
#ifndef MARLIN
  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3)&& (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_E3))
  {
    HAL_GPIO_TogglePin(BSP_MOTOR_CONTROL_BOARD_PWM_E3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_E3_PIN);
    if ((BSP_MotorControl_GetDeviceState(5) != INACTIVE)&& 
        (HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PWM_E3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_E3_PIN) == GPIO_PIN_SET))
    {
      BSP_MotorControl_StepClockHandler(5);
    }
  }
#endif  
  if ((htim->Instance == BSP_MISC_TIMER_TICK)&& (htim->Channel == BSP_MISC_HAL_ACT_CHAN_TIMER_TICK))
  {
#ifdef MARLIN
    IsrStepperHandler();
#else    
    TC3_Handler();
#endif    
    
  }
#if defined(MARLIN) && defined(BSP_SERVO0_PIN)
  if ((htim->Instance == BSP_MISC_TIMER_SERVO)&& (htim->Channel == BSP_MISC_HAL_ACT_CHAN_TIMER_SERVO))
  {
    TimerStService();
  }    
#endif
#ifdef MARLIN
  if ((htim->Instance == BSP_MISC_TIMER_TICK2)&& (htim->Channel == BSP_MISC_HAL_ACT_CHAN_TIMER_TICK2))
  {
    IsrTemperatureHandler();
  }  
#endif
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
{
  if(htim->Instance == TIMx)
  {
    /* Peripheral clock enable */
	TIMx_CLK_ENABLE();

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);   BDI : not needed. Done once at startup
    HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim)
{

  if(htim->Instance == TIMx)
  {
    /* Peripheral clock disable */
	  TIMx_CLK_ENABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIMx_IRQn);
  }
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
  if(htim_oc->Instance == BSP_MISC_TIMER_TICK)
  {
    /* Peripheral clock enable */
    __BSP_MISC_TIMER_TICK_CLCK_ENABLE();

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);   BDI : not needed. Done once at startup
    HAL_NVIC_SetPriority(BSP_MISC_TICK_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(BSP_MISC_TICK_IRQn);
  }
#ifdef MARLIN
  if(htim_oc->Instance == BSP_MISC_TIMER_TICK2)
  {
    /* Peripheral clock enable */
    __BSP_MISC_TIMER_TICK2_CLCK_ENABLE();

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);  BDI
    //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);  BDI : not needed. Done once at startup
    HAL_NVIC_SetPriority(BSP_MISC_TICK2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BSP_MISC_TICK2_IRQn);
  }  
#ifdef BSP_SERVO0_PIN
  if(htim_oc->Instance == BSP_MISC_TIMER_SERVO)
  {
    /* Peripheral clock enable */
    __BSP_MISC_TIMER_SERVO_CLCK_ENABLE();

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    // HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);  BDI
    //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);  BDI : not needed. Done once at startup

    HAL_NVIC_SetPriority(BSP_MISC_SERVO_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(BSP_MISC_SERVO_IRQn);
  } 
#endif//BSP_SERVO0_PIN
#endif//MARLIN
}

void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* htim_oc)
{

  if(htim_oc->Instance == BSP_MISC_TIMER_TICK)
  {
    /* Peripheral clock disable */
    __BSP_MISC_TIMER_TICK_CLCK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(BSP_MISC_TICK_IRQn);
  }
#ifdef MARLIN
  if(htim_oc->Instance == BSP_MISC_TIMER_TICK2)
  {
    /* Peripheral clock disable */
    __BSP_MISC_TIMER_TICK2_CLCK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(BSP_MISC_TICK2_IRQn);
  }  
#ifdef BSP_SERVO0_PIN
  if(htim_oc->Instance == BSP_MISC_TIMER_SERVO)
  {
    /* Peripheral clock disable */
    __BSP_MISC_TIMER_SERVO_CLCK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(BSP_MISC_SERVO_IRQn);
  }    
#endif//BSP_SERVO0_PIN
#endif//MARLIN
}

/**
  * @brief External Line Callback
  * @param[in] GPIO_Pin pin number
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BSP_STOP_W_PIN)
	{
		if(HAL_GPIO_ReadPin(BSP_STOP_W_PORT,BSP_STOP_W_PIN)==GPIO_PIN_RESET)
			BSP_LED_On(LED_RED);
		else
			BSP_LED_Off(LED_RED);
	}
 }


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hadc->Instance == BSP_ADC)
  {
    DMA_HandleTypeDef *pDmaHandle = &(gBspAdcData.dmaHandle);
    
    /* ADC clock enable */
    __BSP_ADC_CLK_ENABLE();
  
    /* DMA controller clock enable */
    __BSP_DMA_CLK_ENABLE();
    
    /* ADC1 GPIO Configuration    */
    GPIO_InitStruct.Pin = BSP_THERM_BED1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_THERM_BED1_PORT, &GPIO_InitStruct);
#ifdef BSP_THERM_BED2_PIN
    GPIO_InitStruct.Pin = BSP_THERM_BED2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_THERM_BED2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_THERM_BED3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_THERM_BED3_PORT, &GPIO_InitStruct);
#endif//BSP_THERM_BED2_PIN
    GPIO_InitStruct.Pin = BSP_THERM_E1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_THERM_E1_PORT, &GPIO_InitStruct);

#ifdef BSP_THERM_E2_PIN
    GPIO_InitStruct.Pin = BSP_THERM_E2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_THERM_E2_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = BSP_THERM_E3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_THERM_E3_PORT, &GPIO_InitStruct);
#endif//BSP_THERM_E2_PIN
    

    /* Peripheral DMA init*/
    pDmaHandle->Instance = BSP_DMA;
    //TODO: adjust DMA init options
//    pDmaHandle->Init.Channel = BSP_DMA_CHANNEL;
    pDmaHandle->Init.Direction = DMA_PERIPH_TO_MEMORY;
    pDmaHandle->Init.PeriphInc = DMA_PINC_DISABLE;
    pDmaHandle->Init.MemInc = DMA_MINC_ENABLE;
    pDmaHandle->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    pDmaHandle->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    pDmaHandle->Init.Mode = DMA_CIRCULAR;
    pDmaHandle->Init.Priority = DMA_PRIORITY_LOW;
//    pDmaHandle->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_DeInit(pDmaHandle);
    HAL_DMA_Init(pDmaHandle);

    __HAL_LINKDMA(hadc,DMA_Handle,gBspAdcData.dmaHandle);

    HAL_NVIC_SetPriority(BSP_DMA_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(BSP_DMA_IRQn);
    
    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriority(BSP_ADC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(BSP_ADC_IRQn);
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
    __BSP_ADC_FORCE_RESET();
    __BSP_ADC_RELEASE_RESET();   
    
    /* Peripheral clock disable */
    __BSP_ADC_CLK_DISABLE();
    
    /* ADC GPIO Deconfiguration    */
    HAL_GPIO_DeInit(BSP_THERM_BED1_PORT, BSP_THERM_BED1_PIN);
#ifdef BSP_THERM_BED2_PIN
    HAL_GPIO_DeInit(BSP_THERM_BED2_PORT, BSP_THERM_BED2_PIN);
    HAL_GPIO_DeInit(BSP_THERM_BED3_PORT, BSP_THERM_BED3_PIN);
#endif//BSP_THERM_BED2_PIN
    HAL_GPIO_DeInit(BSP_THERM_E1_PORT, BSP_THERM_E1_PIN);
#ifdef BSP_THERM_E2_PIN
    HAL_GPIO_DeInit(BSP_THERM_E2_PORT, BSP_THERM_E2_PIN);
    HAL_GPIO_DeInit(BSP_THERM_E3_PORT, BSP_THERM_E3_PIN);
#endif//BSP_THERM_E2_PIN
    
    /* Peripheral DMA DeInit*/
    if(hadc->DMA_Handle != NULL)
    {
      HAL_DMA_DeInit(hadc->DMA_Handle);
    }

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(BSP_DMA_IRQn);
    HAL_NVIC_DisableIRQ(BSP_ADC_IRQn);
  }

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
