/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
//#include "motorcontrol.h"
#include "stm32f0xx_3dprinter_uart.h"
#include "stm32f0xx_3dprinter_sd.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef hTimPwmX;
extern TIM_HandleTypeDef hTimPwmY;
extern TIM_HandleTypeDef hTimPwmZ;
extern TIM_HandleTypeDef hTimPwmE1;
#ifdef BSP_HEAT_E2_PIN
extern TIM_HandleTypeDef hTimPwmE2;
extern TIM_HandleTypeDef hTimPwmE3;
extern TIM_HandleTypeDef hTimPwmE4;
#endif//BSP_HEAT_E2_PIN
extern TIM_HandleTypeDef hTimTick;
extern TIM_HandleTypeDef hTimTick2;
#ifdef BSP_SERVO0_PIN
extern TIM_HandleTypeDef hTimServo;
#endif//BSP_SERVO0_PIN
extern BspAdcDataType gBspAdcData;
extern PCD_HandleTypeDef hpcd;
extern TIM_HandleTypeDef TimHandle;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

/**
  * @brief  This function handles interrupt for External lines 10 to 15
  * @param  None
  * @retval None
  */
void BSP_STOP_W_IRQHandler(void)
{
	//BSP ZMIN switch, triggers LED's
	HAL_GPIO_EXTI_IRQHandler(BSP_STOP_W_PIN);
}

/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
//void TIM1_CC_IRQHandler(void)
//{
////  HAL_TIM_IRQHandler(&hTimPwmX);
//}
/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
//void TIM2_IRQHandler(void)
//{
////  HAL_TIM_IRQHandler(&hTimPwmY);
//}
/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval None
  */
//void TIM3_IRQHandler(void)
//{
////  HAL_TIM_IRQHandler(&hTimPwmZ);
//}
/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
//void TIM4_IRQHandler(void)
//{
////  HAL_TIM_IRQHandler(&hTimPwmE1);
//}

/**
* @brief This function handles TIM5 global interrupt.
*/
//TODO: check that these timer handler's match up with that they should be doing
void TIM3_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
  HAL_TIM_IRQHandler(&hTimTick);

}


/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
#ifdef BSP_FAN_E2_PIN
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimPwmE2);
}
#endif//BSP_FAN_E2_PIN
/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
#if defined(BSP_FAN_E2_PIN) || defined(BSP_SERVO0_PIN)
void TIM1_UP_TIM10_IRQHandler(void)
{
#ifdef MARLIN
  HAL_TIM_IRQHandler(&hTimServo);
#else
  HAL_TIM_IRQHandler(&hTimPwmE3);
#endif
}
#endif//(BSP_FAN_E2_PIN) || (BSP_SERVO0_PIN)
/**
  * @brief  This function handles TIM1 interrupt request.
  * @param  None
  * @retval None
  */
void TIM14_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(TIM14_IRQn);
	HAL_TIM_IRQHandler(&hTimTick2);
}
//void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
//{
//#ifdef MARLIN
//  HAL_NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
//  HAL_TIM_IRQHandler(&hTimTick2);
//#else
//  HAL_TIM_IRQHandler(&hTimPwmE4);
//#endif
//
//}


/**
  * @brief  This function handles UART interrupt request for debug.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data transmission
  */
//TODO: should use a different handle between debug UART and LCD UART
void BSP_UART_DEBUG_IRQHandler(void)
{
  HAL_UART_IRQHandler(&gBspUartData.handle);
}

/**
  * @brief  This function handles LCD interrupt request for debug.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data transmission
  */
#ifdef MALYAN_LCD
void BSP_UART_LCD_IRQHandler(void)
{
  HAL_UART_IRQHandler(&gBspUartData.handle);
}
#endif

/**
* @brief This function handles DMA global interrupt.
*/
void BSP_DMA_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(BSP_DMA_IRQn);
  HAL_DMA_IRQHandler(&(gBspAdcData.dmaHandle));
}

/**
* @brief This function handles ADC global interrupts.
*/
void BSP_ADC_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(BSP_ADC_IRQn);
  HAL_ADC_IRQHandler(&(gBspAdcData.adcHandle));
}

#ifdef STM32_USE_USB_CDC
/**
  * @brief  This function handles USB Handler.
  * @param  None
  * @retval None
  */
void USB_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_IRQHandler(void) {
	HAL_TIM_IRQHandler(&TimHandle);
}
#endif
void HardFault_Handler(void)
{
	BSP_MiscErrorHandler(0);
}

