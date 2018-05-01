/** 
  ******************************************************************************
  * @file    mpmd_3dPrinter_motor.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   Header for motor functions of 3D Printer BSP driver 
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
#ifndef __MPMD_3DPRINTER_MOTOR_H
#define __MPMD_3DPRINTER_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f0xx_nucleo.h"
//#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"
#include "motorcontrol.h"
#include "stm32f0xx_3dprinter_uart.h"
//#include "stm32f0xx_3dprinter_sd.h"
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup stm32f0XX_3DPRINTER_MOTOR
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup stm32f0XX_3DPRINTER_MOTOR_Exported_Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_stm32f0XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_stm32f0XX_3DPRINTER_MOTOR_MOTOR
* @{
*/   
/// Interrupt line used for L6474 FLAG
#define BSP_MOTOR_CONTROL_BOARD_FLAG_IRQn           (EXTI4_15_IRQn)
//TODO: change timers to unused timers to avoid contention
/// Timer used for PWM_X
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X                   (TIM1)
/// Channel Timer used for PWM_X
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_X              (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_X
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_X      (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X_CLCK_ENABLE()   __TIM1_CLK_ENABLE()
/// Timer Clock Disable for PWM1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X_CLCK_DISABLE()  __TIM1_CLK_DISABLE()
/// PWM_X global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM_X_IRQn                    (TIM1_CC_IRQn)      
/// PWM1 GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_X                (GPIO_AF2_TIM1)
/// PWM1 frequency rescaler (1 for HW PWM, 2 for SW PWM)
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X_FREQ_RESCALER            (1)  
   
/// Timer used for PWM_Y
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y                   (TIM3)
/// Channel Timer used for PWM_Y
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Y              (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_Y
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_Y      (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_Y
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y_CLCK_ENABLE()   __TIM3_CLK_ENABLE()
/// Timer Clock Disable for PWM_Y
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y_CLCK_DISABLE()  __TIM3_CLK_DISABLE()
/// PWM_Y global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM_Y_IRQn                    (TIM3_IRQn)
/// PWM_Y GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_Y                 (GPIO_AF0_TIM3)
/// PWM_Y frequency rescaler (1 for HW PWM, 2 for SW PWM)
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y_FREQ_RESCALER           (1)    
   
/// Timer used for PWM_Z
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z                   (TIM14)
/// Channel Timer used for PWM_Z
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Z              (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_Z
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_Z      (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_Z
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z_CLCK_ENABLE()    __TIM14_CLK_ENABLE()
/// Timer Clock Disable for PWM_Z
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z_CLCK_DISABLE()   __TIM14_CLK_DISABLE()
/// PWM_Z global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM_Z_IRQn              (TIM14_IRQn)
/// PWM_Z GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_Z                (GPIO_AF0_TIM14)
/// PWM_Z frequency rescaler (1 for HW PWM, 2 for SW PWM)   
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z_FREQ_RESCALER     (1)

/// Timer used for PWM_E1
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1                  (TIM15)
/// Channel Timer used for PWM_E1
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E1             (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_E1
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_E1     (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_E1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1_CLCK_ENABLE()  __TIM15_CLK_ENABLE()
/// Timer Clock Disable for PWM_E1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1_CLCK_DISABLE() __TIM15_CLK_DISABLE()
/// PWM_E1 global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM_E1_IRQn             (TIM15_IRQn)
/// PWM_E1 GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_E1               (GPIO_AF0_TIM15)
/// PWM_E1 frequency rescaler (1 for HW PWM, 2 for SW PWM)   
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1_FREQ_RESCALER    (1)   
   
/// Timer used for PWM_E2
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2                  (TIM17)
/// Channel Timer used for PWM_E2
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E2             (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_E2
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_E2     (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_E2
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2_CLCK_ENABLE()  __TIM17_CLK_ENABLE()
/// Timer Clock Disable for PWM_E2
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2_CLCK_DISABLE() __TIM17_CLK_DISABLE()
/// PWM_E2 global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM_E2_IRQn             (TIM17_IRQn)
/// PWM_E2 GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_E2               (GPIO_AF2_TIM17)
/// PWM_E2 frequency rescaler (1 for HW PWM, 2 for SW PWM)   
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2_FREQ_RESCALER    (1)   

/// Timer used for PWM_E3
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3                  (TIM17)
/// Channel Timer used for PWM_E3
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E3             (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_E3
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_E3     (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_E3
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3_CLCK_ENABLE()  __TIM17_CLK_ENABLE()
/// Timer Clock Disable for PWM_E3
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3_CLCK_DISABLE() __TIM17_CLK_DISABLE()
/// PWM_E3 global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM_E3_IRQn             (TIM17_IRQn)
/// PWM_E3 GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_E3               (GPIO_AF2_TIM17)
/// PWM_E3 frequency rescaler (1 for HW PWM, 2 for SW PWM)   
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3_FREQ_RESCALER    (1)   
  
 /**
* @}
*/

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

   /** @defgroup Constants_For_Motor_GPIO_Mapping
* @{
*/   
//TODO:remove flag pin
/// GPIO Pin used for the L6474 flag pin
#define BSP_MOTOR_CONTROL_BOARD_FLAG_PIN   (GPIO_PIN_1)
/// GPIO port used for the L6474 flag pin
#define BSP_MOTOR_CONTROL_BOARD_FLAG_PORT   (GPIOF)

/// GPIO Pin used for the L6474 step clock pin of device 0  
#define BSP_MOTOR_CONTROL_BOARD_PWM_X_PIN  (GPIO_PIN_14)
/// GPIO Port used for the L6474 step clock pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_PWM_X_PORT  (GPIOB)

/// GPIO Pin used for the L6474 step clock pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_Y_PIN  (GPIO_PIN_12)
/// GPIO port used for the L6474 step clock pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_Y_PORT  (GPIOB)
  
/// GPIO Pin used for the L6474 step clock pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_Z_PIN  (GPIO_PIN_2)
/// GPIO port used for the L6474 step clock pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_Z_PORT  (GPIOB)
   
/// GPIO Pin used for the L6474 step clock pin of device 3
#define BSP_MOTOR_CONTROL_BOARD_PWM_E1_PIN   (GPIO_PIN_7)
/// GPIO port used for the L6474 step clock pin of device 3
#define BSP_MOTOR_CONTROL_BOARD_PWM_E1_PORT  (GPIOA)

/// GPIO Pin used for the L6474 step clock pin of device 4
#define BSP_MOTOR_CONTROL_BOARD_PWM_E2_PIN   (GPIO_PIN_5)
/// GPIO port used for the L6474 step clock pin of device 4
#define BSP_MOTOR_CONTROL_BOARD_PWM_E2_PORT  (GPIOF)

/// GPIO Pin used for the L6474 step clock pin of device 5
#define BSP_MOTOR_CONTROL_BOARD_PWM_E3_PIN   (GPIO_PIN_8)
/// GPIO port used for the L6474 step clock pin of device 5
#define BSP_MOTOR_CONTROL_BOARD_PWM_E3_PORT  (GPIOB)
   
/// GPIO Pin used for the L6474 direction pin of device 0                      *
#define BSP_MOTOR_CONTROL_BOARD_DIR_X_PIN  (GPIO_PIN_13)
/// GPIO port used for the L6474 direction pin of device 0                     *
#define BSP_MOTOR_CONTROL_BOARD_DIR_X_PORT  (GPIOB)

/// GPIO Pin used for the L6474 direction pin of device 1                      *
#define BSP_MOTOR_CONTROL_BOARD_DIR_Y_PIN   (GPIO_PIN_11)
/// GPIO port used for the L6474 direction pin of device 1                     *
#define BSP_MOTOR_CONTROL_BOARD_DIR_Y_PORT  (GPIOB)

/// GPIO Pin used for the L6474 direction pin of device 2                      *
#define BSP_MOTOR_CONTROL_BOARD_DIR_Z_PIN   (GPIO_PIN_1)
/// GPIO port used for the L6474 direction pin of device 2                     *
#define BSP_MOTOR_CONTROL_BOARD_DIR_Z_PORT  (GPIOB)

/// GPIO Pin used for the L6474 direction pin of device 3                      *
#define BSP_MOTOR_CONTROL_BOARD_DIR_E1_PIN   (GPIO_PIN_6)
/// GPIO port used for the L6474 direction pin of device 3                     *
#define BSP_MOTOR_CONTROL_BOARD_DIR_E1_PORT  (GPIOA)
   
/// GPIO Pin used for the L6474 direction pin of device 4
#define BSP_MOTOR_CONTROL_BOARD_DIR_E2_PIN   (GPIO_PIN_6)
/// GPIO port used for the L6474 direction pin of device 4
#define BSP_MOTOR_CONTROL_BOARD_DIR_E2_PORT  (GPIOF)

/// GPIO Pin used for the L6474 direction pin of device 5
#define BSP_MOTOR_CONTROL_BOARD_DIR_E3_PIN   (GPIO_PIN_2)
/// GPIO port used for the L6474 direction pin of device 5
#define BSP_MOTOR_CONTROL_BOARD_DIR_E3_PORT  (GPIOF)
//TODO: rename RESET to enable etc.  Although all are shared so who knows
/// GPIO Pin used for the L6474 reset pin (device 0)                           *
#define BSP_MOTOR_CONTROL_BOARD_RESET_X_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6474 reset  (device 0)                             *
#define BSP_MOTOR_CONTROL_BOARD_RESET_X_PORT (GPIOB)
   
/// GPIO Pin used for the L6474 reset pin  (device 1)                          *
#define BSP_MOTOR_CONTROL_BOARD_RESET_Y_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6474 reset pin (device 1)                          *
#define BSP_MOTOR_CONTROL_BOARD_RESET_Y_PORT (GPIOB)

/// GPIO Pin used for the L6474 reset pin (device 2)                           *
#define BSP_MOTOR_CONTROL_BOARD_RESET_Z_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6474 reset pin (device 2)                          *
#define BSP_MOTOR_CONTROL_BOARD_RESET_Z_PORT (GPIOB)
   
/// GPIO Pin used for the L6474 reset pin (device 3)                           *
#define BSP_MOTOR_CONTROL_BOARD_RESET_E1_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6474 reset pin (device 3)                          *
#define BSP_MOTOR_CONTROL_BOARD_RESET_E1_PORT (GPIOB)
   
/// GPIO Pin used for the L6474 reset pin (device 4)
#define BSP_MOTOR_CONTROL_BOARD_RESET_E2_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6474 reset pin (device 4)
#define BSP_MOTOR_CONTROL_BOARD_RESET_E2_PORT (GPIOB)

/// GPIO Pin used for the L6474 reset pin (device 5)
#define BSP_MOTOR_CONTROL_BOARD_RESET_E3_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6474 reset pin (device 5)
#define BSP_MOTOR_CONTROL_BOARD_RESET_E3_PORT (GPIOB)
//TODO: remove SPI support since we are not using L6474
/// GPIO Pin used for the L6474 SPI chip select pin
#define BSP_MOTOR_CONTROL_BOARD_CS_PIN  (GPIO_PIN_4)
/// GPIO port used for the L6474 SPI chip select  pin
#define BSP_MOTOR_CONTROL_BOARD_CS_PORT (GPIOA)

/* Definition for SPIx clock resources */

/// Used SPI
#define SPIx                             (SPI1)

/// SPI clock enable
#define SPIx_CLK_ENABLE()                __SPI1_CLK_ENABLE()

/// SPI SCK enable
#define SPIx_SCK_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()

/// SPI MISO enable
#define SPIx_MISO_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 

/// SPI MOSI enable
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 

/// SPI Force reset
#define SPIx_FORCE_RESET()               __SPI1_FORCE_RESET()

/// SPI Release reset
#define SPIx_RELEASE_RESET()             __SPI1_RELEASE_RESET()

/// SPI SCK pin
#define SPIx_SCK_PIN                     (GPIO_PIN_5)

/// SPI SCK port
#define SPIx_SCK_GPIO_PORT               (GPIOA)

/// SPI MISO pin 
#define SPIx_MISO_PIN                    (GPIO_PIN_6)

/// SPI MISO port
#define SPIx_MISO_GPIO_PORT              (GPIOA)

/// SPI MOSI pin
#define SPIx_MOSI_PIN                    (GPIO_PIN_7)

/// SPI MOSI port
#define SPIx_MOSI_GPIO_PORT              (GPIOA)

/// SPI_SCK alternate function   
#define SPIx_SCK_AF                      (GPIO_AF2_TIM1)
   
/// SPI MISO AF 
#define SPIx_MISO_AF                     (SPIx_SCK_AF)

/// SPI MOSI AF
#define SPIx_MOSI_AF                     (SPIx_SCK_AF)

/**
  * @}
  */

/**
  * @}
  */


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __MPMD_3DPRINTER_MOTOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
