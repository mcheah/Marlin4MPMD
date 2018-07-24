/** 
  ******************************************************************************
  * @file    mpmd_3dPrinter_uart.h
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
#ifndef __MPMD_3DPRINTER_MISC_H
#define __MPMD_3DPRINTER_MISC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
   
/* Exported macros ------------------------------------------------------------*/

/* Definition for IR ON Pin used for Z probing*/
 //TODO: remove IR pin or rename to bed switch probe
//#define BSP_IR_ON_PIN               (GPIO_PIN_7)
//#define BSP_IR_ON_PORT              (GPIOB)

/* Definition for Stops */
 //TODO: remove unused optical stops
#define BSP_STOP_X_PIN               (GPIO_PIN_13)
#define BSP_STOP_X_PORT              (GPIOC)
#define STOP_X__PULL_UP
#define BSP_STOP_Y_PIN               (GPIO_PIN_14)
#define BSP_STOP_Y_PORT              (GPIOC)
#define STOP_Y__PULL_UP
#define BSP_STOP_Z_PIN               (GPIO_PIN_15)
#define BSP_STOP_Z_PORT              (GPIOC)
#define STOP_Z__PULL_UP
//#define BSP_STOP_U_PIN               (GPIO_PIN_13)
//#define BSP_STOP_U_PORT              (GPIOC)
//#define BSP_STOP_V_PIN               (GPIO_PIN_13)
//#define BSP_STOP_V_PORT              (GPIOC)
#define BSP_STOP_W_PIN               (GPIO_PIN_7)
#define BSP_STOP_W_PORT              (GPIOB)
#define STOP_W__PULL_UP
#define BSP_STOP_W_IRQN				 (EXTI4_15_IRQn)
#define BSP_STOP_W_IRQHandler        (EXTI4_15_IRQHandler)

/* Definition for Heats*/
 //TODO: removed unused heaters
#define BSP_HEAT_BED1_PIN              (GPIO_PIN_5)
#define BSP_HEAT_BED1_PORT             (GPIOA)
//#define BSP_HEAT_BED2_PIN             (GPIO_PIN_5)
//#define BSP_HEAT_BED2_PORT            (GPIOA)
//#define BSP_HEAT_BED3_PIN             (GPIO_PIN_5)
//#define BSP_HEAT_BED3_PORT            (GPIOA)
#define BSP_HEAT_E1_PIN               (GPIO_PIN_1)
#define BSP_HEAT_E1_PORT              (GPIOA)
//#define BSP_HEAT_E2_PIN               (GPIO_PIN_1)
//#define BSP_HEAT_E2_PORT              (GPIOA)
//#define BSP_HEAT_E3_PIN               (GPIO_PIN_1)
//#define BSP_HEAT_E3_PORT              (GPIOA)

/* Definition for FAN 0*/
//TODO: remove unused fans
#define BSP_FAN_PWM_FREQ			(75) //Desired PWM frequency in hertz

#define BSP_FAN_E1_PIN              (GPIO_PIN_8)
#define BSP_FAN_E1_PORT             (GPIOA)
//#define BSP_FAN_E2_PIN              (GPIO_PIN_8)
//#define BSP_FAN_E2_PORT             (GPIOA)
//#define BSP_FAN_E3_PIN              (GPIO_PIN_8)
//#define BSP_FAN_E3_PORT             (GPIOA)


/* Definition for Servo 0*/
//TODO: removed unused servo
//#define BSP_SERVO0_PIN               (GPIO_PIN_8)
//#define BSP_SERVO0_PORT              (GPIOA)
   
/* Definition for Tick timer */
/// Timer used for Tick
#define BSP_MISC_TIMER_TICK                  (TIM3)
/// Channel Timer used for Tick
#define BSP_MISC_CHAN_TIMER_TICK              (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for Tick
#define BSP_MISC_HAL_ACT_CHAN_TIMER_TICK      (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for Tick
#define __BSP_MISC_TIMER_TICK_CLCK_ENABLE()   __TIM3_CLK_ENABLE()
/// Timer Clock Disable for Tick
#define __BSP_MISC_TIMER_TICK_CLCK_DISABLE()  __TIM3_CLK_DISABLE()

/// TICK global interrupt
#define BSP_MISC_TICK_IRQn                    (TIM3_IRQn)
   
/* Definition for Tick2 timer  (Marlin Only)*/
/// Timer used for Tick2
#define BSP_MISC_TIMER_TICK2                  (TIM14)
/// Channel Timer used for Tick
#define BSP_MISC_CHAN_TIMER_TICK2              (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for Tick
#define BSP_MISC_HAL_ACT_CHAN_TIMER_TICK2      (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for Tick
#define __BSP_MISC_TIMER_TICK2_CLCK_ENABLE()   __TIM14_CLK_ENABLE()
/// Timer Clock Disable for Tick
#define __BSP_MISC_TIMER_TICK2_CLCK_DISABLE()  __TIM14_CLK_DISABLE()

/// TICK global interrupt
#define BSP_MISC_TICK2_IRQn                    (TIM14_IRQn)

//TODO: update these timers since only FAN PWM is connector to a timer channel
 /// Timer used for PWM_HEAT_BED
#define BSP_MISC_TIMER_PWM_HEAT_BED                  (TIM1)
/// Channel Timer used for PWM_HEAT_BED
#define BSP_MISC_CHAN_TIMER_PWM_HEAT_BED                  (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_HEAT_BED
#define BSP_MISC_HAL_ACT_CHAN_TIMER_PWM_HEAT_BED     (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_HEAT_BED
#define __BSP_MISC_TIMER_PWM_HEAT_BED_CLCK_ENABLE()  __TIM1_CLK_ENABLE()
/// Timer Clock Disable for PWM_HEAT_BED
#define __BSP_MISC_TIMER_PWM_HEAT_BED_CLCK_DISABLE() __TIM1_CLK_DISABLE()
/// PWM_HEAT_BED GPIO alternate function 
#define BSP_MISC_AFx_TIMx_PWM_HEAT_BED               (GPIO_AF2_TIM1)

///// Timer used for PWM_HEAT_BED2
//#define BSP_MISC_TIMER_PWM_HEAT_BED2                  (TIM1)
///// Channel Timer used for PWM_HEAT_BED2
//#define BSP_MISC_CHAN_TIMER_PWM_HEAT_BED2                  (TIM_CHANNEL_1)
///// HAL Active Channel Timer used for PWM_HEAT_BED2
//#define BSP_MISC_HAL_ACT_CHAN_TIMER_PWM_HEAT_BED2     (HAL_TIM_ACTIVE_CHANNEL_1)
///// Timer Clock Enable for PWM_HEAT_BED2
//#define __BSP_MISC_TIMER_PWM_HEAT_BED2_CLCK_ENABLE()  __TIM1_CLK_ENABLE()
///// Timer Clock Disable for PWM_HEAT_BED2
//#define __BSP_MISC_TIMER_PWM_HEAT_BED2_CLCK_DISABLE() __TIM1_CLK_DISABLE()
///// PWM_HEAT_BED2 GPIO alternate function
//#define BSP_MISC_AFx_TIMx_PWM_HEAT_BED2               (GPIO_AF2_TIM1)

///// Timer used for PWM_HEAT_BED3
//#define BSP_MISC_TIMER_PWM_HEAT_BED3                  (GPIO_AF2_TIM1)
///// Channel Timer used for PWM_HEAT_BED3
//#define BSP_MISC_CHAN_TIMER_PWM_HEAT_BED3                  (TIM_CHANNEL_1)
///// HAL Active Channel Timer used for PWM_HEAT_BED3
//#define BSP_MISC_HAL_ACT_CHAN_TIMER_PWM_HEAT_BED3     (HAL_TIM_ACTIVE_CHANNEL_1)
///// Timer Clock Enable for PWM_HEAT_BED3
//#define __BSP_MISC_TIMER_PWM_HEAT_BED3_CLCK_ENABLE()  __TIM1_CLK_ENABLE()
///// Timer Clock Disable for PWM_HEAT_BED3
//#define __BSP_MISC_TIMER_PWM_HEAT_BED3_CLCK_DISABLE() __TIM1_CLK_DISABLE()
///// PWM_HEAT_BED3 GPIO alternate function
//#define BSP_MISC_AFx_TIMx_PWM_HEAT_BED3               (GPIO_AF2_TIM1)
   
/// Timer used for PWM_HEAT_E1
#define BSP_MISC_TIMER_PWM_HEAT_E1                  (TIM15)
/// Channel Timer used for PWM_HEAT_E1
#define BSP_MISC_CHAN_TIMER_PWM_HEAT_E1                  (TIM_CHANNEL_1)
/// HAL Active Channel Timer used for PWM_HEAT_E1
#define BSP_MISC_HAL_ACT_CHAN_TIMER_PWM_HEAT_E1     (HAL_TIM_ACTIVE_CHANNEL_1)
/// Timer Clock Enable for PWM_HEAT_E1
#define __BSP_MISC_TIMER_PWM_HEAT_E1_CLCK_ENABLE()  __TIM15_CLK_ENABLE()
/// Timer Clock Disable for PWM_HEAT_E1
#define __BSP_MISC_TIMER_PWM_HEAT_E1_CLCK_DISABLE() __TIM15_CLK_DISABLE()
/// PWM_HEAT_E1 GPIO alternate function 
#define BSP_MISC_AFx_TIMx_PWM_HEAT_E1               (GPIO_AF3_TIM15)
   
///// Timer used for PWM_HEAT_E2
//#define BSP_MISC_TIMER_PWM_HEAT_E2                  (TIM15)
///// Channel Timer used for PWM_HEAT_E2
//#define BSP_MISC_CHAN_TIMER_PWM_HEAT_E2                  (TIM_CHANNEL_1)
///// HAL Active Channel Timer used for PWM_HEAT_E2
//#define BSP_MISC_HAL_ACT_CHAN_TIMER_PWM_HEAT_E2     (HAL_TIM_ACTIVE_CHANNEL_1)
///// Timer Clock Enable for PWM_HEAT_E2
//#define __BSP_MISC_TIMER_PWM_HEAT_E2_CLCK_ENABLE()  __TIM15_CLK_ENABLE()
///// Timer Clock Disable for PWM_HEAT_E2
//#define __BSP_MISC_TIMER_PWM_HEAT_E2_CLCK_DISABLE() __TIM15_CLK_DISABLE()
///// PWM_HEAT_E2 GPIO alternate function
//#define BSP_MISC_AFx_TIMx_PWM_HEAT_E2               (GPIO_AF3_TIM15)

///// Timer used for PWM_HEAT_E3
//#define BSP_MISC_TIMER_PWM_HEAT_E3                  (TIM15)
///// Channel Timer used for PWM_HEAT_E3
//#define BSP_MISC_CHAN_TIMER_PWM_HEAT_E3                  (TIM_CHANNEL_1)
///// HAL Active Channel Timer used for PWM_HEAT_E3
//#define BSP_MISC_HAL_ACT_CHAN_TIMER_PWM_HEAT_E3     (HAL_TIM_ACTIVE_CHANNEL_1)
///// Timer Clock Enable for PWM_HEAT_E3
//#define __BSP_MISC_TIMER_PWM_HEAT_E3_CLCK_ENABLE()  __TIM15_CLK_ENABLE()
///// Timer Clock Disable for PWM_HEAT_E3
//#define __BSP_MISC_TIMER_PWM_HEAT_E3_CLCK_DISABLE() __TIM15_CLK_DISABLE()
///// PWM_HEAT_E3 GPIO alternate function
//#define BSP_MISC_AFx_TIMx_PWM_HEAT_E3               (GPIO_AF3_TIM15)

//TODO: remove servo support
/// Timer used for Servo (marlin only)
//#define BSP_MISC_TIMER_SERVO                      (TIM1)
///// Channel Timer used for PWM_HEAT_E4
//#define BSP_MISC_CHAN_TIMER_SERVO                 (TIM_CHANNEL_1)
///// HAL Active Channel Timer used for PWM_HEAT_E4
//#define BSP_MISC_HAL_ACT_CHAN_TIMER_SERVO         (HAL_TIM_ACTIVE_CHANNEL_1)
///// Timer Clock Enable for PWM_HEAT_E4
//#define __BSP_MISC_TIMER_SERVO_CLCK_ENABLE()       __TIM1_CLK_ENABLE()
///// Timer Clock Disable for PWM_HEAT_E4
//#define __BSP_MISC_TIMER_SERVO_CLCK_DISABLE()     __TIM1_CLK_DISABLE()
///// PWM_HEAT_E4 GPIO alternate function
//#define BSP_MISC_AFx_TIMx_SERVO                    (GPIO_AF2_TIM1)
//   /// SERVO global interrupt
//#define BSP_MISC_SERVO_IRQn                    (TIM1_BRK_UP_TRG_COM_IRQn)
  
//TODO: remove user SPI
/// User SPI (in example for LCD)
//#define SPI_USER                             (SPI2)
///// SPI clock enable
//#define SPI_USER_CLK_ENABLE()                __SPI2_CLK_ENABLE()
///// SPI SCK enable
//#define SPI_USER_SCK_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
///// SPI MISO enable
//#define SPI_USER_MISO_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
///// SPI MOSI enable
//#define SPI_USER_MOSI_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
///// SPI Force reset
//#define SPI_USER_FORCE_RESET()               __SPI2_FORCE_RESET()
///// SPI Release reset
//#define SPI_USER_RELEASE_RESET()             __SPI2_RELEASE_RESET()
///// SPI SCK pin
//#define SPI_USER_NSS_PIN                     (GPIO_PIN_12)
///// SPI SCK port
//#define SPI_USER_NSS_GPIO_PORT               (GPIOB)
///// SPI SCK pin
//#define SPI_USER_SCK_PIN                     (GPIO_PIN_13)
///// SPI SCK port
//#define SPI_USER_SCK_GPIO_PORT               (GPIOB)
///// SPI MISO pin
//#define SPI_USER_MISO_PIN                    (GPIO_PIN_14)
///// SPI MISO port
//#define SPI_USER_MISO_GPIO_PORT              (GPIOB)
///// SPI MOSI pin
//#define SPI_USER_MOSI_PIN                    (GPIO_PIN_15)
///// SPI MOSI port
//#define SPI_USER_MOSI_GPIO_PORT              (GPIOB)
///// SPI_SCK alternate function
//#define SPI_USER_SCK_AF                      (GPIO_AF5_SPI2)
///// SPI MISO AF
//#define SPI_USER_MISO_AF                     (SPI_USER_SCK_AF)
///// SPI MOSI AF
//#define SPI_USER_MOSI_AF                     (SPI_USER_SCK_AF)
   
//TODO: remove user GPIO's
/// User GPIOs
//#define BSP_USER_1_PIN                       (GPIO_PIN_0)
//#define BSP_USER_1_PORT                      (GPIOA)
//#define BSP_USER_2_PIN                       (GPIO_PIN_0)
//#define BSP_USER_2_PORT                      (GPIOA)
//#define BSP_USER_3_PIN                       (GPIO_PIN_0)
//#define BSP_USER_3_PORT                      (GPIOA)
//#define BSP_USER_4_PIN                       (GPIO_PIN_0)
//#define BSP_USER_4_PORT                      (GPIOA)

//TODO: remove I2C
/// User I2C (in example for LCD)
//#define BSP_MISC_I2C                         (I2C1)
//#define __BSP_MISC_I2C_CLK_ENABLE()          __I2C1_CLK_ENABLE()
//#define __BSP_MISC_I2C_CLK_DISABLE()         __I2C1_CLK_DISABLE()
//#define __BSP_MISC_I2C_FORCE_RESET()         __I2C1_FORCE_RESET()
//#define __BSP_MISC_I2C_RELEASE_RESET()       __I2C1_RELEASE_RESET()
//#define __BSP_MISC_I2C_SCL_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
//#define __BSP_MISC_I2C_SDA_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
//#define BSP_MISC_I2C_SCL_PIN                 (GPIO_PIN_6)
//#define BSP_MISC_I2C_SCL_PORT                (GPIOB)
//#define BSP_MISC_I2C_SDA_PIN                 (GPIO_PIN_7)
//#define BSP_MISC_I2C_SDA_PORT                (GPIOB)
//#define BSP_MISC_AFx_I2C                     (GPIO_AF1_I2C1)
//#define BSP_MISC_I2C_CLOCK_SPEED             (100000)
   
//TODO: double check this value to make sure that it matches the pin array
//This effectively does the arduino pin mapping
#define BSP_MISC_MAX_PIN_NUMBER (53)

//TODO: review this value
//Tick timer prescaler
#define TICK_TIMER_PRESCALER  (32)
   
/* Exported constants --------------------------------------------------------*/
extern GPIO_TypeDef* gArrayGpioPort[BSP_MISC_MAX_PIN_NUMBER];
extern uint16_t gArrayGpioPin[BSP_MISC_MAX_PIN_NUMBER];
/* Extern function -----------------------------------------------------------*/

void BSP_MiscOverallInit(uint8_t nbDevices);
void BSP_MiscErrorHandler(uint16_t error);   
void BSP_MiscInitZProbeInit(void);
void BSP_MiscInitZProbeEnable(uint8_t enable);

void BSP_MiscStopInit(uint8_t id);
uint8_t BSP_MiscStopGetStatus(uint8_t id);
void BSP_MiscFanInit(uint8_t id);
void BSP_MiscFanSetSpeed(uint8_t id,uint8_t speed);
void BSP_MiscTickInit(void);
void BSP_MiscTickSetFreq(uint32_t newFreq);
void BSP_MiscTickSetPeriod(uint32_t newTimPeriod);
void BSP_MiscTickStop(void);
void BSP_MiscTick2Init(void);
void BSP_MiscTick2SetFreq(float newPeriod);
void BSP_MiscTick2Stop(void);
void BSP_MiscSetStepClockToSwMode(void);
void BSP_MiscGenerateStepClockPulse(uint8_t deviceId);
void BSP_MiscSetOcdThreshold(uint8_t deviceId, uint32_t current);
void BSP_MiscHeatManualInit(uint8_t heatId);
void BSP_MiscHeatPwmInit(uint8_t heatId);
void BSP_MiscHeatPwmSetDutyCycle(uint8_t heatId, uint8_t newDuty);
void  BSP_MotorControlBoard_ServoInit(void);
void  BSP_MotorControlBoard_ServoSetTimerValue(uint32_t value);
void  BSP_MotorControlBoard_ServoStop(void);
void BSP_MiscUserGpioInit(uint8_t id, uint32_t mode, uint32_t pull);

#ifdef __cplusplus
}
#endif

#endif /* __MPMD_3DPRINTER_MISC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

