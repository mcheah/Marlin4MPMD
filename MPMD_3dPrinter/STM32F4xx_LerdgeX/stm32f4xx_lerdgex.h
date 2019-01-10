/** 
  ******************************************************************************
  * @file    stm32f4xx_lerdgex.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32F4XX-Mpmd Kit
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
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
#ifndef __STM32F4XX_LERDGEX_H
#define __STM32F4XX_LERDGEX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "configuration_STM.h"
#include "stm32f4xx_hal.h"
   
/* To be defined only if the board is provided with the related shield */
/* https://www.adafruit.com/products/802 */
//#ifndef ADAFRUIT_TFT_JOY_SD_ID802
//#define ADAFRUIT_TFT_JOY_SD_ID802
//#endif
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F4XX_LERDGEX
  * @{
  */

/** @addtogroup STM32F4XX_LERDGEX_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_Exported_Types STM32F4XX NUCLEO 144 LOW LEVEL Exported Types
  * @{
  */

//#define SOFTWARE_SPI
typedef enum 
{
  LED1 = 0,
  LED_RED = LED1,
  LED2 = 1,
  LED_GREEN = LED2,
  LED3 = 2,
  LED_BLUE = LED3
}Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0
//  /* Alias */
//  BUTTON_KEY = BUTTON_USER
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

//typedef enum
//{
//  JOY_NONE  = 0,
//  JOY_SEL   = 1,
//  JOY_DOWN  = 2,
//  JOY_LEFT  = 3,
//  JOY_RIGHT = 4,
//  JOY_UP    = 5
//}JOYState_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_Exported_Constants STM32F4XX NUCLEO 144 LOW LEVEL Exported Constants
  * @{
  */ 

/** 
  * @brief Define for STM32F4XX_LERDGEX board
  */ 
#if !defined (USE_STM32F4XX_LERDGEX)
 #define USE_STM32F4XX_LERDGEX
#endif

/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_LED STM32F4XX NUCLEO 144 LOW LEVEL LED
  * @{
  */
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_6
#define LED1_GPIO_PORT                          GPIOC
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOC_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_7
#define LED2_GPIO_PORT                          GPIOC
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOBC_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_8
#define LED3_GPIO_PORT                          GPIOA
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) != 2) {__HAL_RCC_GPIOC_CLK_ENABLE();} else\
                                                                    {__HAL_RCC_GPIOA_CLK_ENABLE();   }} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) != 2) {__HAL_RCC_GPIOC_CLK_DISABLE();} else\
                                                                    {__HAL_RCC_GPIOA_CLK_DISABLE();   }} while(0)
/**
  * @}
  */ 
  
/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_BUTTON STM32F4XX NUCLEO 144 LOW LEVEL BUTTON
  * @{
  */  
#define BUTTONn                                 0

/**
 * @brief Key push-button
 */
//#define USER_BUTTON_PIN                          GPIO_PIN_7
//#define USER_BUTTON_GPIO_PORT                    GPIOB
//#define USER_BUTTON_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
//#define USER_BUTTON_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//#define USER_BUTTON_EXTI_LINE                    GPIO_PIN_7
//#define USER_BUTTON_EXTI_IRQn                    EXTI4_15_IRQn
//
//#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)      USER_BUTTON_GPIO_CLK_ENABLE()
//#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)     USER_BUTTON_GPIO_CLK_DISABLE()
//
///* Aliases */
//#define KEY_BUTTON_PIN                       USER_BUTTON_PIN
//#define KEY_BUTTON_GPIO_PORT                 USER_BUTTON_GPIO_PORT
//#define KEY_BUTTON_GPIO_CLK_ENABLE()         USER_BUTTON_GPIO_CLK_ENABLE()
//#define KEY_BUTTON_GPIO_CLK_DISABLE()        USER_BUTTON_GPIO_CLK_DISABLE()
//#define KEY_BUTTON_EXTI_LINE                 USER_BUTTON_EXTI_LINE
//#define KEY_BUTTON_EXTI_IRQn                 USER_BUTTON_EXTI_IRQn


/**
  * @brief OTG_FS1 OVER_CURRENT and POWER_SWITCH Pins definition
  */

//
//#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_7
//#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
//#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()
//
//#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
//#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
//#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

/**
  * @}
  */ 

/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_BUS STM32F4XX NUCLEO 144 LOW LEVEL BUS
  * @{
  */
/*############################### SPI_A #######################################*/
#ifdef HAL_SPI_MODULE_ENABLED

#define NUCLEO_SPIx                                     SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                        __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                              GPIO_AF0_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                       GPIOB
#define NUCLEO_SPIx_SCK_PIN                             GPIO_PIN_3
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                        GPIO_AF0_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT                 GPIOB
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                            GPIO_PIN_4
#define NUCLEO_SPIx_MOSI_PIN                            GPIO_PIN_5
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000



/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_6
#define SD_CS_GPIO_PORT                           GPIOB
#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()
//TODO: revisit whether we can use some pin for card detect
///**
//  * @brief  SD Detect Interface pins
//  */
//#define SD_DETECT_PIN                   GPIO_PIN_15             /* PB.15 */
//#define SD_DETECT_GPIO_PORT             GPIOB                   /* GPIOB */
//#define SD_DETECT_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
//#define SD_DETECT_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
//#define SD_DETECT_EXTI_IRQn             EXTI4_15_IRQn

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
     
/**
  * @brief  LCD Control Interface pins (shield D10)
  */
#define LCD_CS_PIN                                 GPIO_PIN_14
#define LCD_CS_GPIO_PORT                           GPIOD
#define LCD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()
    
/**
  * @brief  LCD Data/Command Interface pins (shield D8)
  */
#define LCD_DC_PIN                                 GPIO_PIN_12
#define LCD_DC_GPIO_PORT                           GPIOF
#define LCD_DC_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOF_CLK_DISABLE()

#endif /* HAL_SPI_MODULE_ENABLED */
#ifdef HAL_SD_MODULE_ENABLED
#define SDIOx                                     SDIO
#define LERDGEX_SDIO_CLK_ENABLE()                        __HAL_RCC_SDIO_CLK_ENABLE()

#define LERDGEX_SDIO_D0_AF                              GPIO_AF12_SDIO
#define LERDGEX_SDIO_D0_GPIO_PORT                       GPIOC
#define LERDGEX_SDIO_D0_PIN                             GPIO_PIN_8
#define LERDGEX_SDIO_D0_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define LERDGEX_SDIO_D0_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()

#define LERDGEX_SDIO_D1_AF                              GPIO_AF12_SDIO
#define LERDGEX_SDIO_D1_GPIO_PORT                       GPIOC
#define LERDGEX_SDIO_D1_PIN                             GPIO_PIN_9
#define LERDGEX_SDIO_D1_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define LERDGEX_SDIO_D1_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()

#define LERDGEX_SDIO_D2_AF                              GPIO_AF12_SDIO
#define LERDGEX_SDIO_D2_GPIO_PORT                       GPIOC
#define LERDGEX_SDIO_D2_PIN                             GPIO_PIN_10
#define LERDGEX_SDIO_D2_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define LERDGEX_SDIO_D2_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()

#define LERDGEX_SDIO_D3_AF                              GPIO_AF12_SDIO
#define LERDGEX_SDIO_D3_GPIO_PORT                       GPIOC
#define LERDGEX_SDIO_D3_PIN                             GPIO_PIN_11
#define LERDGEX_SDIO_D3_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define LERDGEX_SDIO_D3_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()

#define LERDGEX_SDIO_CLK_AF                              GPIO_AF12_SDIO
#define LERDGEX_SDIO_CLK_GPIO_PORT                       GPIOC
#define LERDGEX_SDIO_CLK_PIN                             GPIO_PIN_12
#define LERDGEX_SDIO_CLK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define LERDGEX_SDIO_CLK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()

#define LERDGEX_SDIO_CMD_AF                        GPIO_AF12_SDIO
#define LERDGEX_SDIO_CMD_GPIO_PORT                 GPIOD
#define LERDGEX_SDIO_CMD_PIN                            GPIO_PIN_2
#define LERDGEX_SDIO_CMD_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE()
#define LERDGEX_SDIO_CMD_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOD_CLK_DISABLE()

/* DMA definitions for SD DMA transfer */
#define SD_DMAx_TxRx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE
#define SD_DMAx_Tx_CHANNEL                DMA_CHANNEL_4
#define SD_DMAx_Rx_CHANNEL                DMA_CHANNEL_4
#define SD_DMAx_Tx_STREAM                 DMA2_Stream6
#define SD_DMAx_Rx_STREAM                 DMA2_Stream3
#define SD_DMAx_Tx_IRQn                   DMA2_Stream6_IRQn
#define SD_DMAx_Rx_IRQn                   DMA2_Stream3_IRQn
#define BSP_SD_IRQHandler                 SDIO_IRQHandler
#define BSP_SD_DMA_Tx_IRQHandler          DMA2_Stream6_IRQHandler
#define BSP_SD_DMA_Rx_IRQHandler          DMA2_Stream3_IRQHandler
#define SD_DetectIRQHandler()             HAL_GPIO_EXTI_IRQHandler(SD_DETECT_PIN)

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000
#endif //HAL_SD_MODULE_ENABLED
#ifdef HAL_I2C_MODULE_ENABLED
/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for I2Cx clock resources */
#define I2C_FRAM_ADDR					 (0xA0)
#define I2Cx                             I2C1
#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SCL_AF                     GPIO_AF4_I2C1
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SDA_AF                     GPIO_AF4_I2C1
#endif //HAL_I2C_MODULE_ENABLED

#define LCD_D0_PIN                                 GPIO_PIN_14
#define LCD_D0_GPIO_PORT                           GPIOD
#define LCD_D0_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D0_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()
#define LCD_D1_PIN                                 GPIO_PIN_15
#define LCD_D1_GPIO_PORT                           GPIOD
#define LCD_D1_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D1_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()
#define LCD_D2_PIN                                 GPIO_PIN_0
#define LCD_D2_GPIO_PORT                           GPIOD
#define LCD_D2_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D2_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()
#define LCD_D3_PIN                                 GPIO_PIN_1
#define LCD_D3_GPIO_PORT                           GPIOD
#define LCD_D3_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D3_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()
#define LCD_D4_PIN                                 GPIO_PIN_7
#define LCD_D4_GPIO_PORT                           GPIOE
#define LCD_D4_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D4_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D5_PIN                                 GPIO_PIN_8
#define LCD_D5_GPIO_PORT                           GPIOE
#define LCD_D5_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D5_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D6_PIN                                 GPIO_PIN_9
#define LCD_D6_GPIO_PORT                           GPIOE
#define LCD_D6_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D6_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D7_PIN                                 GPIO_PIN_10
#define LCD_D7_GPIO_PORT                           GPIOE
#define LCD_D7_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D7_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D8_PIN                                 GPIO_PIN_11
#define LCD_D8_GPIO_PORT                           GPIOE
#define LCD_D8_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D8_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D9_PIN                                 GPIO_PIN_12
#define LCD_D9_GPIO_PORT                           GPIOE
#define LCD_D9_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D9_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D10_PIN                                GPIO_PIN_13
#define LCD_D10_GPIO_PORT                          GPIOE
#define LCD_D10_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D10_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D11_PIN                                GPIO_PIN_14
#define LCD_D11_GPIO_PORT                          GPIOE
#define LCD_D11_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D11_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D12_PIN                                GPIO_PIN_15
#define LCD_D12_GPIO_PORT                          GPIOE
#define LCD_D12_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_D12_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOE_CLK_DISABLE()
#define LCD_D13_PIN                                GPIO_PIN_8
#define LCD_D13_GPIO_PORT                          GPIOD
#define LCD_D13_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D13_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()
#define LCD_D14_PIN                                GPIO_PIN_9
#define LCD_D14_GPIO_PORT                          GPIOD
#define LCD_D14_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D14_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()
#define LCD_D15_PIN                                GPIO_PIN_10
#define LCD_D15_GPIO_PORT                          GPIOD
#define LCD_D15_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_D15_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_NOE_PIN                                GPIO_PIN_4
#define LCD_NOE_GPIO_PORT                          GPIOD
#define LCD_NOE_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_NOE_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_NWE_PIN                                GPIO_PIN_5
#define LCD_NWE_GPIO_PORT                          GPIOD
#define LCD_NWE_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_NWE_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_NE1_PIN                                GPIO_PIN_7
#define LCD_NE1_GPIO_PORT                          GPIOD
#define LCD_NE1_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_NE1_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_RS_PIN                                 GPIO_PIN_11
#define LCD_RS_GPIO_PORT                           GPIOD
#define LCD_RS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_RS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_FSMC_AF								   GPIO_AF12_FSMC
#define LCD_FSMC_D_PINS							 (    LCD_D0_PIN | LCD_D1_PIN | LCD_D2_PIN | LCD_D3_PIN | LCD_D13_PIN | LCD_D14_PIN  \
													| LCD_D15_PIN | LCD_NOE_PIN | LCD_NWE_PIN | LCD_NE1_PIN | LCD_RS_PIN )
#define LCD_FSMC_E_PINS							 (    LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN | LCD_D7_PIN | LCD_D8_PIN | LCD_D9_PIN  \
													| LCD_D10_PIN | LCD_D11_PIN | LCD_D12_PIN )

#define LCD_TOUCHEN_PIN                            GPIO_PIN_6
#define LCD_TOUCHEN_GPIO_PORT                      GPIOB
#define LCD_TOUCHEN_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_TOUCHEN_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

#define LCD_BACKLIGHT_PIN                          GPIO_PIN_3
#define LCD_BACKLIGHT_GPIO_PORT                    GPIOD
#define LCD_BACKLIGHT_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_BACKLIGHT_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_RST_PIN                                GPIO_PIN_6
#define LCD_RST_GPIO_PORT                          GPIOD
#define LCD_RST_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_RST_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_BUZZ_PIN                               GPIO_PIN_12
#define LCD_BUZZ_GPIO_PORT                         GPIOD
#define LCD_BUZZ_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_BUZZ_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOD_CLK_DISABLE()

#define LCD_BTN_PIN                                GPIO_PIN_2
#define LCD_BTN_GPIO_PORT                          GPIOE
#define LCD_BTN_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_BTN_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOE_CLK_DISABLE()

#define LCD_ENC1_PIN                               GPIO_PIN_3
#define LCD_ENC1_GPIO_PORT                         GPIOE
#define LCD_ENC1_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_ENC1_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOE_CLK_DISABLE()

#define LCD_ENC2_PIN                               GPIO_PIN_4
#define LCD_ENC2_GPIO_PORT                         GPIOE
#define LCD_ENC2_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_ENC2_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOE_CLK_DISABLE()

/*################################ ADCx for Nucleo 144 board ######################################*/
/**
  * @brief  ADCx Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
  
///* For some Nucleo144 boards, Arduino UNO pin7 (A3) is connected to PF3 in others to PC01 */
//#if defined(ADC3)
//#define NUCLEO_ADCx                          ADC3
//#define NUCLEO_ADCx_CLK_ENABLE()             __HAL_RCC_ADC3_CLK_ENABLE()
//#define NUCLEO_ADCx_CLK_DISABLE()            __HAL_RCC_ADC3_CLK_DISABLE()
//
//#define NUCLEO_ADCx_CHANNEL                  ADC_CHANNEL_9
//#define NUCLEO_ADCx_GPIO_PORT                GPIOF
//#define NUCLEO_ADCx_GPIO_PIN                 GPIO_PIN_3
//#define NUCLEO_ADCx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
//#define NUCLEO_ADCx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOF_CLK_DISABLE()
//
//#else
//#define NUCLEO_ADCx                          ADC1
//#define NUCLEO_ADCx_CLK_ENABLE()             __HAL_RCC_ADC1_CLK_ENABLE()
//#define NUCLEO_ADCx_CLK_DISABLE()            __HAL_RCC_ADC1_CLK_DISABLE()
//#define NUCLEO_ADCx_CHANNEL                  ADC_CHANNEL_11
//
//#define NUCLEO_ADCx_GPIO_PORT                GPIOC
//#define NUCLEO_ADCx_GPIO_PIN                 GPIO_PIN_1
//#define NUCLEO_ADCx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
//#define NUCLEO_ADCx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()
//#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_Exported_Macros STM32F4XX NUCLEO 144 LOW LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup STM32F4XX_LERDGEX_LOW_LEVEL_Exported_Functions STM32F4XX NUCLEO 144 LOW LEVEL Exported Functions
  * @{
  */

extern I2C_HandleTypeDef I2cHandle;
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);
void			 BSP_LCD_Init();
void 			 HAL_I2Cx_init(void);
//#ifdef HAL_ADC_MODULE_ENABLED
//uint8_t          BSP_JOY_Init(void);
//JOYState_TypeDef BSP_JOY_GetState(void);
//void             BSP_JOY_DeInit(void);
//#endif /* HAL_ADC_MODULE_ENABLED */
static inline void delay_basic(float Sec)
{
	  uint32_t steps_per_sec = 48e6/450;
	  for(uint32_t i=0; i<Sec*steps_per_sec;i++)
	  {

	  }
}
  
/**
  * @}
  */ 

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

#endif /* __STM32F4XX_LERDGEX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
