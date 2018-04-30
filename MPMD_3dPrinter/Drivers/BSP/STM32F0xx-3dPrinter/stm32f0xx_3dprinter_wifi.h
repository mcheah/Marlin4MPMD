/** 
  ******************************************************************************
  * @file    stm32f0xx_3dPrinter_wifi.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   Header for wifi functions of 3D Printer BSP driver 
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
#ifndef __stm32f0XX_3DPRINTER_WIFI_H
#define __stm32f0XX_3DPRINTER_WIFI_H
//TODO:remove wifi support since we don't have the stm wifi module
#ifdef __cplusplus
 extern "C" {
#endif

   /* Includes ---------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
   
/* Exported macros -----------------------------------------------------------*/

/* Definition for Wifi resources *********************************************/

/// GPIO Pin used for the WIFI boot0 pin 
#define BSP_WIFI_BOOT_PIN               (GPIO_PIN_12)
/// GPIO Port used for the WIFI boot0 pin 
#define BSP_WIFI_BOOT_PORT              (GPIOE)

/// GPIO Pin used for the WIFI Reset pin 
#define BSP_WIFI_RESET_PIN              (GPIO_PIN_11)
/// GPIO Port used for the WIFI Reset pin 
#define BSP_WIFI_RESET_PORT             (GPIOE)   

/// GPIO Pin used for the WIFI wakeup pin 
#define BSP_WIFI_WAKEUP_PIN             (GPIO_PIN_5)
/// GPIO Port used for the WIFI wakeup pin    
#define BSP_WIFI_WAKEUP_PORT            (GPIOB)

/* Definition for the UART used by the WIFI module */
#define BSP_WIFI_UART_BYTES_NONE                     (0x0)   
#define BSP_WIFI_UART_BYTES_TO_WIFI                  (0x1) 
#define BSP_WIFI_THRES_TO_GCODE_PARSER               (0x2)
#define BSP_WIFI_UART_BYTES_TO_GCODE_PARSER          (0x4)
#define BSP_WIFI_UART_BYTES_TO_WIFI_AND_GCODE_PARSER (BSP_WIFI_UART_BYTES_TO_WIFI |\
                                                      BSP_WIFI_UART_BYTES_TO_GCODE_PARSER)
#define BSP_WIFI_FILE_CREATION_OK            (0)
#define BSP_WIFI_FILE_SIZE_TOO_BIG           (1)
#define BSP_WIFI_FILE_NAME_TOO_LONG          (2)

#define BSP_WIFI_SOURCE_IS_PLATFORM          (0)
#define BSP_WIFI_SOURCE_IS_DEBUG_UART        (1)
   
#define BSP_WIFI_UART                           (USART2)
#define __BSP_WIFI_UART_CLK_ENABLE()              __USART2_CLK_ENABLE()
#define __BSP_WIFI_UART_CLK_DISABLE()             __USART2_CLK_DISABLE()

#define __BSP_WIFI_UART_CTS_GPIO_CLK_ENABLE()      __GPIOD_CLK_ENABLE()
#define __BSP_WIFI_UART_RTS_GPIO_CLK_ENABLE()      __GPIOD_CLK_ENABLE()   
#define __BSP_WIFI_UART_RX_GPIO_CLK_ENABLE()      __GPIOD_CLK_ENABLE()
#define __BSP_WIFI_UART_TX_GPIO_CLK_ENABLE()      __GPIOD_CLK_ENABLE()

#define __BSP_WIFI_UART_FORCE_RESET()             __USART2_FORCE_RESET()
#define __BSP_WIFI_UART_RELEASE_RESET()           __USART2_RELEASE_RESET()

/// GPIO Pin used for the UART2 CTS pin 
#define BSP_WIFI_UART_CTS_PIN               (GPIO_PIN_3)
/// GPIO Port used for the UART2 CTS pin 
#define BSP_WIFI_UART_CTS_PORT              (GPIOD)

/// GPIO Pin used for the UART2 RTS pin 
#define BSP_WIFI_UART_RTS_PIN               (GPIO_PIN_4)
/// GPIO Port used for the UART2 RTS pin 
#define BSP_WIFI_UART_RTS_PORT              (GPIOD)

/// GPIO Pin used for the UART2 TX pin 
#define BSP_WIFI_UART_TX_PIN               (GPIO_PIN_5)
/// GPIO Port used for the UART2 TX pin 
#define BSP_WIFI_UART_TX_PORT              (GPIOD)
   
/// GPIO Pintused for the UART2 Rx pin 
#define BSP_WIFI_UART_RX_PIN               (GPIO_PIN_6)
/// GPIO Port used for the UART2 Rx pin 
#define BSP_WIFI_UART_RX_PORT              (GPIOD)

/* Definition for BSP_UART_DEBUG's NVIC */
#define BSP_WIFI_UART_IRQn                      (USART2_IRQn)
#define BSP_WIFI_UART_IRQHandler                (USART2_IRQHandler)   

#define BSP_WIFI_UART_CTS_AF                     (GPIO_AF7_USART2)
#define BSP_WIFI_UART_RTS_AF                     (GPIO_AF7_USART2)
#define BSP_WIFI_UART_TX_AF                     (GPIO_AF7_USART2)
#define BSP_WIFI_UART_RX_AF                     (GPIO_AF7_USART2)
   
/* Definition for DMA resources used by UART for Wifi module ******************/

#define __BSP_WIFI_UART_DMA_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
   
#define BSP_WIFI_UART_DMA_RX           (DMA1_Stream5)
#define BSP_WIFI_UART_DMA_RX_CHANNEL   (DMA_CHANNEL_4)

#define BSP_WIFI_UART_DMA_TX           (DMA1_Stream6)
#define BSP_WIFI_UART_DMA_TX_CHANNEL   (DMA_CHANNEL_4)

#define BSP_WIFI_UART_DMA_TX_IRQn               (DMA1_Stream6_IRQn)
#define BSP_WIFI_UART_DMA_TX_IRQHandler         (DMA1_Stream6_IRQHandler)

#define BSP_WIFI_UART_DMA_RX_IRQn               (DMA1_Stream5_IRQn)
#define BSP_WIFI_UART_DMA_RX_IRQHandler         (DMA1_Stream5_IRQHandler)
   
/* Exported types --- --------------------------------------------------------*/
typedef struct BspWifiDataTag
{
  UART_HandleTypeDef uartHandle;
  DMA_HandleTypeDef rxDmaHandle;
  DMA_HandleTypeDef txDmaHandle;
  volatile uint8_t txFlag;
  uint8_t *pRxBuffer;
  uint8_t *pRxWriteBuffer;
  uint8_t *pRxReadBuffer;
  uint8_t *pTxBuffer;
  uint8_t *pTxWriteBuffer;
  uint8_t mode;
  uint8_t state;
  uint8_t restartPending;
  volatile uint8_t commandPending;
  int8_t configPending;
  uint8_t fileCreationPending;
  uint32_t lastTime;
  uint32_t replyCounter;
#if defined(PROD_TEST)
  uint8_t fwUpdatePending;
  int8_t fsUpdatePending;
  char currentFwVersion[14+1];
  char expectedFwVersion[14+1];
  char expectedFsVersion[21+1];
#endif //#if defined(PROD_TEST)
  char wepKey[13+1];
  char ssid[32+1];
}BspWifiDataType;

/* Exported variables  --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
#if defined(PROD_TEST)
void BSP_WifiHwInit(uint32_t baudRate, char* ssid, char* wepKey, char* expectedFwVersion, char* expectedFsVersion);
#else
void BSP_WifiHwInit(uint32_t baudRate, char* ssid, char* wepKey);
#endif
uint8_t BSP_WifiUartRxCpltCallback(UART_HandleTypeDef *UartHandle,\
  unsigned char **c);
void BSP_WifiUartTxCpltCallback(UART_HandleTypeDef *UartHandle);
void BSP_WifiProcessUartBytes(void);
uint8_t BSP_WifiParseTxBytes(const char* pBuffer, uint16_t nbTxBytes,\
  uint8_t source);
uint8_t BSP_WifiCreateFileInWifiModuleRam(const char* fileName,\
  uint16_t fileSizeWithoutHttpHeader);
uint8_t BSP_WifiIsFileCreation(void);

#ifdef __cplusplus
}
#endif

#endif /* __stm32f0XX_3DPRINTER_WIFI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
