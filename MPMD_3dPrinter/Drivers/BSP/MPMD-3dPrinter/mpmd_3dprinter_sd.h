/** 
  ******************************************************************************
  * @file    mpmd_3dPrinter_sd.h
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    March 02, 2015
  * @brief   Header for SD functions of 3D Printer BSP driver 
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
//TODO: revisit this later, we use soft SD card, so this probably can go
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPMD_3DPRINTER_SD_H
#define __MPMD_3DPRINTER_SD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
   
/* Exported macros ------------------------------------------------------------*/

#define MSD_OK         0x00
#define MSD_ERROR      0x01

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

#define SD_DATATIMEOUT           ((uint32_t)100000000)
    
/* Definition for SD resources *********************************************/
//TOOD: remove card detect pin since we use standard spi mode
#define BSP_SD_DETECT_PIN    (GPIO_PIN_15)
#define BSP_SD_DETECT_PORT   (GPIOA)

/// Interrupt line used for SD card detection
#define BSP_SD_DETECT_IRQn           (EXTI0_IRQn)

/// Priority used for SD card detection.
/// Warning, this priority shall be less than SysTick priority (higuest value)
#define BSP_SD_DETECT_PRIORITY	(0x07)
   
/* DMA definitions for SD DMA transfer */
#define __BSP_BSP_SD_DMAx_TxRx_CLK_ENABLE            __DMA2_CLK_ENABLE
#define BSP_SD_DMAx_Tx_CHANNEL                DMA_CHANNEL_3
#define BSP_SD_DMAx_Rx_CHANNEL                DMA_CHANNEL_2
 //TODO: no concept of DMA streams, we can remove
//#define BSP_SD_DMAx_Tx_STREAM                 DMA2_Stream6
//#define BSP_SD_DMAx_Rx_STREAM                 DMA2_Stream3
#define BSP_SD_DMAx_Tx_IRQn                   DMA1_Channel2_3_IRQn
#define BSP_SD_DMAx_Rx_IRQn                   DMA1_Channel2_3_IRQn
#define BSP_SD_DMAx_Tx_IRQHandler             DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler
#define BSP_SD_DMAx_Rx_IRQHandler             DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler
   
/* Exported types --- --------------------------------------------------------*/
//TODO: remove this since there's no hardware SD card
//#define SD_CardInfo HAL_SD_CardInfoTypedef
   
//typedef struct BspSdDataTag
//{
//  SD_HandleTypeDef uSdHandle;
//  SD_CardInfo uSdCardInfo;
//}BspSdDataType;

/* Exported variables  --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_DeInit(void);
void 	BSP_SD_DetectInit(void);
void    BSP_SD_DetectIT(void);
void    BSP_SD_DetectCallback(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_Erase(uint64_t StartAddr, uint64_t EndAddr);
void    BSP_SD_IRQHandler(void);
void    BSP_SD_DMA_Tx_IRQHandler(void);
void    BSP_SD_DMA_Rx_IRQHandler(void);
//HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void);
//void    BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef *CardInfo);
uint8_t BSP_SD_IsDetected(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPMD_3DPRINTER_SD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
