/** 
  ******************************************************************************
  * @file    mpmd_3dPrinter_cdc.h
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
#ifndef __MPMD_3DPRINTER_CDC_H
#define __MPMD_3DPRINTER_CDC_H

#ifdef __cplusplus
 extern "C" {
#endif

   /* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "Configuration_STM.h"
/* Exported macros ------------------------------------------------------------*/
//TODO: we should probably adjust these to a smaller size
#ifdef STM32_USE_USB_CDC
#define CDC_TX_BUFFER_SIZE (128)
#define CDC_RX_BUFFER_SIZE (512*4)
#else
#define CDC_TX_BUFFER_SIZE (1)
#define CDC_RX_BUFFER_SIZE (1)
#endif

/* Exported types --- --------------------------------------------------------*/

/* Exported variables  --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void BSP_CdcHwInit(uint32_t newBaudRate);
void BSP_CdcHwDeInit(void);
void BSP_CdcIfStart(void);
void BSP_CdcIfStop(void);
void BSP_CdcIfQueueTxData(uint8_t *pBuf, uint8_t nbData);
void BSP_CdcIfSendQueuedData();
uint32_t BSP_CdcPrintf(const char* format,...);
uint32_t BSP_CdcGetNbRxAvailableBytes(uint8_t waitForNewLine);
int8_t BSP_CdcGetNextRxByte(void);
uint32_t BSP_CdcCopyNextRxBytes(uint8_t *buff, uint32_t maxlen);
uint8_t BSP_CdcIsTxOnGoing(void);
void BSP_CdcLockingTx(uint8_t *pBuf, uint8_t nbData);
#ifdef __cplusplus
}
#endif

#endif /* __MPMD_3DPRINTER_CDC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
