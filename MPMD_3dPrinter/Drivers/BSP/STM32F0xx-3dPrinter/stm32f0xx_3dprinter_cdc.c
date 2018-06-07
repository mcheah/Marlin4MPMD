/**
  ******************************************************************************
  * @file    stm32f0xx_3dPrinter_cdc.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   uart functions of 3D Printer BSP driver 
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
#include "stm32f0xx_3dprinter_cdc.h"
#include "stm32f0xx_3dprinter_misc.h"
// #include "stm32f0xx_3dprinter_wifi.h"
#include <string.h> /* for memcpy */
#include <stdarg.h> /* for va_start */
#include <stdio.h> /* for vsprintf */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
/* Private defines -----------------------------------------------------------*/
/* Private constant ----------------------------------------------------------*/
#define CDC_ERROR_TAG        (0x5000)
#define CDC_ERROR(error)     BSP_MiscErrorHandler(error|CDC_ERROR_TAG)

#ifdef USE_XONXOFF
#define BSP_CDC_GET_NB_BYTES_IN_RX_BUFFER()  ((gBspUartData.pRxReadBuffer <= gBspUartData.pRxWriteBuffer)? \
                                    ( (unsigned int )(gBspUartData.pRxWriteBuffer - gBspUartData.pRxReadBuffer)): \
                                    ( (unsigned int )(gBspUartData.pRxWriteBuffer + UART_RX_BUFFER_SIZE - gBspUartData.pRxReadBuffer)))


#define BSP_CDC_GET_NB_BYTES_IN_TX_BUFFER()  ((gBspUartData.pTxReadBuffer <= gBspUartData.pTxWriteBuffer)? \
                                    ( (unsigned int )(gBspUartData.pTxWriteBuffer - gBspUartData.pTxReadBuffer)): \
                                    ( (unsigned int )(gBspUartData.pTxWriteBuffer + UART_TX_BUFFER_SIZE - gBspUartData.pTxReadBuffer)))


#define BSP_CDC_TX_THRESHOLD_XOFF  (UART_TX_BUFFER_SIZE / 50)
#define BSP_CDC_TX_THRESHOLD_XON   (UART_TX_BUFFER_SIZE / 100)
#define BSP_CDC_RX_THRESHOLD_XOFF  (UART_RX_BUFFER_SIZE / 2)
#define BSP_CDC_RX_THRESHOLD_XON   (UART_RX_BUFFER_SIZE / 3)
#endif

/* Private functions ---------------------------------------------------------*/
uint8_t BSP_CdcParseRxAvalaibleBytes(const char* pBuffer, uint8_t nbRxBytes);

/* Global variables ----------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
//BspUartDataType gBspUartData;
uint8_t gBspCdcTxBuffer[2 * CDC_TX_BUFFER_SIZE]; // real size is double to easily handle memcpy and tx uart
uint8_t gBspCdcRxBuffer[2 * CDC_RX_BUFFER_SIZE];

volatile uint8_t rxWriteChar;
uint8_t *pRxBuffer = gBspCdcRxBuffer;
volatile uint8_t *pRxWriteBuffer;
volatile uint8_t *pRxReadBuffer;
uint8_t *pTxBuffer = gBspCdcTxBuffer;
volatile uint8_t *pTxWriteBuffer;
volatile uint8_t *pTxReadBuffer;
volatile uint8_t *pTxWrap;
uint32_t debugNbRxFrames = 0;
uint32_t debugNbTxFrames = 0;
#ifdef USE_XONXOFF
static uint8_t BspUartXonXoff = 0;
static uint8_t BspUartXoffBuffer[12] = " SEND XOFF\n";
static uint8_t BspUartXonBuffer[11] = " SEND XON\n";
#endif
/* Extern function -----------------------------------------------------------*/

/******************************************************//**
 * @brief  Usart Hw initialisation
 * @param None
 * @retval None
 **********************************************************/
void BSP_CdcHwInit(uint32_t newBaudRate)
{
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, &USBD_CDC);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
//
//  //TODO: replace this with the updated function
//  BspUartDataType *pUart = &gBspUartData;
//
//  pUart->handle.Instance = BSP_CDC_DEBUG;
//  pUart->handle.Init.BaudRate = newBaudRate;
//  pUart->handle.Init.WordLength = UART_WORDLENGTH_8B;
//  pUart->handle.Init.StopBits = UART_STOPBITS_1;
//  pUart->handle.Init.Parity = UART_PARITY_NONE;
//  pUart->handle.Init.Mode = UART_MODE_TX_RX;
//  pUart->handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  pUart->handle.Init.OverSampling = UART_OVERSAMPLING_16;
//
//  if(HAL_UART_DeInit(&pUart->handle) != HAL_OK)
//  {
//    CDC_ERROR(1);
//  }
//
//  if( HAL_UART_Init(&pUart->handle) != HAL_OK)
//  {
//    CDC_ERROR(2);
//  }
}

/******************************************************//**
 * @brief  Start the UART interface with the GUI 
 * @param None
 * @retval None
 **********************************************************/

void BSP_CdcIfStart(void)
{
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  //Replace this with the correct function
//  BspUartDataType *pUart = &gBspUartData;
//  pUart->pRxBuffer = (uint8_t *)gBspCdcRxBuffer;
  pRxWriteBuffer =  pRxBuffer;
  pRxReadBuffer =  pRxBuffer;
//
//  pUart->pTxBuffer = (uint8_t *)gBspCdcTxBuffer;
//  pTxWriteBuffer =  pUart->pTxBuffer;
  pTxReadBuffer =  pTxBuffer;
  pTxWrap  =  pTxBuffer + CDC_TX_BUFFER_SIZE;
//
//  pUart->rxBusy = RESET;
//  pUart->txBusy = RESET;
  debugNbTxFrames = 0;
  debugNbRxFrames = 0;
//
//  pUart->newTxRequestInThePipe = 0;
//  pUart->nbBridgedBytes = 0;
//  pUart->gCodeDataMode = 0;
//
//  /* wait for 1 bytes on the RX uart */
//  if (HAL_UART_Receive_IT(&pUart->handle, (uint8_t *)(&pUart->rxWriteChar), 1) != HAL_OK)
//  {
//    CDC_ERROR(3);
//  }
//
//  pUart->rxBusy = SET;
//  for(uint32_t i=0;i<48000000*5;i++) {
//	  volatile uint32_t x = i;
//  }
  HAL_Delay(5000);
}

/******************************************************//**
 * @brief  Queue tx data to be sent on the UART
 * @param[in]  pBuf pointer to the data to be sent
 * @param[in]  nbData number of bytes to be sent
 * @retval None
 **********************************************************/
void BSP_CdcIfQueueTxData(uint8_t *pBuf, uint8_t nbData)
{
//  if (nbData != 0)
//  {
//    BspUartDataType *pUart= &gBspUartData;
//    int32_t nbFreeBytes = pUart->pTxReadBuffer - pUart->pTxWriteBuffer;
//
//    if (nbFreeBytes <= 0)
//    {
//      nbFreeBytes += UART_TX_BUFFER_SIZE;
//    }
//    if (nbData > nbFreeBytes)
//    {
//        /* Uart Tx buffer is full */
//    	char txt[80];
//    	sprintf(txt,"Fuckyou nbData=%d nbFree=%d,size=%d\r\n",nbData,nbFreeBytes,sizeof(gBspCdcTxBuffer));
//    	BSP_CdcLockingTx(txt,80);
//        CDC_ERROR(4);
//    }
//
//    //use of memcpy is safe as real buffer size is 2 * UART_TX_BUFFER_SIZE
//    memcpy((uint8_t *)pUart->pTxWriteBuffer, pBuf, nbData);
//    pUart->pTxWriteBuffer += nbData;
//#if defined(MARLIN)
//    if (pBuf[nbData-1] == '\n')
//    {
//      *pUart->pTxWriteBuffer = '\n';
//      pUart->pTxWriteBuffer--;
//      *pUart->pTxWriteBuffer = '\r';
//      pUart->pTxWriteBuffer += 2;
//      if (pUart->pTxWriteBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
//      {
//        pUart->pTxWrap = pUart->pTxWriteBuffer;
//        pUart->pTxWriteBuffer = pUart->pTxBuffer;
//      }
//      //BSP_CdcIfSendQueuedData();  // BDI
//    }
//#else
//    if (pUart->pTxWriteBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
//    {
//      pUart->pTxWrap = pUart->pTxWriteBuffer;
//      pUart->pTxWriteBuffer = pUart->pTxBuffer;
//    }
//#endif
//    BSP_CdcIfSendQueuedData();
////#endif
//  }
    if(nbData) {
    	uint32_t Len = nbData;
    	//Wait until txstate is done transmitting before queing
    	CDC_Itf_SetTxBuffer(pBuf, &Len);
//    	HAL_Delay(0);
    	BSP_CdcIfSendQueuedData(&Len);
    }
}
   
/******************************************************//**
 * @brief  Send queued data to the GUI
 * @param None
 * @retval None
 **********************************************************/
void BSP_CdcIfSendQueuedData(uint32_t Len)
{
//    BspUartDataType *pUart = &gBspUartData;
//
//#ifdef USE_XONXOFF
//    if ((pUart->newTxRequestInThePipe == 0)&&
//        (pUart->txBusy == RESET))
//    {
//      if ((BspUartXonXoff == 2)||
//      ((BSP_CDC_GET_NB_BYTES_IN_TX_BUFFER()  > BSP_CDC_TX_THRESHOLD_XOFF) && (BspUartXonXoff == 0)))
//      {
//        pUart->txBusy = SET;
//        pUart->nbTxBytesOnGoing = 0;
//        BspUartXoffBuffer[0] = 0x13;
//        if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXoffBuffer, sizeof(BspUartXoffBuffer))!= HAL_OK)
//        {
//          CDC_ERROR(10);
//        }
//        BspUartXonXoff = 3;
//        return;
//      }
//      else if ((BspUartXonXoff == 1)||
//        ((BSP_CDC_GET_NB_BYTES_IN_RX_BUFFER()  < BSP_CDC_RX_THRESHOLD_XON) && (BspUartXonXoff == 3)&& (BSP_CDC_GET_NB_BYTES_IN_TX_BUFFER() < BSP_CDC_TX_THRESHOLD_XON)))
//      {
//        pUart->txBusy = SET;
//        pUart->nbTxBytesOnGoing = 0;
//        BspUartXonBuffer[0] = 0x11;
//        if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXonBuffer, sizeof(BspUartXonBuffer))!= HAL_OK)
//        {
//          CDC_ERROR(11);
//        }
//        BspUartXonXoff = 0;
//        return;
//      }
//    }
//#endif
//    if ((pUart->newTxRequestInThePipe == 0)&&
//        (pUart->txBusy == RESET)&&
//        (pUart->pTxReadBuffer != pUart->pTxWriteBuffer))
//    {
//      int32_t nbTxBytes = pUart->pTxWriteBuffer - pUart->pTxReadBuffer;
//      pUart->newTxRequestInThePipe++;
//      if (nbTxBytes < 0)
//      {
//        nbTxBytes = pUart->pTxWrap - pUart->pTxReadBuffer;
//      }
//
//#if defined(MARLIN)
//      if (pUart->pTxReadBuffer[nbTxBytes-1]!='\n')
//      {
//        pUart->newTxRequestInThePipe--;
//        return;
//      }
//#endif
//      pUart->txBusy = SET;
//      pUart->nbTxBytesOnGoing = nbTxBytes;
//
//      //use of HAL_UART_Transmit_IT is safe as real buffer size is 2 * UART_TX_BUFFER_SIZE
//      if(HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *) pUart->pTxReadBuffer, nbTxBytes)!= HAL_OK)
//      {
//        CDC_ERROR(5);
//      }
//
//      pUart->debugNbTxFrames++;
//      pUart->newTxRequestInThePipe--;
//    }
	CDC_Itf_Transmit(&Len);
}

/******************************************************//**
 * @brief  Tx Transfer completed callback
 * @param[in] UartHandle UART handle. 
 * @retval None
 **********************************************************/
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//  BspUartDataType *pUart = &gBspUartData;
//
//  if (UartHandle == &(pUart->handle))
//  {
//    /* Set transmission flag: transfer complete*/
//    pUart->txBusy = RESET;
//
//#ifdef USE_XONXOFF
//    if ((BspUartXonXoff == 2)||
//        ((BSP_CDC_GET_NB_BYTES_IN_TX_BUFFER()  > BSP_CDC_TX_THRESHOLD_XOFF) && (BspUartXonXoff == 0)))
//    {
//      pUart->txBusy = SET;
//      BspUartXoffBuffer[0] = 0x13;
//      if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXoffBuffer, sizeof(BspUartXoffBuffer))!= HAL_OK)
//      {
//        CDC_ERROR(10);
//      }
//      BspUartXonXoff = 3;
//      return;
//    }
//    else if ((BspUartXonXoff == 1)||
//        ((BSP_CDC_GET_NB_BYTES_IN_RX_BUFFER()  < BSP_CDC_RX_THRESHOLD_XON) && (BspUartXonXoff == 3)&& (BSP_CDC_GET_NB_BYTES_IN_TX_BUFFER() < BSP_CDC_TX_THRESHOLD_XON)))
//    {
//      pUart->txBusy = SET;
//      BspUartXonBuffer[0] = 0x11;
//      if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXonBuffer, sizeof(BspUartXonBuffer))!= HAL_OK)
//      {
//        CDC_ERROR(11);
//      }
//      BspUartXonXoff = 0;
//      return;
//    }
//#endif
//    pUart->pTxReadBuffer += pUart->nbTxBytesOnGoing;
//
//    if (pUart->pTxReadBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
//    {
//      pUart->pTxReadBuffer  = pUart->pTxBuffer;
//    }
//
//    if (pUart->uartTxDoneCallback != 0)
//    {
//      pUart->uartTxDoneCallback();
//    }
//  }
//#if !defined(NO_WIFI)
//  else
//  {
//    BSP_WifiUartTxCpltCallback(UartHandle);
//  }
//#endif //#if !defined(NO_WIFI)
//}

/******************************************************//**
 * @brief  Rx Transfer completed callback
 * @param[in] UartHandle UART handle. 
 * @retval None
 **********************************************************/
void BSP_CDC_RxCpltCallback(uint8_t* Buf, uint32_t *Len)
{
//  BspUartDataType *pUart = &gBspUartData;
//#if !defined(NO_WIFI)
//  unsigned char *pChar = NULL;
//#endif //#if !defined(NO_WIFI)

//  if (UartHandle == &(pUart->handle))
//  {
for(uint32_t i=0;i<*Len;i++)
{
    *pRxWriteBuffer = Buf[i];
//    if (HAL_UART_Receive_IT(&pUart->handle, (uint8_t *)(&pUart->rxWriteChar), 1) != HAL_OK)
//    {
//      CDC_ERROR(6);
//    }

    pRxWriteBuffer++;

    if (pRxWriteBuffer >= (pRxBuffer + CDC_RX_BUFFER_SIZE))
    {
      pRxWriteBuffer = pRxBuffer;
    }

//#ifdef USE_XONXOFF
//    if ((BSP_CDC_GET_NB_BYTES_IN_RX_BUFFER()  > BSP_CDC_RX_THRESHOLD_XOFF) && (BspUartXonXoff == 0))
//    {
//      BspUartXonXoff = 2;
//    }
//    else if ((BSP_CDC_GET_NB_BYTES_IN_RX_BUFFER()  < BSP_CDC_RX_THRESHOLD_XON) && (BspUartXonXoff == 3)&& (BSP_CDC_GET_NB_BYTES_IN_TX_BUFFER() <BSP_CDC_TX_THRESHOLD_XON))
//    {
//      BspUartXonXoff = 1;
//    }
//#endif
    if (pRxWriteBuffer == pRxReadBuffer)
    {
      // Rx buffer is full
      CDC_ERROR(7);
    }
//    HAL_UART_Transmit(UartHandle,&(pUart->rxWriteChar),1,500);
//    if (uartRxDataCallback != 0)
//    {
//      uartRxDataCallback((uint8_t *)pRxReadBuffer,pRxWriteBuffer - pRxReadBuffer);
//    }
    debugNbRxFrames++;
//  }
//#if !defined(NO_WIFI)
//  else
//  {
//    uint32_t tmp = BSP_WifiUartRxCpltCallback(UartHandle, &pChar);
//    if (tmp != 0)
//    {
//      memcpy((uint8_t*)(pUart->pRxWriteBuffer), pChar, tmp);
//      pUart->pRxWriteBuffer += tmp;
//      *pUart->pRxWriteBuffer = '\0';
//      if (pUart->pRxWriteBuffer >= (pUart->pRxBuffer + UART_RX_BUFFER_SIZE))
//      {
//        pUart->pRxWriteBuffer = pUart->pRxBuffer;
//      }
//    }
//  }
//#endif //#if !defined(NO_WIFI)
}
}

/******************************************************//**
 * @brief  Uart Error callback
 * @param[in] UartHandle UART handle. 
 * @retval None
 **********************************************************/
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//	char txt[80];
////	sprintf(txt,"Fuckyou nbData=%d nbFree=%d,size=%d\r\n",nbData,nbFreeBytes,sizeof(gBspCdcTxBuffer));
//	sprintf(txt,"Fuckyou Code=%0X ISR=%0X\r\n",huart->ErrorCode,huart->Instance->ISR);
//	BSP_CdcLockingTx(txt,80);
//    CDC_ERROR(8);
//}

/******************************************************//**
 * @brief  Attaches a callback which will be called when
 * a complete rx uart buffer is ready
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
//void BSP_CdcAttachRxDataHandler(void (*callback)(uint8_t *, uint8_t))
//{
//  BspUartDataType *pUart = &gBspUartData;
//  pUart->uartRxDataCallback = (void (*)(uint8_t *, uint8_t))callback;
//}

/******************************************************//**
 * @brief  Attaches a callback which will be called when
 * a complete tx uart buffer is ready
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
//void BSP_CdcAttachTxDoneCallback(void (*callback)(void))
//{
//  BspUartDataType *pUart = &gBspUartData;
//  pUart->uartTxDoneCallback = (void (*)(void))callback;
//}


/******************************************************//**
 * @brief  This function trigs the transmission of a string over the UART 
 *             for printing
 * @param[in] format string with formatting
 * @param[in]  Optional arguments to fit with formatting
 * @retval Lengthj of the string to print (uint32_t)
 **********************************************************/
uint32_t BSP_CdcPrintf(const char* format,...)
{
//  BspUartDataType *pUart = &gBspUartData;
  va_list args;
  uint32_t size;
  uint32_t retSize = 0;
//  int32_t nbFreeBytes = pUart->pTxReadBuffer - pUart->pTxWriteBuffer;
  
//  if (nbFreeBytes <= 0)
//  {
//    nbFreeBytes += UART_TX_BUFFER_SIZE;
//  }
  
//  char *writeBufferp =(char *) pUart->pTxWriteBuffer;
  char *writeBufferp = gBspCdcTxBuffer;
  /* the string to transmit is copied in the temporary buffer in order to    */
  /* check its size.                                                         */
  va_start(args, format);
  size=vsprintf(writeBufferp, (const char*)format, args);
  va_end(args);
   
  retSize = size;   
  if (*(writeBufferp + size - 1) == '\n')
  {
    *(writeBufferp + size - 1) = '\r';
    *(writeBufferp + size) = '\n';
    size++;
  }
  if (size != 0)
  {
    if ( size > CDC_TX_BUFFER_SIZE )
    {
      CDC_ERROR(9);
    }
//    pUart->pTxWriteBuffer += size;
//    if (pUart->pTxWriteBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
//    {
//      pUart->pTxWrap = pUart->pTxWriteBuffer;
//      pUart->pTxWriteBuffer  = pUart->pTxBuffer;
//    }
    BSP_CdcIfQueueTxData(writeBufferp,&size);
//    BSP_CdcIfSendQueuedData(&size);
  }
  return(retSize);  
}

/******************************************************//**
 * @brief  This function returns the number of bytes received via the UART
 * @param[in] fnone
  * @retval nxRxBytes nb received bytes
 **********************************************************/
uint32_t BSP_CdcGetNbRxAvalaibleBytes(void)
{
//  BspUartDataType *pUart = &gBspUartData;
  uint8_t *writePtr = (uint8_t *)(pRxWriteBuffer - 1);
  
  if (writePtr < pRxBuffer)
  {
    writePtr += CDC_RX_BUFFER_SIZE;
  }  
  
  //waitline feed to have a complete line before processing bytes
  if ((*writePtr) != '\r' && (*writePtr) != '\n')
    return (0);
  
  int32_t nxRxBytes = pRxWriteBuffer - pRxReadBuffer;
  if (nxRxBytes < 0)
  {
    nxRxBytes += CDC_RX_BUFFER_SIZE;
  }
//#if !defined(MARLIN)
//  if (nxRxBytes != 0)
//  {
//    uint8_t result = BSP_CdcParseRxAvalaibleBytes((char const*)pUart->pRxReadBuffer, nxRxBytes);
//    if (result < BSP_WIFI_THRES_TO_GCODE_PARSER)
//    {
//      //The available bytes will not to go into the Gcode parser
//      pUart->pRxReadBuffer += nxRxBytes;
//      nxRxBytes = 0;
//      if (pUart->pRxReadBuffer >= (pUart->pRxBuffer + UART_RX_BUFFER_SIZE))
//      {
//        pUart->pRxReadBuffer = pUart->pRxBuffer;
//      }
//    }
//  }
//#endif
  
  return ((uint32_t) nxRxBytes );
}

/******************************************************//**
 * @brief  This function returns the number of bytes received via the UART
 * @param[in] fnone
  * @retval nxRxBytes nb received bytes
 **********************************************************/
//uint8_t BSP_CdcParseRxAvalaibleBytes(const char* pBuffer, uint8_t nbRxBytes)
//{
//  return (BSP_WifiParseTxBytes(pBuffer, nbRxBytes, BSP_WIFI_SOURCE_IS_DEBUG_UART));
//}

/******************************************************//**
 * @brief  This function returns the first byte available on the UART
 * @param[in] none
 * @retval byteValue (0-0X7F)  or -1 if no byte is available
 **********************************************************/
int8_t BSP_CdcGetNextRxBytes(void)
{
//  BspUartDataType *pUart = &gBspUartData;
  int8_t byteValue;

  uint8_t *writePtr = (uint8_t *)(pRxWriteBuffer);
  
  if (writePtr < pRxBuffer)
  {
    writePtr += CDC_RX_BUFFER_SIZE;
  }  
  
  if (pRxReadBuffer != writePtr)
  {
    byteValue = (int8_t)(*(pRxReadBuffer));
    pRxReadBuffer++;

    if (pRxReadBuffer >= (pRxBuffer + CDC_RX_BUFFER_SIZE))
    {
      pRxReadBuffer = pRxBuffer;
    } 
  }
  else
  {
    byteValue = -1;
  }
  
  return (byteValue);
}

/******************************************************//**
 * @brief  Returns if there is a pending TX request in the UART
 * @param[in] none
 * @retval 0 if no pending TX request in the UART
 **********************************************************/
uint8_t BSP_CdcIsTxOnGoing(void)
{
//  BspUartDataType *pUart = &gBspUartData;
//  return (pUart->newTxRequestInThePipe||pUart->txBusy);
	return 0;
}

//#if defined(MARLIN)
///******************************************************//**
// * @brief  This function calls the WIFI TX parser and returns 0 when the command
//   in the buffer is not destinated to the gcode parser
// * @param[in] pBuf pointer to the buffer holding the command
// * @retval number of bytes destinated to the gcode parser
// **********************************************************/
//uint32_t BSP_CdcCommandsFilter(char *pBufCmd, uint8_t nxRxBytes)
//{
//  if (BSP_CdcParseRxAvalaibleBytes((char const*)pBufCmd, nxRxBytes)\
//        < BSP_WIFI_THRES_TO_GCODE_PARSER)
//  {
//    nxRxBytes = 0;
//  }
//  return nxRxBytes;
//}
//#endif

/******************************************************//**
 * @brief  This function sends data via the Uart in locking
 * mode (no interrupt used).
 * It should not be used except by the Error handler
 * @param[in]  pBuf pointer to the data to be sent
 * @param[in]  nbData number of bytes to be sent
 * @retval None
 **********************************************************/

void BSP_CdcLockingTx(uint8_t *pBuf, uint8_t nbData)
{
//   BspUartDataType *pUart = &gBspUartData;
  
//    HAL_UART_Transmit(&pUart->handle, pBuf, nbData, 1000);
	if(nbData)
	{
		uint32_t Len = nbData;
		CDC_Itf_SetTxBuffer(pBuf,&Len);
		CDC_Itf_Transmit(&Len);
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

