/**
  ******************************************************************************
  * @file    stm32f4xx_3dPrinter_uart.c
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
#include "stm32f4xx_3dprinter_uart.h"
#include "stm32f4xx_3dprinter_misc.h"
#include "stm32f4xx_3dprinter_wifi.h"
#include <string.h> /* for memcpy */
#include <stdarg.h> /* for va_start */
#include <stdio.h> /* for vsprintf */

/* Private defines -----------------------------------------------------------*/
/* Private constant ----------------------------------------------------------*/
#define UART_ERROR_TAG        (0x1000)
#define UART_ERROR(error)     BSP_MiscErrorHandler(error|UART_ERROR_TAG)

#ifdef USE_XONXOFF
#define BSP_UART_GET_NB_BYTES_IN_RX_BUFFER()  ((gBspUartData.pRxReadBuffer <= gBspUartData.pRxWriteBuffer)? \
                                    ( (unsigned int )(gBspUartData.pRxWriteBuffer - gBspUartData.pRxReadBuffer)): \
                                    ( (unsigned int )(gBspUartData.pRxWriteBuffer + UART_RX_BUFFER_SIZE - gBspUartData.pRxReadBuffer)))


#define BSP_UART_GET_NB_BYTES_IN_TX_BUFFER()  ((gBspUartData.pTxReadBuffer <= gBspUartData.pTxWriteBuffer)? \
                                    ( (unsigned int )(gBspUartData.pTxWriteBuffer - gBspUartData.pTxReadBuffer)): \
                                    ( (unsigned int )(gBspUartData.pTxWriteBuffer + UART_TX_BUFFER_SIZE - gBspUartData.pTxReadBuffer)))


#define BSP_UART_TX_THRESHOLD_XOFF  (UART_TX_BUFFER_SIZE / 50)
#define BSP_UART_TX_THRESHOLD_XON   (UART_TX_BUFFER_SIZE / 100)
#define BSP_UART_RX_THRESHOLD_XOFF  (UART_RX_BUFFER_SIZE / 2)
#define BSP_UART_RX_THRESHOLD_XON   (UART_RX_BUFFER_SIZE / 3)
#endif

/* Private functions ---------------------------------------------------------*/
uint8_t BSP_UartParseRxAvalaibleBytes(const char* pBuffer, uint8_t nbRxBytes);
    
/* Global variables ----------------------------------------------------------*/
BspUartDataType gBspUartData;
uint8_t gBspUartTxBuffer[2 * UART_TX_BUFFER_SIZE]; // real size is double to easily handle memcpy and tx uart
uint8_t gBspUartRxBuffer[2 * UART_RX_BUFFER_SIZE];
#ifdef USE_XONXOFF
static uint8_t  BspUartXonXoff = 0;
static uint8_t BspUartXoffBuffer[12] = " SEND XOFF\n";
static uint8_t BspUartXonBuffer[11] = " SEND XON\n";
#endif
/* Extern function -----------------------------------------------------------*/

/******************************************************//**
 * @brief  Usart Hw initialisation
 * @param None
 * @retval None
 **********************************************************/
void BSP_UartHwInit(uint32_t newBaudRate)
{
  BspUartDataType *pUart = &gBspUartData;
  
  pUart->handle.Instance = BSP_UART_DEBUG;
  pUart->handle.Init.BaudRate = newBaudRate;
  pUart->handle.Init.WordLength = UART_WORDLENGTH_8B;
  pUart->handle.Init.StopBits = UART_STOPBITS_1;
  pUart->handle.Init.Parity = UART_PARITY_NONE;
  pUart->handle.Init.Mode = UART_MODE_TX_RX;
  pUart->handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  pUart->handle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_DeInit(&pUart->handle) != HAL_OK)
  {
    UART_ERROR(1);
  }  

  if( HAL_UART_Init(&pUart->handle) != HAL_OK)
  {
    UART_ERROR(2);
  }
}

/******************************************************//**
 * @brief  Start the UART interface with the GUI 
 * @param None
 * @retval None
 **********************************************************/

void BSP_UartIfStart(void)
{
  BspUartDataType *pUart = &gBspUartData;
  pUart->pRxBuffer = (uint8_t *)gBspUartRxBuffer;
  pUart->pRxWriteBuffer =  pUart->pRxBuffer;
  pUart->pRxReadBuffer =  pUart->pRxBuffer;
  
  pUart->pTxBuffer = (uint8_t *)gBspUartTxBuffer;
  pUart->pTxWriteBuffer =  pUart->pTxBuffer;
  pUart->pTxReadBuffer =  pUart->pTxBuffer;
  pUart->pTxWrap  =  pUart->pTxBuffer + UART_TX_BUFFER_SIZE;
  
  pUart->rxBusy = RESET;
  pUart->txBusy = RESET;
  pUart->debugNbTxFrames = 0;
  pUart->debugNbRxFrames = 0;
  
  pUart->newTxRequestInThePipe = 0;
  pUart->nbBridgedBytes = 0;
  pUart->gCodeDataMode = 0;
    
  /* wait for 1 bytes on the RX uart */
  if (HAL_UART_Receive_IT(&pUart->handle, (uint8_t *)(&pUart->rxWriteChar), 1) != HAL_OK)
  {
    UART_ERROR(3);
  }  

  pUart->rxBusy = SET;
}

/******************************************************//**
 * @brief  Queue tx data to be sent on the UART
 * @param[in]  pBuf pointer to the data to be sent
 * @param[in]  nbData number of bytes to be sent
 * @retval None
 **********************************************************/
void BSP_UartIfQueueTxData(uint8_t *pBuf, uint8_t nbData)
{
  if (nbData != 0)
  {
    BspUartDataType *pUart= &gBspUartData;  
    int32_t nbFreeBytes = pUart->pTxReadBuffer - pUart->pTxWriteBuffer;
       
    if (nbFreeBytes <= 0)
    {
      nbFreeBytes += UART_TX_BUFFER_SIZE;
    }
    if (nbData > nbFreeBytes)
    {
        /* Uart Tx buffer is full */
        UART_ERROR(4);
    }
    
    //use of memcpy is safe as real buffer size is 2 * UART_TX_BUFFER_SIZE
    memcpy((uint8_t *)pUart->pTxWriteBuffer, pBuf, nbData);
    pUart->pTxWriteBuffer += nbData;
#if defined(MARLIN)
    if (pBuf[nbData-1] == '\n')
    {
      *pUart->pTxWriteBuffer = '\n';
      pUart->pTxWriteBuffer--;
      *pUart->pTxWriteBuffer = '\r';
      pUart->pTxWriteBuffer += 2;
      if (pUart->pTxWriteBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
      {
        pUart->pTxWrap = pUart->pTxWriteBuffer; 
        pUart->pTxWriteBuffer = pUart->pTxBuffer;
      }        
    }
#else
    if (pUart->pTxWriteBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
    {
      pUart->pTxWrap = pUart->pTxWriteBuffer; 
      pUart->pTxWriteBuffer = pUart->pTxBuffer;
    }
    BSP_UartIfSendQueuedData();
#endif
  }
}
   
/******************************************************//**
 * @brief  Send queued data to the GUI
 * @param None
 * @retval None
 **********************************************************/
void BSP_UartIfSendQueuedData(void)
{
    BspUartDataType *pUart = &gBspUartData;  
    
#ifdef USE_XONXOFF    
    if ((pUart->newTxRequestInThePipe == 0)&&
        (pUart->txBusy == RESET))
    {    
      if ((BspUartXonXoff == 2)||
      ((BSP_UART_GET_NB_BYTES_IN_TX_BUFFER()  > BSP_UART_TX_THRESHOLD_XOFF) && (BspUartXonXoff == 0)))
      {
        pUart->txBusy = SET;
        pUart->nbTxBytesOnGoing = 0;
        BspUartXoffBuffer[0] = 0x13;
        if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXoffBuffer, sizeof(BspUartXoffBuffer))!= HAL_OK)
        {
          UART_ERROR(10);
        }
        BspUartXonXoff = 3;
        return;
      }
      else if ((BspUartXonXoff == 1)||
        ((BSP_UART_GET_NB_BYTES_IN_RX_BUFFER()  < BSP_UART_RX_THRESHOLD_XON) && (BspUartXonXoff == 3)&& (BSP_UART_GET_NB_BYTES_IN_TX_BUFFER() < BSP_UART_TX_THRESHOLD_XON)))
      {
        pUart->txBusy = SET;
        pUart->nbTxBytesOnGoing = 0;
        BspUartXonBuffer[0] = 0x11;
        if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXonBuffer, sizeof(BspUartXonBuffer))!= HAL_OK)
        {
          UART_ERROR(11);
        } 
        BspUartXonXoff = 0;
        return;
      }
    }
#endif
    if ((pUart->newTxRequestInThePipe == 0)&&
        (pUart->txBusy == RESET)&&
        (pUart->pTxReadBuffer != pUart->pTxWriteBuffer))
    {
      int32_t nbTxBytes = pUart->pTxWriteBuffer - pUart->pTxReadBuffer;
      pUart->newTxRequestInThePipe++;
      if (nbTxBytes < 0)
      {
        nbTxBytes = pUart->pTxWrap - pUart->pTxReadBuffer;
      }
      
#if defined(MARLIN)
      if (pUart->pTxReadBuffer[nbTxBytes-1]!='\n')
      {
        pUart->newTxRequestInThePipe--;
        return;
      }
#endif
      pUart->txBusy = SET;
      pUart->nbTxBytesOnGoing = nbTxBytes;       
      
      //use of HAL_UART_Transmit_IT is safe as real buffer size is 2 * UART_TX_BUFFER_SIZE
      if(HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *) pUart->pTxReadBuffer, nbTxBytes)!= HAL_OK)
      {
        UART_ERROR(5);
      }
      
      pUart->debugNbTxFrames++;
      pUart->newTxRequestInThePipe--;
    }
}

/******************************************************//**
 * @brief  Tx Transfer completed callback
 * @param[in] UartHandle UART handle. 
 * @retval None
 **********************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BspUartDataType *pUart = &gBspUartData;  
  
  if (UartHandle == &(pUart->handle))
  {
    /* Set transmission flag: transfer complete*/
    pUart->txBusy = RESET;
    
#ifdef USE_XONXOFF
    if ((BspUartXonXoff == 2)||
        ((BSP_UART_GET_NB_BYTES_IN_TX_BUFFER()  > BSP_UART_TX_THRESHOLD_XOFF) && (BspUartXonXoff == 0)))
    {
      pUart->txBusy = SET;
      BspUartXoffBuffer[0] = 0x13;
      if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXoffBuffer, sizeof(BspUartXoffBuffer))!= HAL_OK)
      {
        UART_ERROR(10);
      }
      BspUartXonXoff = 3;
      return;
    }
    else if ((BspUartXonXoff == 1)||
        ((BSP_UART_GET_NB_BYTES_IN_RX_BUFFER()  < BSP_UART_RX_THRESHOLD_XON) && (BspUartXonXoff == 3)&& (BSP_UART_GET_NB_BYTES_IN_TX_BUFFER() < BSP_UART_TX_THRESHOLD_XON)))
    {
      pUart->txBusy = SET;
      BspUartXonBuffer[0] = 0x11;
      if (HAL_UART_Transmit_IT(&pUart->handle, (uint8_t *)&BspUartXonBuffer, sizeof(BspUartXonBuffer))!= HAL_OK)
      {
        UART_ERROR(11);
      } 
      BspUartXonXoff = 0;
      return;
    }
#endif
    pUart->pTxReadBuffer += pUart->nbTxBytesOnGoing;
    
    if (pUart->pTxReadBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
    {
      pUart->pTxReadBuffer  = pUart->pTxBuffer;
    }          
    
    if (pUart->uartTxDoneCallback != 0)
    {
      pUart->uartTxDoneCallback();
    }
  }
#if !defined(NO_WIFI)
  else
  {
    BSP_WifiUartTxCpltCallback(UartHandle);
  }
#endif //#if !defined(NO_WIFI)
}

/******************************************************//**
 * @brief  Rx Transfer completed callback
 * @param[in] UartHandle UART handle. 
 * @retval None
 **********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BspUartDataType *pUart = &gBspUartData;
#if !defined(NO_WIFI)
  unsigned char *pChar = NULL;
#endif //#if !defined(NO_WIFI)
  
  if (UartHandle == &(pUart->handle))
  {
    *pUart->pRxWriteBuffer = pUart->rxWriteChar;
    if (HAL_UART_Receive_IT(&pUart->handle, (uint8_t *)(&pUart->rxWriteChar), 1) != HAL_OK)
    {
      UART_ERROR(6);
    }
    
    pUart->pRxWriteBuffer++;
    
    if (pUart->pRxWriteBuffer >= (pUart->pRxBuffer + UART_RX_BUFFER_SIZE))
    {
      pUart->pRxWriteBuffer = pUart->pRxBuffer;
    }
    
#ifdef USE_XONXOFF    
    if ((BSP_UART_GET_NB_BYTES_IN_RX_BUFFER()  > BSP_UART_RX_THRESHOLD_XOFF) && (BspUartXonXoff == 0))
    {
      BspUartXonXoff = 2;
    }
    else if ((BSP_UART_GET_NB_BYTES_IN_RX_BUFFER()  < BSP_UART_RX_THRESHOLD_XON) && (BspUartXonXoff == 3)&& (BSP_UART_GET_NB_BYTES_IN_TX_BUFFER() <BSP_UART_TX_THRESHOLD_XON))
    {
      BspUartXonXoff = 1;
    }
#endif    
    if (pUart->pRxWriteBuffer == pUart->pRxReadBuffer)
    {
      // Rx buffer is full 
      UART_ERROR(7);
    }    
    
    if (pUart->uartRxDataCallback != 0)
    {
      pUart->uartRxDataCallback((uint8_t *)pUart->pRxReadBuffer,pUart->pRxWriteBuffer - pUart->pRxReadBuffer);
    }
    pUart->debugNbRxFrames++;
  }
#if !defined(NO_WIFI)
  else
  {
    uint32_t tmp = BSP_WifiUartRxCpltCallback(UartHandle, &pChar);
    if (tmp != 0)
    {
      memcpy((uint8_t*)(pUart->pRxWriteBuffer), pChar, tmp);
      pUart->pRxWriteBuffer += tmp;
      *pUart->pRxWriteBuffer = '\0';
      if (pUart->pRxWriteBuffer >= (pUart->pRxBuffer + UART_RX_BUFFER_SIZE))
      {
        pUart->pRxWriteBuffer = pUart->pRxBuffer;
      }
    }
  }
#endif //#if !defined(NO_WIFI)
}

/******************************************************//**
 * @brief  Uart Error callback
 * @param[in] UartHandle UART handle. 
 * @retval None
 **********************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_ERROR(8);
}

/******************************************************//**
 * @brief  Attaches a callback which will be called when
 * a complete rx uart buffer is ready
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
void BSP_UartAttachRxDataHandler(void (*callback)(uint8_t *, uint8_t))
{
  BspUartDataType *pUart = &gBspUartData;  
  pUart->uartRxDataCallback = (void (*)(uint8_t *, uint8_t))callback;
}

/******************************************************//**
 * @brief  Attaches a callback which will be called when
 * a complete tx uart buffer is ready
 * @param[in] callback Name of the callback to attach 
 * @retval None
 **********************************************************/
void BSP_UartAttachTxDoneCallback(void (*callback)(void))
{
  BspUartDataType *pUart = &gBspUartData;  
  pUart->uartTxDoneCallback = (void (*)(void))callback;
}


/******************************************************//**
 * @brief  This function trigs the transmission of a string over the UART 
 *             for printing
 * @param[in] format string with formatting
 * @param[in]  Optional arguments to fit with formatting
 * @retval Lengthj of the string to print (uint32_t)
 **********************************************************/
uint32_t BSP_UartPrintf(const char* format,...)
{
  BspUartDataType *pUart = &gBspUartData;  
  va_list args;
   uint32_t size;
  uint32_t retSize = 0;
  int32_t nbFreeBytes = pUart->pTxReadBuffer - pUart->pTxWriteBuffer;
  
  if (nbFreeBytes <= 0)
  { 
    nbFreeBytes += UART_TX_BUFFER_SIZE;
  }  
  
  char *writeBufferp =(char *) pUart->pTxWriteBuffer;
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
    if ( size > nbFreeBytes )
    {
      UART_ERROR(9);
    }
    pUart->pTxWriteBuffer += size;
    if (pUart->pTxWriteBuffer >= pUart->pTxBuffer + UART_TX_BUFFER_SIZE)
    {
      pUart->pTxWrap = pUart->pTxWriteBuffer; 
      pUart->pTxWriteBuffer  = pUart->pTxBuffer;
    }
      
    BSP_UartIfSendQueuedData();
  }
  return(retSize);  
}

/******************************************************//**
 * @brief  This function returns the number of bytes received via the UART
 * @param[in] fnone
  * @retval nxRxBytes nb received bytes
 **********************************************************/
uint32_t BSP_UartGetNbRxAvalaibleBytes(void)
{
  BspUartDataType *pUart = &gBspUartData;  
  uint8_t *writePtr = (uint8_t *)(pUart->pRxWriteBuffer - 1);
  
  if (writePtr < pUart->pRxBuffer)
  {
    writePtr += UART_RX_BUFFER_SIZE;
  }  
  
  //waitline feed to have a complete line before processing bytes
  if ((int8_t)(*writePtr) != 0XA)
    return (0);
  
  int32_t nxRxBytes = pUart->pRxWriteBuffer - pUart->pRxReadBuffer;
  if (nxRxBytes < 0)
  {
    nxRxBytes += UART_RX_BUFFER_SIZE;
  }
#if !defined(MARLIN)
  if (nxRxBytes != 0)
  {
    uint8_t result = BSP_UartParseRxAvalaibleBytes((char const*)pUart->pRxReadBuffer, nxRxBytes);
    if (result < BSP_WIFI_THRES_TO_GCODE_PARSER)
    {
      //The available bytes will not to go into the Gcode parser
      pUart->pRxReadBuffer += nxRxBytes;
      nxRxBytes = 0; 
      if (pUart->pRxReadBuffer >= (pUart->pRxBuffer + UART_RX_BUFFER_SIZE))
      {
        pUart->pRxReadBuffer = pUart->pRxBuffer;
      }
    }
  }
#endif
  
  return ((uint32_t) nxRxBytes );
}

/******************************************************//**
 * @brief  This function returns the number of bytes received via the UART
 * @param[in] fnone
  * @retval nxRxBytes nb received bytes
 **********************************************************/
uint8_t BSP_UartParseRxAvalaibleBytes(const char* pBuffer, uint8_t nbRxBytes)
{
  return (BSP_WifiParseTxBytes(pBuffer, nbRxBytes, BSP_WIFI_SOURCE_IS_DEBUG_UART));
}

/******************************************************//**
 * @brief  This function returns the first byte available on the UART
 * @param[in] none
 * @retval byteValue (0-0X7F)  or -1 if no byte is available
 **********************************************************/
int8_t BSP_UartGetNextRxBytes(void)
{
  BspUartDataType *pUart = &gBspUartData;  
  int8_t byteValue;

  uint8_t *writePtr = (uint8_t *)(pUart->pRxWriteBuffer);
  
  if (writePtr < pUart->pRxBuffer)
  {
    writePtr += UART_RX_BUFFER_SIZE;
  }  
  
  if (pUart->pRxReadBuffer != writePtr)
  {
    byteValue = (int8_t)(*(pUart->pRxReadBuffer));
    pUart->pRxReadBuffer++;

    if (pUart->pRxReadBuffer >= (pUart->pRxBuffer + UART_RX_BUFFER_SIZE))
    {
      pUart->pRxReadBuffer = pUart->pRxBuffer;
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
uint8_t BSP_UartIsTxOnGoing(void)
{
  BspUartDataType *pUart = &gBspUartData; 
  return (pUart->newTxRequestInThePipe||pUart->txBusy);
}

#if defined(MARLIN)
/******************************************************//**
 * @brief  This function calls the WIFI TX parser and returns 0 when the command 
   in the buffer is not destinated to the gcode parser
 * @param[in] pBuf pointer to the buffer holding the command
 * @retval number of bytes destinated to the gcode parser
 **********************************************************/
uint32_t BSP_UartCommandsFilter(char *pBufCmd, uint8_t nxRxBytes)
{
  if (BSP_UartParseRxAvalaibleBytes((char const*)pBufCmd, nxRxBytes)\
        < BSP_WIFI_THRES_TO_GCODE_PARSER)
  {
    nxRxBytes = 0; 
  }
  return nxRxBytes;  
}
#endif

/******************************************************//**
 * @brief  This function sends data via the Uart in locking
 * mode (no interrupt used).
 * It should not be used except by the Error handler
 * @param[in]  pBuf pointer to the data to be sent
 * @param[in]  nbData number of bytes to be sent
 * @retval None
 **********************************************************/

void BSP_UartLockingTx(uint8_t *pBuf, uint8_t nbData)
{
   BspUartDataType *pUart = &gBspUartData;  
  
    HAL_UART_Transmit(&pUart->handle, pBuf, nbData, 1000);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

