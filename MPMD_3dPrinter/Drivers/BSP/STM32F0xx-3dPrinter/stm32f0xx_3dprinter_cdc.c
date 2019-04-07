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
#include "stm32f0xx_mpmd.h"
/* Private defines -----------------------------------------------------------*/
/* Private constant ----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/
volatile USBD_HandleTypeDef USBD_Device;
//TODO: investigate whether these actually need to be 2x BUFFER_SIZE, this is just copied direct from STM
volatile uint8_t gBspCdcTxBuffer[CDC_TX_BUFFER_SIZE]; // real size is double to easily handle memcpy and tx uart
volatile uint8_t gBspCdcRxBuffer[CDC_RX_BUFFER_SIZE];

static volatile uint8_t *pRxBuffer = gBspCdcRxBuffer;
static volatile uint8_t *pRxWriteBuffer;
static volatile uint8_t *pRxReadBuffer;
uint32_t debugNbRxFrames = 0;
uint32_t debugNbTxFrames = 0;
#ifdef USE_XONXOFF
static uint8_t BspUartXonXoff = 0;
static uint8_t BspUartXoffBuffer[12] = " SEND XOFF\n";
static uint8_t BspUartXonBuffer[11] = " SEND XON\n";
#endif
/* Extern function -----------------------------------------------------------*/

/******************************************************//**
 * @brief  USB CDC Hw initialisation
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
}

void BSP_CdcHwDeInit()
{
	USBD_DeInit(&USBD_Device);
}


/******************************************************//**
 * @brief  Start the USB CDC interface
 * @param None
 * @retval None
 **********************************************************/
void BSP_CdcIfStart(void)
{
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  pRxWriteBuffer =  pRxBuffer;
  pRxReadBuffer =  pRxBuffer;
  debugNbTxFrames = 0;
  debugNbRxFrames = 0;
//Delay to allow connection before trying to send data
  HAL_Delay(10);
}

void BSP_CdcIfStop(void)
{
	  USBD_Stop(&USBD_Device);
	  pRxWriteBuffer =  pRxBuffer;
	  pRxReadBuffer =  pRxBuffer;
	  debugNbTxFrames = 0;
	  debugNbRxFrames = 0;
	//Delay to allow connection before trying to send data
	  HAL_Delay(10);
}

/******************************************************//**
 * @brief  Queue tx data to be sent on the UART.  Data will be sent on the next
 * timer expired interrupt to prevent race conditions
 * @param[in]  pBuf pointer to the data to be sent
 * @param[in]  nbData number of bytes to be sent
 * @retval None
 **********************************************************/
void BSP_CdcIfQueueTxData(uint8_t *pBuf, uint8_t nbData)
{
    if(nbData) {
    	uint32_t Len = nbData;
    	CDC_Itf_QueueTxBytes(pBuf, Len);
    }
}
   
/******************************************************//**
 * @brief  Data is sent on background timer task, wait until all data
 * is shifted out
 * @param None
 * @retval None
 **********************************************************/
void BSP_CdcIfSendQueuedData()
{
	static const uint32_t timeout = 1000;
	uint32_t startTick = HAL_GetTick();
	while(BSP_CdcIsTxOnGoing()) {
		BSP_LED_On(LED_GREEN);
		if((HAL_GetTick()-startTick)>timeout)
			return;
	}
	BSP_LED_Off(LED_GREEN);
}

/******************************************************//**
 * @brief  Rx Transfer completed callback
 * Transfers data from the CDC ITF buffer to the larger user
 * RX buffer
 * @param[in] uint8_t* Buf pointer to the incoming data buffer from
 * the USB CDC ITF driver
 * @param[in] *Len pointer containing the number of received bytes
 * @retval None
 **********************************************************/
uint8_t rxInProgress = 0;
void BSP_CDC_RxCpltCallback(uint8_t* Buf, uint32_t *Len)
{
	rxInProgress = 0;
	uint32_t startnB = BSP_CdcGetNbRxAvailableBytes(0);
	BSP_LED_On(LED_GREEN);
	uint8_t *writePtr = (uint8_t *)(Buf);
	uint32_t bytesToCopy = MIN(*Len,CDC_RX_BUFFER_SIZE);
	uint32_t firstBytesToCopy = MIN(bytesToCopy,
	&pRxBuffer[CDC_RX_BUFFER_SIZE]-pRxWriteBuffer);
	uint32_t secondBytesToCopy = bytesToCopy - firstBytesToCopy;
	memcpy(pRxWriteBuffer,
			writePtr,
			firstBytesToCopy);
	pRxWriteBuffer += firstBytesToCopy;
	if(pRxWriteBuffer==&pRxBuffer[CDC_RX_BUFFER_SIZE])
	{
		if(secondBytesToCopy>0)
			memcpy(pRxBuffer,
				&writePtr[firstBytesToCopy],
				secondBytesToCopy);
		pRxWriteBuffer = &pRxBuffer[secondBytesToCopy];
	}
	BSP_LED_Off(LED_GREEN);
//Wait until at least half the buffer is free before signaling to host we are available for reception.	Otherwise check in the timer ISR periodically
if(CDC_RX_BUFFER_SIZE-BSP_CdcGetNbRxAvailableBytes(0)>CDC_RX_BUFFER_SIZE/2)
		USBD_CDC_ReceivePacket(&USBD_Device);
	//We've overflowed, throw an error
	if(*Len>0 && BSP_CdcGetNbRxAvailableBytes(0)<=startnB)
		CDC_ERROR(10);
}

/******************************************************//**
 * @brief  This function trigs the transmission of a string over the USB CDC
 *             for printing data is queued, but is sent out on the background
 *             timer task.  If the queue is full, this routine will be blocked
 *             until it is empty enough to queue the entire string
 * @param[in] format string with formatting
 * @param[in]  Optional arguments to fit with formatting
 * @retval Length of the string to print (uint32_t)
 **********************************************************/
uint32_t BSP_CdcPrintf(const char* format,...)
{
  va_list args;
  uint32_t size;
  uint32_t retSize = 0;
  unsigned char *writeBufferp = gBspCdcTxBuffer;
  /* the string to transmit is copied in the temporary buffer in order to    */
  /* check its size.                                                         */
  va_start(args, format);
  size=vsprintf((char *)writeBufferp, (const char*)format, args);
  va_end(args);
   
  retSize = size;   
  if (size != 0) {
    if ( size > CDC_TX_BUFFER_SIZE ) {
      CDC_ERROR(9);
    }
    //Queue data for the background timer task
    CDC_Itf_QueueTxBytes(writeBufferp,size);
  }
  return(retSize);  
}

/******************************************************//**
 * @brief  This function returns the number of bytes received via the USB CDC
 * @param[in] fnone
  * @retval nxRxBytes nb received bytes
 **********************************************************/
uint32_t BSP_CdcGetNbRxAvailableBytes(uint8_t waitForNewLine)
{
  uint8_t *writePtr = (uint8_t *)(pRxWriteBuffer - 1);
  
  if (writePtr < pRxBuffer)
  {
    writePtr += CDC_RX_BUFFER_SIZE;
  }  
  
  //waitline feed to have a complete line before processing bytes
  if (waitForNewLine && ((*writePtr) != '\r' && (*writePtr) != '\n'))
    return (0);
  
  int32_t nxRxBytes = pRxWriteBuffer - pRxReadBuffer;
  if (nxRxBytes < 0)  {
    nxRxBytes += CDC_RX_BUFFER_SIZE;
  }
  return ((uint32_t) nxRxBytes );
}

uint32_t BSP_CdcCopyNextRxBytes(uint8_t *buff, uint32_t maxlen)
{
	BSP_LED_On(LED_BLUE);
	uint32_t bytesToCopy = MIN(maxlen,BSP_CdcGetNbRxAvailableBytes(0));
	uint32_t firstBytesToCopy = MIN(bytesToCopy,
											&pRxBuffer[CDC_RX_BUFFER_SIZE]-pRxReadBuffer);
	uint32_t secondBytesToCopy = bytesToCopy - firstBytesToCopy;
	memcpy(buff,
			pRxReadBuffer,
			firstBytesToCopy);
	pRxReadBuffer += firstBytesToCopy;
	if(pRxReadBuffer==&pRxBuffer[CDC_RX_BUFFER_SIZE])	{
		if(secondBytesToCopy)
			memcpy(&buff[firstBytesToCopy],
				pRxBuffer,
				secondBytesToCopy);
		pRxReadBuffer = &pRxBuffer[secondBytesToCopy];
	}
	BSP_LED_Off(LED_BLUE);
	return bytesToCopy;
}

/******************************************************//**
 * @brief  This function returns the first byte available on the USB CDC
 * @param[in] none
 * @retval byteValue (0-0X7F)  or -1 if no byte is available
 **********************************************************/
int8_t BSP_CdcGetNextRxByte(void)
{
  int8_t byteValue;

  uint8_t *writePtr = (uint8_t *)(pRxWriteBuffer);
  
  if (writePtr < pRxBuffer)  {
    writePtr += CDC_RX_BUFFER_SIZE;
  }  
  
  if (pRxReadBuffer != writePtr)  {
    byteValue = (int8_t)(*(pRxReadBuffer));
    pRxReadBuffer++;

    if (pRxReadBuffer >= (pRxBuffer + CDC_RX_BUFFER_SIZE))    {
      pRxReadBuffer = pRxBuffer;
    } 
  }
  else  {
    byteValue = -1;
  }
  
  return (byteValue);
}

/******************************************************//**
 * @brief  Returns if there is a pending TX request in the USB CDC
 * @param[in] none
 * @retval 0 if no pending TX request in the UART
 **********************************************************/
uint8_t BSP_CdcIsTxOnGoing(void)
{
	return !CDC_Itf_IsTxQueueEmpty();
}

/******************************************************//**
 * @brief  This function sends data via the USB in locking
 * mode.  Data is still sent out on the background queue, but
 * we will not return until it has all been shifted out
 * @param[in]  pBuf pointer to the data to be sent
 * @param[in]  nbData number of bytes to be sent
 * @retval None
 **********************************************************/

void BSP_CdcLockingTx(uint8_t *pBuf, uint8_t nbData)
{
	CDC_Itf_QueueTxBytes(pBuf,nbData);
	BSP_CdcIfSendQueuedData();
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

