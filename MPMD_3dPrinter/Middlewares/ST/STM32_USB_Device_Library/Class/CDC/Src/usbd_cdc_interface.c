/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 
/* Exported functions --------------------------------------------------------*/
void BSP_CDC_RxCpltCallback(uint8_t* Buf, uint32_t *Len);
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//TODO: Find a strategy to selectively disable UART/CDC
//#ifdef STM32_USE_USB_CDC
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  256
//#else
//#define APP_RX_DATA_SIZE  1
//#define APP_TX_DATA_SIZE  1
//#endif //STM32_USE_USB_CDC

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//Pretty sure these values are arbitrary, but there is some expectation to act like a real COM port
USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };
//UserRxBuffer is fed directly USBD driver, should not be bigger than the MAX_FS_BUFFER_SIZE
volatile uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
//UserTxBuffer is cleared MAX_FS_BUFFER_SIZE at a time, so it should be at least this big for efficiency
volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
volatile uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USB */
volatile uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */

/* TIM handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

static void TIM_Config(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* USART configured as follow:
      - Word Length = 8 Bits
      - Stop Bit    = One Stop bit
      - Parity      = No parity
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
    /* Initialization Error */

  
  /*##-2- Put UART peripheral in IT reception process ########################*/
  /* Any data received will be stored in "UserRxBuffer" buffer  */
    /* Transfer error in reception process */
  
  /*##-3- Configure the TIM Base generation  #################################*/
  TIM_Config();

  /*##-4- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
	  CDC_ERROR(1);
  }
  
  /*##-5- Set Application Buffers ############################################*/
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
  
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;     
    
    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }
  
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_QueueTxBytes
  *         Queues up the data to be shifted out on the USB CDC interface.  Actual
  *         data transfer occurs during the timer background interrupt, so if there
  *         is not enough space we will wait until the queue empties.
  * @param  *Buf: Incoming character buffer to shift out on the USB CDC interface
  * @param  Len: Number of data to be sent (in bytes)
  */
void CDC_Itf_QueueTxBytes(uint8_t *Buf, uint32_t Len) {
	static const uint32_t timeout = 5;//Use much shorter timeout per character
	for (uint32_t i=0;i<Len;i++) {
		//Wait until the queue has cleared
		uint8_t nBytes = CDC_Itf_GetNbTxAvailableBytes();
		//Fill up the queue while disconnected, but do not block the thread
		uint32_t startTick = HAL_GetTick();
		while(nBytes<1 && CDC_Itf_IsConnected()) {
			BSP_LED_On(LED_GREEN);
			nBytes = CDC_Itf_GetNbTxAvailableBytes();
			if((HAL_GetTick()-startTick)>timeout)
				return;
		}
		if(!CDC_Itf_IsConnected() && nBytes<1)
			break;
		BSP_LED_Off(LED_GREEN);
		UserTxBuffer[UserTxBufPtrIn] = Buf[i];
		UserTxBufPtrIn++;
		//Wraparound
		if(UserTxBufPtrIn >= APP_TX_DATA_SIZE)
			UserTxBufPtrIn = 0;
	}
}

/**
  * @brief  CDC_Itf_GetNbTxQueuedBytes
  *         Returns the number of bytes currently queued for transmit by USB CDC
  * @retval Number of queued bytes
  */
uint32_t CDC_Itf_GetNbTxQueuedBytes(void)
{
	uint32_t nB;
	if(UserTxBufPtrIn >= UserTxBufPtrOut)
		nB = UserTxBufPtrIn - UserTxBufPtrOut;
	else//wraparound
		nB = APP_TX_DATA_SIZE + (UserTxBufPtrIn-UserTxBufPtrOut);
	return nB;
}

/**
  * @brief  CDC_Itf_GetNbTxAvailableBytes
  *         Returns the number of bytes available to queue
  * @retval Number of bytes available
  */
uint32_t CDC_Itf_GetNbTxAvailableBytes(void)
{
	return APP_TX_DATA_SIZE - CDC_Itf_GetNbTxQueuedBytes()-1;
}
/**
  * @brief  CDC_Itf_IsTransmitting
  *         Returns 1 if the USB device is currently transmitting
  * @retval 1 if connected, 0 otherwise
  */
uint8_t CDC_Itf_IsTransmitting(void) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) USBD_Device.pClassData;
	return (hcdc->TxState == 1);
}

/**
  * @brief  CDC_Itf_IsTxQueueEmpty
  *         Returns 1 if we the TX queue is empty
  * @retval 1 if empty, 0 otherwise
  */
uint8_t CDC_Itf_IsTxQueueEmpty(void) {
	return (!CDC_Itf_IsTransmitting() && CDC_Itf_GetNbTxQueuedBytes()==0);
}

/**
  * @brief  TIM period elapsed callback
  * Data will be shifted out at CDC_POLLING_INTERVAL, by CDC_DATA_FS_MAX_PACKET_SIZE
  * at a time.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t buffptr;
  uint32_t buffsize;
  static uint32_t lastSuccessfulTX = 0;
  if(UserTxBufPtrOut != UserTxBufPtrIn) //Do we have data?
  {
    if(UserTxBufPtrOut > UserTxBufPtrIn) /* rollback */
    {
      buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
    }
    else
    {
      buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
    }
    //We can't queue up more than 64 bytes at a time without changing the endpoint
    buffsize = MIN(buffsize,USB_FS_MAX_PACKET_SIZE-1);
    buffptr = UserTxBufPtrOut;
    //Set up a EPIN interrupt using UserTxBuffer
    if(USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[buffptr], buffsize) != USBD_OK)
    {
    	if(HAL_GetTick()-lastSuccessfulTX>1000) //Haven't sent data successfully in 1000ms, clear the buffer
    		UserTxBufPtrOut = UserTxBufPtrIn;
    	return;
    }
    lastSuccessfulTX = HAL_GetTick();
    if(USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
    {
      UserTxBufPtrOut += buffsize;
      if (UserTxBufPtrOut == APP_TX_DATA_SIZE)//We align buffsize to the end of the buffer above
      {
        UserTxBufPtrOut = 0;
      }
    }
  }
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are retrieved here.  Transfer to
  *         larger user buffer is done in th eRxCpltCallback
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Pointer to number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
    BSP_CDC_RxCpltCallback(Buf,Len);
    USBD_CDC_ReceivePacket(&USBD_Device);
    return (USBD_OK);
}

/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None.
  */
static void TIM_Config(void)
{
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follow:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = (CDC_POLLING_INTERVAL*1000) - 1;
  TimHandle.Init.Prescaler = 84-1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
	  CDC_ERROR(4);
  }
}

/**
  * @brief  CDC_Itf_IsConnected
  *         Returns 1 if we are connected
  * @retval 1 if connected, 0 otherwise
  */
uint8_t CDC_Itf_IsConnected(void)
{
	return USBD_Device.dev_state==USBD_STATE_CONFIGURED;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

