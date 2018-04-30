/**
  ******************************************************************************
  * @file    stm32f0xx_3dPrinter_wifi.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   wifi functions of 3D Printer BSP driver 
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
#include "stm32f0xx_3dprinter_wifi.h"
#include "stm32f0xx_3dprinter_misc.h"
#include "stm32f0xx_3dprinter_uart.h"
#include "string.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define WIFI_COMMAND_MODE     (0x00)
#define WIFI_DATA_MODE        (0x01)
#define WIFI_FILE_APPEND      (0x02)
#define WIFI_DATA_MODE_AND_GCODE_PARSER             (0x03)
#define WIFI_COMMAND_MODE_AND_GCODE_PARSER          (0x04)
#define WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER (WIFI_COMMAND_MODE_AND_GCODE_PARSER | 0x01)

#define WIFI_INPUT_CGI_DELAY                    (500)
#define WIFI_COMMAND_MODE_WEB_WRITE_TIMEOUT     (15000)

#define WIFI_FILE_CREATION_PENDING              (1)
#define WIFI_CONFIG_FS_UPDATE_PENDING           (3+WIFI_FILE_CREATION_PENDING)
#define WIFI_CONFIG_FS_UPDATE_START             (4+WIFI_FILE_CREATION_PENDING)
#define WIFI_CONFIG_FS_UPDATE_CONNECT           (5+WIFI_FILE_CREATION_PENDING)
#define WIFI_CONFIG_FS_UPDATE_CHECK             (6+WIFI_FILE_CREATION_PENDING)
#define WIFI_CONFIG_FS_UPDATE_PRINT_VERSION     (7+WIFI_FILE_CREATION_PENDING)
#define WIFI_CONFIG_FS_UPDATE                   (8+WIFI_FILE_CREATION_PENDING)
#define WIFI_CONFIG_SAVE_CASE                   (8+WIFI_CONFIG_FS_UPDATE)
#define WIFI_CONFIG_ITEMS                       (14)
#if defined(PROD_TEST)
#define WIFI_CONFIG_FW_UPDATE                   (WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS+1)
#endif //#if defined(PROD_TEST)

/* Private constant ----------------------------------------------------------*/
#define WIFI_ERROR_TAG        (0x4000)
#define WIFI_ERROR(error)     BSP_MiscErrorHandler(error|WIFI_ERROR_TAG)

#define WIFI_UART_RX_BUFFER_SIZE                (512)
#define WIFI_UART_TX_BUFFER_SIZE (256)

#define WIFI_FILE_MAX_SIZE                      (4096)
#define WIFI_FILE_NAME_LENGTH                   (30)

#define WIFI_UART_GCODE_START_STRING "GcodesStart"
#define WIFI_UART_GCODE_STOP_STRING "GcodesStop"
#define WIFI_UART_FILE_WRITE_START_STRING "FileWriteStart"
//Two stop strings can occur depending on the wifi module firmware and mode
#define WIFI_UART_FILE_WRITE_STOP_STRING "+WIND:56:Insert message to client: FileWriteStop"
#define WIFI_UART_FILE_WRITE_STOP_STRING_1 "+WIND:56:Insert message to client:1: FileWriteStop"

/* Global variables ---------------------------------------------------------*/
BspWifiDataType gBspWifiData;
uint8_t gBspWifiUartRxBuffer[2*WIFI_UART_RX_BUFFER_SIZE];
uint8_t gBspWifiUartTxBuffer[2*WIFI_UART_TX_BUFFER_SIZE];
char previousString[256];

/* Private function -----------------------------------------------------------*/
static void BSP_WifiGpioInit(void);
static void BSP_WifiUartInit(uint32_t baudRate);
uint8_t BSP_WifiParseRxBytes(uint8_t offset, uint8_t nbBytes);
void BSP_WifiRestart(void);

#if defined(PROD_TEST)
/******************************************************//**
 * @brief  Wifi Hw initialisation
 * @param baudRate UART baud rate used between the main stm32
 * and the wifi module
 * @param ssid wifi module SSID
 * @param wepKey wifi module WEP KEY
 * @param fwVersionExpected expected version of the wifi module firmware
 * @param fsVersionExpected expected version of the wifi module file system
 * @retval None
 **********************************************************/
void BSP_WifiHwInit(uint32_t baudRate, char* ssid, char* wepKey, char* expectedFwVersion, char* expectedFsVersion)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  
  strcpy(pWifi->ssid,ssid);
  strcpy(pWifi->wepKey,wepKey);
  strcpy(pWifi->expectedFwVersion,expectedFwVersion);
  strcpy(pWifi->expectedFsVersion,expectedFsVersion);
  BSP_WifiGpioInit();
  BSP_WifiUartInit(baudRate);
}
#else //#if defined(PROD_TEST)
/******************************************************//**
 * @brief  Wifi Hw initialisation
 * @param baudRate UART baud rate used between the main stm32
 * and the wifi module
 * @param ssid wifi module SSID
 * @param wepKey wifi module WEP KEY
 * @retval None
 **********************************************************/
void BSP_WifiHwInit(uint32_t baudRate, char* ssid, char* wepKey)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  
  strcpy(pWifi->ssid,ssid);
  strcpy(pWifi->wepKey,wepKey);
  BSP_WifiGpioInit();
  BSP_WifiUartInit(baudRate);
}
#endif //#else //#if defined(PROD_TEST)

/******************************************************//**
 * @brief  Initialisation of the GPIOs for WIFI module
 * @param None
 * @retval None
 **********************************************************/
void BSP_WifiGpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
    
  GPIO_InitStruct.Pin = BSP_WIFI_BOOT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BSP_WIFI_BOOT_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = BSP_WIFI_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BSP_WIFI_RESET_PORT, &GPIO_InitStruct);
  /* Releasing the reset starts the wifi module and so, the communication on  */
  /* the wifi uart starts                                                     */
  HAL_GPIO_WritePin(BSP_WIFI_RESET_PORT, BSP_WIFI_RESET_PIN, GPIO_PIN_SET); 
  
  GPIO_InitStruct.Pin = BSP_WIFI_WAKEUP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BSP_WIFI_WAKEUP_PORT, &GPIO_InitStruct);  
}

/******************************************************//**
 * @brief  Initialisation of the Uart for WIFI module
 * @param[in] baudRate UART baud rate
 * @retval None
 **********************************************************/
void BSP_WifiUartInit(uint32_t baudRate)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  
  pWifi->uartHandle.Instance = BSP_WIFI_UART;
  pWifi->uartHandle.Init.BaudRate = baudRate;
  pWifi->uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  pWifi->uartHandle.Init.StopBits = UART_STOPBITS_1;
  pWifi->uartHandle.Init.Parity = UART_PARITY_NONE;
  pWifi->uartHandle.Init.Mode = UART_MODE_TX_RX;
  pWifi->uartHandle.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  pWifi->uartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_DeInit(&pWifi->uartHandle) != HAL_OK)
  {
    WIFI_ERROR(1);
  }  

  if( HAL_UART_Init(&pWifi->uartHandle) != HAL_OK)
  {
    WIFI_ERROR(2);
  }  
  
  pWifi->state = BSP_WIFI_UART_BYTES_NONE;
  pWifi->txFlag = RESET;
  pWifi->pRxBuffer = (uint8_t *)gBspWifiUartRxBuffer; 
  pWifi->pRxWriteBuffer =  pWifi->pRxBuffer;
  pWifi->pRxReadBuffer =  pWifi->pRxBuffer;
  pWifi->pTxBuffer = (uint8_t *)gBspWifiUartTxBuffer; 
  pWifi->pTxWriteBuffer =  pWifi->pTxBuffer;
  pWifi->mode = WIFI_COMMAND_MODE;
  pWifi->restartPending = 0;
  pWifi->commandPending = 0;
  pWifi->configPending = -1;
  pWifi->fileCreationPending = 0;
#if defined(PROD_TEST)
  pWifi->fwUpdatePending = 0;
  pWifi->fsUpdatePending = -1;
#endif //#if defined(PROD_TEST)
    
 /* wait for 1 bytes on the RX uart */
  if (HAL_UART_Receive_DMA(&pWifi->uartHandle, (uint8_t *)(pWifi->pRxWriteBuffer), 1) != HAL_OK)
  {
    WIFI_ERROR(3);
  }  
  pWifi->pRxWriteBuffer++;
}

/******************************************************//**
 * @brief  Wifi Uart Tx Transfer completed callback
 * @param[in] UartHandle UART handle
 * @retval None
 **********************************************************/
void BSP_WifiUartTxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  pWifi->txFlag = RESET;
}

/******************************************************//**
 * @brief  Wifi Uart Rx Transfer completed callback
 * @param[in] UartHandle UART handle
 * @param[in] pointer to WIFI uart RX buffer pointer
 * @retval result number of bytes to copy to UART debug RX
 **********************************************************/
uint8_t BSP_WifiUartRxCpltCallback(UART_HandleTypeDef *UartHandle,\
  unsigned char **c)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  uint8_t result = 0;
  uint8_t nbRxBytes;
  uint8_t *pRxWriteWrap;
  uint8_t character = *(pWifi->pRxWriteBuffer-1);
  uint8_t parseOffset = 0;
 
  pRxWriteWrap = pWifi->pRxWriteBuffer;
  if ((pWifi->pRxWriteBuffer >=\
    (pWifi->pRxBuffer + WIFI_UART_RX_BUFFER_SIZE - 1))&&(character == '\n'))
  {
    pWifi->pRxWriteBuffer = pWifi->pRxBuffer;
  }  
  if (HAL_UART_Receive_DMA(&pWifi->uartHandle,\
    (uint8_t *)(pWifi->pRxWriteBuffer), 1) != HAL_OK) 
  {
    WIFI_ERROR(4);
  }
  pWifi->pRxWriteBuffer++;
  if (pWifi->pRxWriteBuffer == pWifi->pRxReadBuffer)
  {
    // Rx buffer is full 
    WIFI_ERROR(5);
  }
   
  if (character == '\n')
  {
    nbRxBytes = pRxWriteWrap - pWifi->pRxReadBuffer;
    if (nbRxBytes > 2)
    {
      if ((*(pWifi->pRxReadBuffer+1)) == '\n')
      {
        *pWifi->pRxReadBuffer = '\r';
        parseOffset = 2;
      }
#ifdef WIFI_DEBUG
      BSP_UartIfQueueTxData(pWifi->pRxReadBuffer, nbRxBytes);
#endif /* WIFI_DEBUG */
      result = BSP_WifiParseRxBytes(parseOffset, nbRxBytes);
      *c = pWifi->pRxReadBuffer;
      pWifi->pRxReadBuffer += nbRxBytes;
      if (pWifi->pRxReadBuffer >= (pWifi->pRxBuffer + WIFI_UART_RX_BUFFER_SIZE))
      {
        pWifi->pRxReadBuffer = pWifi->pRxBuffer;
      }
      else
      {
        pWifi->pRxReadBuffer -=2;
      }
    }
  }
  
  return result;
}
/******************************************************//**
 * @brief Parse received bytes from WIFI UART 
 * @param offset number of bytes from which to start the parsing
 * @param nbBytes number of bytes to parse
 * @retval result number of bytes to copy to UART debug RX
**********************************************************/
uint8_t BSP_WifiParseRxBytes(uint8_t offset, uint8_t nbBytes)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  uint8_t result = 0;
  /* a result of 0 means there will be no copy to the UART debug RX           */
  /* else bytes will be copied                                                */

  switch (pWifi->mode)
  {
    case WIFI_COMMAND_MODE:
    {
      if (strncasecmp((const char *)pWifi->pRxReadBuffer+offset, "at",2)==0)
      {
        result = nbBytes;
      }
      else if ((strncmp((const char *)pWifi->pRxReadBuffer+offset, "OK",2)==0)||\
          (strncmp((const char *)pWifi->pRxReadBuffer+offset, "ERROR",5)==0))       
      { 
        pWifi->commandPending = 0;
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_FILE_WRITE_START_STRING,\
        strlen(WIFI_UART_FILE_WRITE_START_STRING))==0) 
      {
        pWifi->mode = WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER;
        pWifi->lastTime = HAL_GetTick();
        result = nbBytes;
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_GCODE_START_STRING, strlen(WIFI_UART_GCODE_START_STRING))==0) 
      {
        pWifi->mode = WIFI_COMMAND_MODE_AND_GCODE_PARSER;
        pWifi->lastTime = HAL_GetTick();
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:60:",9)==0)
      {
        pWifi->mode = WIFI_DATA_MODE;
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:32:",9)==0)
      {
        pWifi->restartPending = 0;
        if (pWifi->configPending<0)
        {
          pWifi->configPending = WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS;
        }
      }
      else if ((strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:24:",9)==0)&&(pWifi->configPending==WIFI_CONFIG_FS_UPDATE))
      {
        pWifi->configPending = WIFI_CONFIG_FS_UPDATE_PRINT_VERSION;
      }
#if defined(PROD_TEST)
      else if (pWifi->fwUpdatePending!=0)
      {
        if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          "+WIND:29:DHCP reply for 192.168.0.2",35)==0)
        {
          pWifi->configPending = WIFI_CONFIG_FW_UPDATE;
          pWifi->lastTime = HAL_GetTick();
        }
        else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          " Complete!  Update will be applied on next reboot", 49)==0)
        {
          pWifi->fwUpdatePending = 0;
          pWifi->restartPending = 2;
        }
      }
      else if (pWifi->configPending == WIFI_CONFIG_FS_UPDATE_CHECK)
      {
        if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          "fsversion",9)!=0)
        {
          if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          pWifi->expectedFsVersion,21)!=0)
        {
          pWifi->fsUpdatePending = 1;
        }
        else
        {
          pWifi->fsUpdatePending = 0;
        }
      }
        else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          "ERROR: File not found",21)!=0)
        {
          pWifi->fsUpdatePending = 0;
        }
      }
      else if (pWifi->configPending == WIFI_CONFIG_FS_UPDATE_CONNECT)
      {
        if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          "HTTP/1.1 200 OK",15)==0)
        {
          pWifi->configPending = WIFI_CONFIG_FS_UPDATE_START;
        }
      }
      else if (pWifi->configPending == WIFI_CONFIG_FS_UPDATE_PENDING)
      {
        if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
          " Complete! Please reboot", 23)==0)
        {
          pWifi->fsUpdatePending = 0;
        }
      }
#endif //#if defined(PROD_TEST)
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:2:",8)==0)
      {
        pWifi->commandPending = 0;
        if (pWifi->restartPending != 0)
        {
          pWifi->restartPending = 2;
        }
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:17:P",10)==0)
      {
        pWifi->restartPending = 3;
        pWifi->configPending = -1;
      }
#if defined(PROD_TEST)
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:1:",8)==0)
      {
        strncpy(pWifi->currentFwVersion,(const char *)pWifi->pRxReadBuffer+offset+17,14);
        if (strcmp(pWifi->currentFwVersion, pWifi->expectedFwVersion)!=0)
        {
          pWifi->fwUpdatePending = 1;
        }
      }
#endif //#if defined(PROD_TEST)
    }
    break;
    case WIFI_DATA_MODE_AND_GCODE_PARSER:
    {
      result = nbBytes;
    }
    case WIFI_DATA_MODE:
    {
      if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_GCODE_START_STRING, strlen(WIFI_UART_GCODE_START_STRING))\
        ==0)
      {
        pWifi->mode = WIFI_DATA_MODE_AND_GCODE_PARSER;
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_GCODE_STOP_STRING, strlen(WIFI_UART_GCODE_STOP_STRING))==0)
      {
        pWifi->mode = WIFI_DATA_MODE;
      }
      else if ((strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:62:",9)==0)||\
        (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:59:",9)==0))
      {
        pWifi->mode = WIFI_COMMAND_MODE;
      }
    }
    break;
    case WIFI_FILE_APPEND:
    { 
      if (strncmp((const char *)pWifi->pRxReadBuffer+offset, "OK",2)==0)
      {
        pWifi->commandPending = 0;
        pWifi->mode = WIFI_COMMAND_MODE;
      }
    }
    break;
    case WIFI_COMMAND_MODE_AND_GCODE_PARSER:
    {
      if (strncmp((const char *)pWifi->pRxReadBuffer+offset, "M20",3)==0)
      {
        pWifi->fileCreationPending = 1;
      }
    }
    case WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER:    
    {
      uint8_t stopStringLength = strlen(WIFI_UART_FILE_WRITE_STOP_STRING);
      if ((strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_FILE_WRITE_STOP_STRING,\
        stopStringLength)==0)||\
        (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_FILE_WRITE_STOP_STRING_1,\
        stopStringLength+2)==0)||\
        (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        WIFI_UART_GCODE_STOP_STRING,\
        strlen(WIFI_UART_GCODE_STOP_STRING))==0))
      {
        if (pWifi->fileCreationPending==0) 
        {
          pWifi->mode = WIFI_COMMAND_MODE;
        }
        else
        {
          pWifi->configPending = WIFI_FILE_CREATION_PENDING;
        }
        previousString[0]='\0';
      }
      else if ((strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:56:",9)!=0)&&\
        (strncmp((const char *)pWifi->pRxReadBuffer+offset, "ERROR",5)!=0))
      {
        /* In WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER mode,             */
        /* The bytes coming from the WIFI module have to be sent back to it */
        /* if they are not an ERROR and not an Insert message indicator.    */
        /* The ERROR comes in when the bytes received from the WIFI module  */
        /* are looped back before the WIFI module had time to run input.cgi */
        /* script                                                           */
        /* In WIFI_COMMAND_MODE_AND_GCODE_PARSER mode,                      */
        /* The bytes are not looped back but they have to be parsed by the  */
        /* the Gcode parser and in case there is a reply from the platform, */
        /* this latter one is sent to the WIFI module                       */
        char *ptr;
        uint32_t tmp = 0;
        ptr = strstr((char *)pWifi->pRxReadBuffer, "%20");
        tmp = ptr+1-(char *)pWifi->pRxReadBuffer;
        result = nbBytes;
        /* The bytes are parsed to replace "%20" by a white space character.*/
        /* Contiguous "%20" strings are replaced by a single white space    */
        /* character.                                                       */
        if ((ptr != NULL)&&(tmp < result))
        {
          while ((ptr != NULL)&&(tmp < result))
          {
            *ptr = ' ';
            uint32_t esp = 3;
            while (strncmp(ptr+esp,"%20",3)==0)
            {
              esp+=3;
            }
            ptr++;
            result -= (esp-1);
            strncpy(ptr, ptr+esp-1, result - tmp);
            ptr = strstr((char *)pWifi->pRxReadBuffer+2, "%20");
            tmp = ptr+1-(char *)pWifi->pRxReadBuffer;
          }
          pWifi->pRxReadBuffer[result]='\0';
        }
        pWifi->lastTime = HAL_GetTick();
      }
      else if (strncmp((const char *)pWifi->pRxReadBuffer+offset,\
        "+WIND:56:",9)==0) 
      {
        previousString[0]='\0';
      }
    }
    break;
    default:
      WIFI_ERROR(10);    
  }
  
  return result;
}

/******************************************************//**
 * @brief Parse transmitted bytes to WIFI UART 
 * @param pBuffer pointer to the bytes buffer to parse
 * @param nbBytes number of bytes to parse
 * @param source from where the function call comes
 * @retval 
**********************************************************/
uint8_t BSP_WifiParseTxBytes(const char* pBuffer, uint16_t nbTxBytes,\
  uint8_t source)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  uint8_t result = 0;

  if (nbTxBytes == 0) return BSP_WIFI_UART_BYTES_TO_GCODE_PARSER;
  pWifi->state = BSP_WIFI_UART_BYTES_NONE;
  switch (pWifi->mode)
  {
    case WIFI_COMMAND_MODE_AND_GCODE_PARSER:
    {
        if (source == BSP_WIFI_SOURCE_IS_PLATFORM)
  {
          if (pWifi->fileCreationPending==0)
  {
            /* This code handles the read of the platform reply by the WIFI   */
            /* module input.cgi script                                        */
            /* The reply is stored in an intermediate buffer until the        */
            /* running of the input.cgi script has been detected              */
            strncpy(previousString, pBuffer, nbTxBytes);
            /* As stated in SPWF01Sx – Dynamic Web Pages application note,    */
            /* the message terminator must be <CR>.                           */
            previousString[nbTxBytes]='\r';
            previousString[nbTxBytes+1]='\0';
          }
          else if (\
            BSP_WifiCreateFileInWifiModuleRam("gcf_list.html",nbTxBytes)==\
            BSP_WIFI_FILE_CREATION_OK)
    {
            pWifi->fileCreationPending = 0;
            strncpy((char*)pWifi->pTxWriteBuffer, pBuffer, nbTxBytes);
            pWifi->pTxWriteBuffer[nbTxBytes] = '\0';
            pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
    }
    else
    {
            WIFI_ERROR(11);
    }
          break;
  }
        else if (strncmp((const char *)pBuffer, "\r\n\r\n",4)==0)
        /* This is what can be seen on WIFI uart when a message is awaited by */
        /* the input.cgi script                                               */
  {
          if (previousString[0]!='\0')
          {
            pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
            strcpy((char*)pWifi->pTxWriteBuffer, previousString);
            previousString[0]='\0';
            break;
          }
        }  
        else if (strncasecmp(pBuffer,"at",2) != 0)
        {
          pWifi->state = BSP_WIFI_UART_BYTES_TO_GCODE_PARSER;
          break;
        }
  }
    case WIFI_COMMAND_MODE:
      {
        if (strncasecmp(pBuffer,"at",2) == 0)
  {
          strncpy((char*)pWifi->pTxWriteBuffer, pBuffer, nbTxBytes);
          pWifi->pTxWriteBuffer[nbTxBytes] = '\0';    
    if (pWifi->commandPending == 0)
    {
      pWifi->commandPending = 1;
    }
          else
    {
            /* AT command has not been acknowledged by the WIFI module either */
            /* with an OK, an ERROR, or a +WIND:2: response */
      WIFI_ERROR(7);
    }    
          pBuffer += 2;
          if (strncasecmp(pBuffer,"+CFUN=",6)==0)
    {
      pWifi->restartPending = 1;
    }
          else if (strncasecmp(pBuffer,"+S.FSA=/",8)==0)
    {
      pWifi->mode = WIFI_FILE_APPEND;
    }
          pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        }
        else
        {
          pWifi->state = BSP_WIFI_UART_BYTES_TO_GCODE_PARSER;
        }
      }
      break;
    case WIFI_DATA_MODE_AND_GCODE_PARSER:
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI_AND_GCODE_PARSER;
      }
    case WIFI_DATA_MODE:
      {
        if (strncasecmp(pBuffer,"AT+S.\r",6) == 0)
        {
          strcpy((char*)pWifi->pTxWriteBuffer, "at+s.\0");
        }
        else
        {
          strncpy((char*)pWifi->pTxWriteBuffer, pBuffer, nbTxBytes);
          pWifi->pTxWriteBuffer[nbTxBytes++] = '\n';
          pWifi->pTxWriteBuffer[nbTxBytes] = '\0';
        }
        pWifi->state |= BSP_WIFI_UART_BYTES_TO_WIFI;
      }
      break;
    case WIFI_FILE_APPEND:
      {
        strncpy((char*)pWifi->pTxWriteBuffer, pBuffer, nbTxBytes);
        pWifi->pTxWriteBuffer[nbTxBytes] = '\0';
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
      }
      break;
    case WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER:
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_GCODE_PARSER;
        if (source != BSP_WIFI_SOURCE_IS_PLATFORM)
        { 
          if (strncmp((const char *)pBuffer, "\r\n\r\n",4)==0)
          /* this is what can be seen on WIFI uart when a message is awaited */
          {
            if (previousString[0]!='\0')
            {
              pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
              strcpy((char*)pWifi->pTxWriteBuffer, previousString);
              previousString[0]='\0';
            }
          }
          else if (previousString[0]=='\0')
          {
            if (strncmp((const char *)pBuffer, "\r\n",2)==0)
            /* this is what has been received from the WIFI uart */
            {
              strncpy(previousString, pBuffer+2, nbTxBytes-2);
              previousString[nbTxBytes-2] = '\0';                        
            }
            else
            {
              strncpy((char*)previousString, pBuffer, nbTxBytes);
              previousString[nbTxBytes] = '\0';    
            }
            pWifi->replyCounter = HAL_GetTick();
            if (pWifi->replyCounter > (0xFFFFFFFF - WIFI_INPUT_CGI_DELAY))
            {
              pWifi->replyCounter = WIFI_INPUT_CGI_DELAY -\
               (0xFFFFFFFF - pWifi->replyCounter);
  }
  else
  {
              pWifi->replyCounter += WIFI_INPUT_CGI_DELAY;
            }
          }
        }
      }
      break;
    default:
      WIFI_ERROR(9);
  }
  
  result =  pWifi->state;
  
  BSP_WifiProcessUartBytes();

  return result;
}

/******************************************************//**
 * @brief Send platform reply, AT command, or data
 * from debug uart to Wifi Uart 
 * @param None
 * @retval None
 * @note the platform calls periodically this function either
 * directly or through BSP_WifiParseTxByteswhen if a new line
 * has been received on the UART debug
**********************************************************/
void BSP_WifiProcessUartBytes(void)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  uint16_t nbBytes;
  
  if (pWifi->mode == WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER)
  {
    uint32_t tmp = HAL_GetTick();
    /* Sometimes the WIFI module omits to send a line feed before waiting an  */
    /* input string as part of the input.cgi script execution.                */
    /* To circumvent this issue, the 3D printer sends the previously received */
    /* string after WIFI_INPUT_CGI_DELAY in ms                                */
    if (tmp > pWifi->replyCounter)
    {
      if (pWifi->txFlag == RESET)
      {
        if (previousString[0]!='\0')
        {
          pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
          strcpy((char*)pWifi->pTxWriteBuffer, previousString);
          previousString[0]='\0';
        }
      }
    } 
    /* In case the WIFI_UART_FILE_WRITE_STOP_STRING has not been received for */    
    /* a too long time since the last received string from the WIFI module    */
    /* the WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER is automatically       */
    /* exited                                                                 */
    {
      if (tmp < pWifi->lastTime)
      {
        tmp = tmp + (0xFFFFFFFF - pWifi->lastTime);
      }
      else
      {
        tmp = tmp - pWifi->lastTime;
      }
      if (tmp > WIFI_COMMAND_MODE_WEB_WRITE_TIMEOUT)
      {
        strcpy((char *)pWifi->pRxReadBuffer,\
          "\r\nExiting WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER\r\n");
        BSP_UartIfQueueTxData(pWifi->pRxReadBuffer,\
          strlen("\r\nExiting WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER\r\n"));
        pWifi->pRxReadBuffer += (strlen(\
          "\r\nExiting WIFI_COMMAND_MODE_WEB_FILE_AND_GCODE_PARSER\r\n") - 2);
        if (pWifi->pRxReadBuffer >=\
          (pWifi->pRxBuffer + WIFI_UART_RX_BUFFER_SIZE))
        {
          pWifi->pRxReadBuffer = pWifi->pRxBuffer;
        }
        pWifi->mode = WIFI_COMMAND_MODE;
      }
    }
  }
  
  /* Restart the WIFI module as a response to AT+CFUN command */
  if (pWifi->restartPending >= 2)
  {
    if (pWifi->restartPending == 3)
    {
      HAL_Delay(20000);
    }
    pWifi->restartPending = 0;
    BSP_WifiRestart();
  }

  /* WIFI module configuration using AT commands */
  switch (pWifi->configPending)
  {
    case -1:
    case 0: 
      break;
    case WIFI_FILE_CREATION_PENDING: /* switch to command mode pending */
      if (pWifi->fileCreationPending==0)
      { 
        pWifi->mode = WIFI_COMMAND_MODE;
        pWifi->configPending=0;
      }
      break;
#if defined(PROD_TEST)
    case WIFI_CONFIG_FS_UPDATE_PENDING: /* file system update on-going */
      if ((pWifi->txFlag == RESET) && (pWifi->fsUpdatePending == 0))
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI; 
        pWifi->restartPending = 2;
        pWifi->configPending = 0;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+CFUN=1\r\n");
      }
      break;
    case WIFI_CONFIG_FS_UPDATE_START: /* start file system update */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI; 
        pWifi->configPending=WIFI_CONFIG_FS_UPDATE_PENDING;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.HTTPDFSUPDATE=192.168.0.2,/");
        strcat((char *)(pWifi->pTxWriteBuffer),pWifi->expectedFsVersion);
        strcat((char *)(pWifi->pTxWriteBuffer),".img\r\n");
      }
      break;
    case WIFI_CONFIG_FS_UPDATE_CONNECT: /* check the http server is responding */
      if (pWifi->txFlag == RESET)
      {
        uint32_t tmp = HAL_GetTick();
        if (tmp < pWifi->lastTime)
        {
          tmp = tmp + (0xFFFFFFFF - pWifi->lastTime);
        }
        else
        {
          tmp = tmp - pWifi->lastTime;
        }
        if (tmp>5000)
        {
          pWifi->lastTime = HAL_GetTick();
          pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
          strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.HTTPGET=192.168.0.2,/\r\n");    
        }
      }
      break;
    case WIFI_CONFIG_FS_UPDATE_CHECK: /* */
      if (pWifi->fsUpdatePending == 1)
      {
        pWifi->configPending=WIFI_CONFIG_FS_UPDATE_CONNECT;
      }
      else if (pWifi->fsUpdatePending == 0)
      {
        pWifi->configPending=0;
      }
      break;
    case WIFI_CONFIG_FS_UPDATE_PRINT_VERSION: /* check file system version */
      if ((pWifi->txFlag == RESET) && (pWifi->restartPending == 0))
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending=WIFI_CONFIG_FS_UPDATE_CHECK;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.FSP=/fsversion.txt\r\n");
      }
      break;
    case WIFI_CONFIG_FS_UPDATE:
      break;
    case WIFI_CONFIG_SAVE_CASE-1: /* reset the module */
      if (pWifi->txFlag == RESET)
      {
        HAL_Delay(100); /* Give the wifi module time to save the settings */
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI; 
        pWifi->restartPending = 2;
        pWifi->configPending=WIFI_CONFIG_FS_UPDATE;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+CFUN=1\r\n");
      }
      break;
#else //#if defined(PROD_TEST)
    case WIFI_CONFIG_SAVE_CASE-1: /* reset the module */
      if (pWifi->txFlag == RESET)
      {
        HAL_Delay(100); /* Give the wifi module time to save the settings */
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI; 
        pWifi->restartPending = 2;
        pWifi->configPending = 0;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+CFUN=1\r\n");
      }
      break;      
#endif //#else //#if defined(PROD_TEST)
    case WIFI_CONFIG_SAVE_CASE: /* Save the settings on the flash memory */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT&W\r\n");
      }
      break;
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-13: /* Enable socket server, default is TCP */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SOCKD=32000\r\n");
      }
      break;  
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-12: /* Change the Mini AP default homepage */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--; /* After last item to configure, next is saving */
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=ip_apredirect,axisctrl.shtml\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-11: /* Set the MiniAP address */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=ip_ipaddr,192.168.0.1\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-10: /* Set the use_dhcpmode (0 = DHCP server off, 1 = DHCP server on, 2 = DHCP server on and customizable) */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=ip_use_dhcp,2\r\n");
      }
      break;
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-9: /* Set the network mode (1 = STA, 2 = IBSS, 3 = MiniAP) */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=wifi_mode,3\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-8: /* Set the network privacy mode (0=OPEN or 1=WEP are supported) */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=wifi_priv_mode,1\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-7: /* Set authentication type */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=wifi_auth_type,0\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-6: /* Set the wep key length, 05 or 0D bytes */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        if (strlen(pWifi->wepKey)==10)
        {
          strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=wifi_wep_key_lens,05\r\n");
        }
        else if (strlen(pWifi->wepKey)==26)
        {
          strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=wifi_wep_key_lens,0D\r\n");
        }
        else
        {
          WIFI_ERROR(12);
        }
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-5: /* Set the wep key */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SCFG=wifi_wep_keys[0],");
        strcat((char *)(pWifi->pTxWriteBuffer),pWifi->wepKey);
        strcat((char *)(pWifi->pTxWriteBuffer),"\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-4: /* Set the SSID */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),"AT+S.SSIDTXT=");
        strcat((char *)(pWifi->pTxWriteBuffer),pWifi->ssid);
        strcat((char *)(pWifi->pTxWriteBuffer),"\r\n");
      }
      break;
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-3: /* Wind 0:31 mask */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),\
          "AT+S.SCFG=wind_off_low,0xC8FDFFF9\r\n");
      }
      break;      
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-2: /* Wind 32:63 mask */      
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),\
          "AT+S.SCFG=wind_off_medium,0xA6FFFFFE\r\n");
      }
      break; 
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS-1: /* Wind 64:95 mask */      
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),\
          "AT+S.SCFG=wind_off_high,0xFFFFFFFF\r\n");
      }
      break;
    case WIFI_CONFIG_SAVE_CASE+WIFI_CONFIG_ITEMS: /* Hardware flow control on */
      if (pWifi->txFlag == RESET)
      {
        pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
        pWifi->configPending--;
        strcpy((char *)(pWifi->pTxWriteBuffer),\
          "AT+S.SCFG=console1_hwfc,1\r\n");
      }
      break;
#if defined(PROD_TEST)
    case WIFI_CONFIG_FW_UPDATE: /* Firmware update */
      if (pWifi->txFlag == RESET)
      {
        uint32_t tmp = HAL_GetTick();
        if (tmp < pWifi->lastTime)
        {
          tmp = tmp + (0xFFFFFFFF - pWifi->lastTime);
        }
        else
        {
          tmp = tmp - pWifi->lastTime;
        }
        if (tmp>1000)
        {
          pWifi->state = BSP_WIFI_UART_BYTES_TO_WIFI;
          pWifi->configPending = 0;
          strcpy((char *)(pWifi->pTxWriteBuffer),\
            "AT+S.FWUPDATE=192.168.0.2,/SPWF01S-150410-c2e37a3-RELEASE-main.ota\r\n");
        }
      }
      break;
#endif //#if defined(PROD_TEST)      
    default:
      WIFI_ERROR(8);
  } 
  
  if (((pWifi->state & BSP_WIFI_UART_BYTES_TO_WIFI)\
    == BSP_WIFI_UART_BYTES_TO_WIFI)&&(pWifi->txFlag == RESET))
  {
    nbBytes = strlen((const char*)pWifi->pTxWriteBuffer);
    if (nbBytes != 0)
  {
    pWifi->txFlag = SET;
      /* State is change here to prevent the triggering of a transmit in case   */
      /* no new one has been ordered at the time the tx flag is reset           */
      pWifi->state = BSP_WIFI_UART_BYTES_NONE;
      if(HAL_UART_Transmit_DMA(&(pWifi->uartHandle),\
        (uint8_t *)(pWifi->pTxWriteBuffer), nbBytes)!= HAL_OK)
    {
      WIFI_ERROR(6);
    }
      pWifi->pTxWriteBuffer += nbBytes;
      if (pWifi->pTxWriteBuffer >= pWifi->pTxBuffer + WIFI_UART_TX_BUFFER_SIZE)
      {
        pWifi->pTxWriteBuffer = pWifi->pTxBuffer;
      }
    if (pWifi->mode == WIFI_FILE_APPEND)
    {
      while (pWifi->txFlag != RESET);
    }
  }
  }
 }

 /******************************************************//**
 * @brief Restart the WIFI module by bringing down and high the reset pin
 * @param None
 * @retval None
 **********************************************************/
void BSP_WifiRestart(void)
{
  HAL_Delay(100);
  HAL_GPIO_WritePin(BSP_WIFI_RESET_PORT, BSP_WIFI_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BSP_WIFI_RESET_PORT, BSP_WIFI_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
}        
 
 /******************************************************//**
 * @brief Create a file in the wifi module RAM and open it in append mode
 * @param fileName file name
 * @param fileSizeWithoutHttpHeader file size without http header
 * @retval BSP_WIFI_FILE_CREATION_OK, BSP_WIFI_FILE_SIZE_TOO_BIG or
 * BSP_WIFI_FILE_NAME_TOO_LONG
 **********************************************************/
uint8_t BSP_WifiCreateFileInWifiModuleRam(const char* fileName,\
  uint16_t fileSizeWithoutHttpHeader)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  char s[10+WIFI_FILE_NAME_LENGTH+8] = "AT+S.FSD=/";
  char configFileTop[] = "HTTP/1.0 200 OK\r\nServer: MyProduct\r\nConnection: close\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n";
  char* ptr;
  uint16_t fileSize = strlen(configFileTop)+fileSizeWithoutHttpHeader;
  
  if (strlen(fileName) > WIFI_FILE_NAME_LENGTH)
    return BSP_WIFI_FILE_NAME_TOO_LONG;
  if (fileSize > WIFI_FILE_MAX_SIZE)
    return BSP_WIFI_FILE_SIZE_TOO_BIG;
  if (pWifi->mode == WIFI_DATA_MODE_AND_GCODE_PARSER)
  {
    /* Escape from data mode */
    pWifi->mode = WIFI_DATA_MODE;
    BSP_WifiParseTxBytes("AT+S.\r", strlen("AT+S.\r"), BSP_WIFI_SOURCE_IS_PLATFORM);
    while (pWifi->mode!=WIFI_COMMAND_MODE);
  }
  else
  {
    pWifi->mode = WIFI_COMMAND_MODE;
  }
  /* Delete the file in the WIFI module RAM if any */
  strcat(s, fileName);
  strcat(s,"\r\n");
  BSP_WifiParseTxBytes(s, strlen(s), BSP_WIFI_SOURCE_IS_PLATFORM);
  while (pWifi->commandPending!=0);
  /* Create the file in the WIFI module RAM */
  strcpy(s, "AT+S.FSC=/");
  strcat(s, fileName);
  ptr = s + strlen(s);
  sprintf(ptr, ",%d\r\n",WIFI_FILE_MAX_SIZE);
  BSP_WifiParseTxBytes(s, strlen(s), BSP_WIFI_SOURCE_IS_PLATFORM);
  while (pWifi->commandPending!=0)
  /* Append the file in the WIFI module RAM */
  strcpy(s, "AT+S.FSA=/");
  strcat(s, fileName);
  ptr = s + strlen(s);
  sprintf(ptr, ",%d\r\n",fileSize);
  BSP_WifiParseTxBytes(s, strlen(s), BSP_WIFI_SOURCE_IS_PLATFORM);
  BSP_WifiParseTxBytes(configFileTop, strlen(configFileTop),\
    BSP_WIFI_SOURCE_IS_PLATFORM);
  
  return BSP_WIFI_FILE_CREATION_OK;
}

 /******************************************************//**
 * @brief Create a file in the wifi module RAM and open it in append mode
 * @param fileName file name
 * @param fileSizeWithoutHttpHeader file size without http header
 * @retval BSP_WIFI_FILE_CREATION_OK, BSP_WIFI_FILE_SIZE_TOO_BIG or
 * BSP_WIFI_FILE_NAME_TOO_LONG
 **********************************************************/
uint8_t BSP_WifiIsFileCreation(void)
{
  BspWifiDataType *pWifi = &gBspWifiData;
  return pWifi->fileCreationPending;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

