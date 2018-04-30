/**
  ******************************************************************************
  * @file    stm32f0xx_3dPrinter_sd.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    March 02, 2015
  * @brief   SD functions of 3D Printer BSP driver 
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
#include "stm32f0xx_3dprinter_sd.h"

/* Private defines ----------------------------------------------------------*/
/* Private constant ----------------------------------------------------------*/
#define SD_ERROR_TAG        (0x3000)
#define SD_ERROR(error)     BSP_MiscErrorHandler(error|SD_ERROR_TAG)
    
/* Global variables ---------------------------------------------------------*/


/* Private variables */
static BspSdDataType gBspSdData;

/* Private function -----------------------------------------------------------*/
static void SD_MspInit(void);

/******************************************************//**
 * @brief  SD card Hw initialisation
 * @param None
 * @retval  SD status
 **********************************************************/
uint8_t BSP_SD_Init(void)
{ 
  BspSdDataType *pSd = &gBspSdData;
  
  uint8_t SD_state = MSD_OK;


  /* uSD device interface configuration */
  pSd->uSdHandle.Instance = SDIO;

  pSd->uSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  pSd->uSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  pSd->uSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  pSd->uSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  pSd->uSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  pSd->uSdHandle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;
  
  /* Configure  SD detect pin */
  //Done only once at startup, using DetectInit method.

  /* Check if the SD card is plugged in the slot */
  if(BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR;
  }
  
  /* HAL SD initialization */
  SD_MspInit();

  if(HAL_SD_Init(&(pSd->uSdHandle), &(pSd->uSdCardInfo)) != SD_OK)
  {
    SD_state = MSD_ERROR;
  }
#if !defined(NO_SD_WIDE_BUS)
  /* Configure SD Bus width */
  if(SD_state == MSD_OK)
  {
    /* Enable wide operation */
    if(HAL_SD_WideBusOperation_Config(&(pSd->uSdHandle), SDIO_BUS_WIDE_4B) != SD_OK)
    {
      SD_state = MSD_ERROR;
    }
    else
    {
      SD_state = MSD_OK;
    }
  }
#endif
  
  return  SD_state;
}

/******************************************************//**
 * @brief  SD card Hw de-initialisation
 * @param None
 * @retval  SD status
 **********************************************************/
uint8_t BSP_SD_DeInit(void)
{
	BspSdDataType *pSd = &gBspSdData;
	uint8_t SD_state = MSD_OK;

	if( HAL_SD_DeInit(&(pSd->uSdHandle)) != HAL_OK)
	{
		SD_state = MSD_ERROR;
	}

	return SD_state;

}



/******************************************************//**
 * @brief  Configures and enable Interrupt mode for SD detection pin.
 * @param None
 * @retval Returns 0
 **********************************************************/
void BSP_SD_DetectInit(void)
{  
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO clock */
  __GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = BSP_SD_DETECT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BSP_SD_DETECT_PORT, &GPIO_InitStruct);

  /* Set Priority of External Line Interrupt used for the SD detect interrupt*/
  HAL_NVIC_SetPriority(BSP_SD_DETECT_IRQn, BSP_SD_DETECT_PRIORITY, 0);

  /* Enable the External Line Interrupt used for the SD detect  interrupt*/
  HAL_NVIC_EnableIRQ(BSP_SD_DETECT_IRQn);
}


/******************************************************//**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param None
 * @retval Returns if SD is detected or not
 **********************************************************/
uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;
  
  /* Check SD card detect pin */
  if(HAL_GPIO_ReadPin(BSP_SD_DETECT_PORT, BSP_SD_DETECT_PIN) == GPIO_PIN_RESET)
  {
    status = SD_NOT_PRESENT;
  }
  
  return status;
}

/******************************************************//**
 * @brief  SD detect IT treatment
 * @param None
 * @retval None
 **********************************************************/
void BSP_SD_DetectIT(void)
{
  /* SD detect IT callback */
  BSP_SD_DetectCallback();
}

/******************************************************//**
 * @brief  SD detect IT detection callback
 * @param None
 * @retval None
 **********************************************************/
__weak void BSP_SD_DetectCallback(void)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
     the BSP_SD_DetectCallback could be implemented in the user file
  */ 
}

/******************************************************//**
 * @brief  SD detect IT detection callback
 * @param[in]  pData: Pointer to the buffer that will contain the data to transmit
 * @param[in]  ReadAddr: Address from where data is to be read  
 * @param[in]  BlockSize: SD card data block size, that should be 512
 * @param[in]  NumOfBlocks: Number of SD blocks to read 
 * @retval SD status
 **********************************************************/
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  BspSdDataType *pSd = &gBspSdData;
  
  if(HAL_SD_ReadBlocks(&(pSd->uSdHandle), pData, ReadAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/******************************************************//**
 * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
 * @param[in]  pData: Pointer to the buffer that will contain the data to transmit
 * @param[in]  WriteAddr: Address from where data is to be written  
 * @param[in]  BlockSize: SD card data block size, that should be 512
 * @param[in]  NumOfBlocks: Number of SD blocks to write
 * @retval SD status
 **********************************************************/
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  BspSdDataType *pSd = &gBspSdData;
  
  if(HAL_SD_WriteBlocks(&(pSd->uSdHandle), pData, WriteAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/******************************************************//**
 * @brief Reads block(s) from a specified address in an SD card, in DMA mode.
 * @param[in]  pData: Pointer to the buffer that will contain the data to transmit
 * @param[in]  ReadAddr: Address from where data is to be read  
 * @param[in]  BlockSize: SD card data block size, that should be 512
 * @param[in]  NumOfBlocks: Number of SD blocks to read 
 * @retval SD status
 **********************************************************/
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  BspSdDataType *pSd = &gBspSdData;
  uint8_t SD_state = MSD_OK;
  
  /* Read block(s) in DMA transfer mode */
  if(HAL_SD_ReadBlocks_DMA(&(pSd->uSdHandle), pData, ReadAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    SD_state = MSD_ERROR;
  }
  
  /* Wait until transfer is complete */
  if(SD_state == MSD_OK)
  {
    if(HAL_SD_CheckReadOperation(&(pSd->uSdHandle), (uint32_t)SD_DATATIMEOUT) != SD_OK)
    {
      SD_state = MSD_ERROR;
    }
    else
    {
      SD_state = MSD_OK;
    }
  }
  
  return SD_state; 
}

/******************************************************//**
 * @brief Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param[in]  pData: Pointer to the buffer that will contain the data to transmit
  * @param[in]  WriteAddr: Address from where data is to be written  
  * @param[in]  BlockSize: SD card data block size, that should be 512
  * @param[in]  NumOfBlocks: Number of SD blocks to write 
 * @retval SD status
 **********************************************************/
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  BspSdDataType *pSd = &gBspSdData;  
  uint8_t SD_state = MSD_OK;
  
  /* Write block(s) in DMA transfer mode */
  if(HAL_SD_WriteBlocks_DMA(&(pSd->uSdHandle), pData, WriteAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    SD_state = MSD_ERROR;
  }
  
  /* Wait until transfer is complete */
  if(SD_state == MSD_OK)
  {
    if(HAL_SD_CheckWriteOperation(&(pSd->uSdHandle), (uint32_t)SD_DATATIMEOUT) != SD_OK)
    {
      SD_state = MSD_ERROR;
    }
    else
    {
      SD_state = MSD_OK;
    }
  }
  
  return SD_state;  
}

/******************************************************//**
 * @brief Erases the specified memory area of the given SD card. 
 * @param[in]  StartAddr: Start byte address
 * @param[in]  EndAddr: End byte address 
 * @retval SD status
 **********************************************************/
uint8_t BSP_SD_Erase(uint64_t StartAddr, uint64_t EndAddr)
{
  BspSdDataType *pSd = &gBspSdData; 
  if(HAL_SD_Erase(&(pSd->uSdHandle), StartAddr, EndAddr) != SD_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/******************************************************//**
 * @brief Initializes the SD MSP. 
 * @param  None
 * @retval SD None
 **********************************************************/
static void SD_MspInit(void)
{
  BspSdDataType *pSd = &gBspSdData;
  static DMA_HandleTypeDef dmaRxHandle;
  static DMA_HandleTypeDef dmaTxHandle;
  GPIO_InitTypeDef GPIO_Init_Structure;
  SD_HandleTypeDef *hsd = &(pSd->uSdHandle);
  
  /* Enable SDIO clock */
  __SDIO_CLK_ENABLE();
  
  /* Enable DMA2 clocks */
  __BSP_BSP_SD_DMAx_TxRx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  
  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_SDIO;
  
  /* GPIOC configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
   
  HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);

  /* GPIOD configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* NVIC configuration for SDIO interrupts */
  HAL_NVIC_SetPriority(SDIO_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);
    
  /* Configure DMA Rx parameters */
  dmaRxHandle.Init.Channel             = BSP_SD_DMAx_Rx_CHANNEL;
  dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaRxHandle.Init.Mode                = DMA_PFCTRL;
  dmaRxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  dmaRxHandle.Instance = BSP_SD_DMAx_Rx_STREAM;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, dmaRxHandle);
  
  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dmaRxHandle);
  
  /* Configure the DMA stream */
  HAL_DMA_Init(&dmaRxHandle);
  
  /* Configure DMA Tx parameters */
  dmaTxHandle.Init.Channel             = BSP_SD_DMAx_Tx_CHANNEL;
  dmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  dmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaTxHandle.Init.Mode                = DMA_PFCTRL;
  dmaTxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaTxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaTxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaTxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaTxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  dmaTxHandle.Instance = BSP_SD_DMAx_Tx_STREAM;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, dmaTxHandle);
  
  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dmaTxHandle);
  
  /* Configure the DMA stream */
  HAL_DMA_Init(&dmaTxHandle); 
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(BSP_SD_DMAx_Rx_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(BSP_SD_DMAx_Rx_IRQn);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(BSP_SD_DMAx_Tx_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(BSP_SD_DMAx_Tx_IRQn);
}

/******************************************************//**
 * @brief Handles SD card interrupt request.
 * @param  None
 * @retval SD None
 **********************************************************/
void BSP_SD_IRQHandler(void)
{
  BspSdDataType *pSd = &gBspSdData;
  HAL_SD_IRQHandler(&(pSd->uSdHandle));
}

/******************************************************//**
 * @brief Handles SD DMA Tx transfer interrupt request.
 * @param  None
 * @retval SD None
 **********************************************************/
void BSP_SD_DMA_Tx_IRQHandler(void)
{
  BspSdDataType *pSd = &gBspSdData;
  HAL_DMA_IRQHandler(pSd->uSdHandle.hdmatx); 
}

/******************************************************//**
 * @brief  Handles SD DMA Rx transfer interrupt request.
 * @param  None
 * @retval SD None
 **********************************************************/
void BSP_SD_DMA_Rx_IRQHandler(void)
{
  BspSdDataType *pSd = &gBspSdData;
  HAL_DMA_IRQHandler(pSd->uSdHandle.hdmarx);
}

/******************************************************//**
 * @brief  Gets the current SD card data status.
 * @param  None
 * @retval Data transfer state.
 *          This value can be one of the following values:
 *            @arg  SD_TRANSFER_OK: No data transfer is acting
 *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
 *            @arg  SD_TRANSFER_ERROR: Data transfer error 
 **********************************************************/
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void)
{
  BspSdDataType *pSd = &gBspSdData;
  return(HAL_SD_GetStatus(&(pSd->uSdHandle)));
}

/******************************************************//**
 * @brief  Get SD information about specific SD card.
 * @param[in]  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
 * @retval None 
 **********************************************************/
void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef *CardInfo)
{
  BspSdDataType *pSd = &gBspSdData;
  /* Get SD card Information */
  HAL_SD_Get_CardInfo(&(pSd->uSdHandle), CardInfo);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

