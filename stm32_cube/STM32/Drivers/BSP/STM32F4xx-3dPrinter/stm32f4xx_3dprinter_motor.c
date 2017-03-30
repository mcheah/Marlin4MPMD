/**
  ******************************************************************************
  * @file    stm32f4xx_3dPrinter_motor.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   motor functions of 3D Printer BSP driver 
  *  (based on L6474)
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
#include "l6474.h"
#include "stm32f4xx_3dprinter_motor.h"
#include "motorcontrol.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32F4XX_3DPRINTER_MOTOR
  * @{
  */   
    
/* Private constants ---------------------------------------------------------*/    

/** @defgroup STM32F4XX_3DPRINTER_MOTOR_Private_Constants
  * @{
  */   
    
/// Timer Prescaler
#define TIMER_PRESCALER (1024)

/// SPI Maximum Timeout values for flags waiting loops
#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)

/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32F4XX_3DPRINTER_MOTOR_Board_Private_Variables
  * @{
  */       
/// SPI handler declaration
static SPI_HandleTypeDef SpiHandle;
/// Timer handler for PWMX
TIM_HandleTypeDef hTimPwmX;
/// Timer handler for PWMY
TIM_HandleTypeDef hTimPwmY;
/// Timer handler for PWMZ
TIM_HandleTypeDef hTimPwmZ;
/// Timer handler for PWME1
TIM_HandleTypeDef hTimPwmE1;
/// Timer handler for PWME2
TIM_HandleTypeDef hTimPwmE2;
/// Timer handler for PWME3
TIM_HandleTypeDef hTimPwmE3;

/**
  * @}
  */ 

/** @defgroup STM32F4XX_3DPRINTER_MOTOR_Board_Private_Function_Prototypes
  * @{
  */   
   
void BSP_MotorControlBoard_Delay(uint32_t delay);         //Delay of the requested number of milliseconds
void BSP_MotorControlBoard_DisableIrq(void);              //Disable Irq
void BSP_MotorControlBoard_EnableIrq(void);               //Enable Irq
void BSP_MotorControlBoard_GpioInit(uint8_t nbDevices);   //Initialise GPIOs used for L6474s
void BSP_MotorControlBoard_PwmXSetFreq(uint16_t newFreq); //Set PWM_X frequency and start it
void BSP_MotorControlBoard_PwmYSetFreq(uint16_t newFreq); //Set PWM_Y frequency and start it  
void BSP_MotorControlBoard_PwmZSetFreq(uint16_t newFreq); //Set PWM_Z frequency and start it
void BSP_MotorControlBoard_PwmE1SetFreq(uint16_t newFreq); //Set PWM_E1 frequency and start it
void BSP_MotorControlBoard_PwmE2SetFreq(uint16_t newFreq); //Set PWM_E2 frequency and start it
void BSP_MotorControlBoard_PwmE3SetFreq(uint16_t newFreq); //Set PWM_E3 frequency and start it
void BSP_MotorControlBoard_PwmE4SetFreq(uint16_t newFreq); //Set PWM_E4 frequency and start it
void BSP_MotorControlBoard_PwmInit(uint8_t deviceId);    //Init the PWM of the specified device
void BSP_MotorControlBoard_PwmStop(uint8_t deviceId);    //Stop the PWM of the specified device
void BSP_MotorControlBoard_ReleaseReset(void);           //Reset the L6474 reset pin 
void BSP_MotorControlBoard_Reset(void);                  //Set the L6474 reset pin 
void BSP_MotorControlBoard_SetDirectionGpio(uint8_t deviceId, uint8_t gpioState); //Set direction GPIO
uint8_t BSP_MotorControlBoard_SpiInit(void);   //Initialise the SPI used for L6474s
uint8_t BSP_MotorControlBoard_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices); //Write bytes to the L6474s via SPI

/**
  * @}
  */


/** @defgroup  STM32F4XX_3DPRINTER_MOTOR_Board_Private_Functions
  * @{
  */   

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_Delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/******************************************************//**
 * @brief This function disable the interruptions
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_DisableIrq(void)
{
  __disable_irq();
}

/******************************************************//**
 * @brief This function enable the interruptions
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_EnableIrq(void)
{
  __enable_irq();
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the L6474s
 * @param[in] nbDevices number of L6474 devices
 * @retval None
  **********************************************************/
void BSP_MotorControlBoard_GpioInit(uint8_t nbDevices)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE(); 
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();

  /* Configure L6474 - DIR pin for device 0 -------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_X_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_X_PORT, &GPIO_InitStruct);
  
  /* Configure L6474 - Flag pin -------------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_FLAG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_FLAG_PORT, &GPIO_InitStruct);
  
 /* Set Priority of External Line Interrupt used for the Flag interrupt*/ 
  HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_FLAG_IRQn, 7, 0);
    
  /* Enable the External Line Interrupt used for the Flag interrupt*/
  HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_FLAG_IRQn);    

  /* Configure L6474 - CS pin ---------------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_CS_PORT, BSP_MOTOR_CONTROL_BOARD_CS_PIN, GPIO_PIN_SET); 
  
  /* Configure L6474 - STBY/RESET pin -------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_X_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_X_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_Y_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_Y_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_Z_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_Z_PORT, &GPIO_InitStruct);  

  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_E1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_E1_PORT, &GPIO_InitStruct);    

  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_E2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_E2_PORT, &GPIO_InitStruct);    
  
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_RESET_E3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_RESET_E3_PORT, &GPIO_InitStruct);   

  BSP_MotorControlBoard_Reset();  

  if (nbDevices > 1) 
  {
    /* Configure L6474 - DIR pin for device  1 ----------------------------*/
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_Y_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_Y_PORT, &GPIO_InitStruct);    
  }
  if (nbDevices > 2) 
  {
    /* Configure L6474 - DIR pin for device  2 ----------------------------*/
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_Z_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_Z_PORT, &GPIO_InitStruct);    
  }  
  if (nbDevices > 3) 
  {
    /* Configure L6474 - DIR pin for device  3 ----------------------------*/
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_E1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_E1_PORT, &GPIO_InitStruct);    
  }  
    if (nbDevices > 4) 
  {
    /* Configure L6474 - DIR pin for device  4 ----------------------------*/
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_E2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_E2_PORT, &GPIO_InitStruct);    
  }  
  if (nbDevices > 5) 
  {
    /* Configure L6474 - DIR pin for device  5 ----------------------------*/
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_DIR_E3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_DIR_E3_PORT, &GPIO_InitStruct);    
  }  
}

/******************************************************//**
 * @brief  Sets the frequency of PWM_X used by device 0
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmXSetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X_FREQ_RESCALER * (uint32_t)newFreq)) - 1;
  
  __HAL_TIM_SetAutoreload(&hTimPwmX, period);
  __HAL_TIM_SetCompare(&hTimPwmX, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_X, period >> 1);
  HAL_TIM_PWM_Start_IT(&hTimPwmX, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_X);  
}

/******************************************************//**
 * @brief  Sets the frequency of PWM_Y used by device 1
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmYSetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y_FREQ_RESCALER  * (uint32_t)newFreq)) - 1;
  
  __HAL_TIM_SetAutoreload(&hTimPwmY, period);
  __HAL_TIM_SetCompare(&hTimPwmY, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Y, period >> 1);
  HAL_TIM_PWM_Start_IT(&hTimPwmY, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Y);
}
/******************************************************//**
 * @brief  Sets the frequency of PWM_Z used by device 2
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmZSetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z_FREQ_RESCALER * (uint32_t)newFreq)) - 1;
  
  __HAL_TIM_SetAutoreload(&hTimPwmZ, period);
  __HAL_TIM_SetCompare(&hTimPwmZ, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Z, period >> 1);
  HAL_TIM_PWM_Start_IT(&hTimPwmZ, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Z);  
}
/******************************************************//**
 * @brief  Sets the frequency of PWM_E1 used by device 3
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmE1SetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1_FREQ_RESCALER * (uint32_t)newFreq)) - 1;
  
  __HAL_TIM_SetAutoreload(&hTimPwmE1, period);
  __HAL_TIM_SetCompare(&hTimPwmE1, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E1, period >> 1);
  HAL_TIM_PWM_Start_IT(&hTimPwmE1, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E1);  
}
/******************************************************//**
 * @brief  Sets the frequency of PWM_E2 used by device 4
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmE2SetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2_FREQ_RESCALER * (uint32_t)newFreq)) - 1;
  
  __HAL_TIM_SetAutoreload(&hTimPwmE2, period);
  __HAL_TIM_SetCompare(&hTimPwmE2, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E2, period >> 1);
  HAL_TIM_PWM_Start_IT(&hTimPwmE2, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E2);  
}
/******************************************************//**
 * @brief  Sets the frequency of PWM_E3 used by device 5
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControlBoard_PwmE3SetFreq(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3_FREQ_RESCALER * (uint32_t)newFreq)) - 1;
  
  __HAL_TIM_SetAutoreload(&hTimPwmE3, period);
  __HAL_TIM_SetCompare(&hTimPwmE3, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E3, period >> 1);
  HAL_TIM_PWM_Start_IT(&hTimPwmE3, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E3);  
}

/******************************************************//**
 * @brief  Initialises the PWM uses by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 * @note Device 0 uses PWM1 based on timer 1 
 * Device 1 uses PWM 2 based on timer 2
 * Device 2 uses PWM3 based timer 0
 **********************************************************/
void BSP_MotorControlBoard_PwmInit(uint8_t deviceId)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_HandleTypeDef *pHTim;
  uint32_t  channel;

  switch (deviceId)
  {

  case 0:
  default:
      pHTim = &hTimPwmX;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_X;
      break;
    case  1:
      pHTim = &hTimPwmY;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Y;
      break;
    case 2:
      pHTim = &hTimPwmZ;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Z;
      break;
    case 3:
      pHTim = &hTimPwmE1;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E1;
      break;
    case 4:
      pHTim = &hTimPwmE2;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E2;
      break;
    case 5:
      pHTim = &hTimPwmE3;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E3;
      break;
  }
  pHTim->Init.Prescaler = TIMER_PRESCALER -1;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.Period = 0;
  pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(pHTim);
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(pHTim, &sConfigOC, channel);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(pHTim, &sMasterConfig);
}

/******************************************************//**
 * @brief  Stops the PWM uses by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_PwmStop(uint8_t deviceId)
{
  switch (deviceId)
  {
    case 0:
       HAL_TIM_PWM_Stop(&hTimPwmX,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_X);
      break;
    case  1:
      HAL_TIM_PWM_Stop(&hTimPwmY,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Y);
      break;
    case 2:
       HAL_TIM_PWM_Stop(&hTimPwmZ,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_Z);
      break;
    case 3:
       HAL_TIM_PWM_Stop(&hTimPwmE1,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E1);
      break;
    case 4:
       HAL_TIM_PWM_Stop(&hTimPwmE2,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E2);
      break;
    case 5:
       HAL_TIM_PWM_Stop(&hTimPwmE3,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM_E3);
      break;
  default:
      break;//ignore error
  }
}

/******************************************************//**
 * @brief  Releases the L6474 reset (pin set to High) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_ReleaseReset(void)
{ 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_X_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_X_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_Y_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_Y_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_Z_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_Z_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_E1_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_E1_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_E2_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_E2_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_E3_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_E3_PIN, GPIO_PIN_SET);
}

/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_Reset(void)
{
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_X_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_X_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_Y_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_Y_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_Z_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_Z_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_E1_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_E1_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_E2_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_E2_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_RESET_E3_PORT, BSP_MOTOR_CONTROL_BOARD_RESET_E3_PIN, GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief  Set the GPIO used for the direction
 * @param[in] deviceId (from 0 to 6)
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void BSP_MotorControlBoard_SetDirectionGpio(uint8_t deviceId, uint8_t gpioState)
{
  switch (deviceId)
  {
    case 5: 
      HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_E3_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_E3_PIN, (GPIO_PinState)gpioState); 
      break;
    case 4:
      HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_E2_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_E2_PIN, (GPIO_PinState)gpioState); 
      break;
    case 3: 
      HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_E1_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_E1_PIN, (GPIO_PinState)gpioState); 
      break;
    case 2:
      HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_Z_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_Z_PIN, (GPIO_PinState)gpioState);
      break;
    case 1:
      HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_Y_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_Y_PIN, (GPIO_PinState)gpioState); 
      break;
    case 0:
      HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_DIR_X_PORT, BSP_MOTOR_CONTROL_BOARD_DIR_X_PIN, (GPIO_PinState)gpioState);
      break;
    default:
      ;
  }
}

/******************************************************//**
 * @brief  Initialise the SPI used by L6474
 * @param None
 * @retval HAL_OK if SPI transaction is OK, HAL_KO else
 **********************************************************/
uint8_t BSP_MotorControlBoard_SpiInit(void)
{
  HAL_StatusTypeDef status;
  
  /* Initialises the SPI  --------------------------------------------------*/
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; 
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;    
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
  
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
  
  status = HAL_SPI_Init(&SpiHandle);
  
  return (uint8_t) status;
}
/******************************************************//**
 * @brief  Write and read SPI byte to the L6474
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @param[in] nbDevices Number of device in the SPI chain
 * @retval HAL_OK if SPI transaction is OK, HAL_KO else 
 **********************************************************/
uint8_t BSP_MotorControlBoard_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t i;
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_CS_PORT, BSP_MOTOR_CONTROL_BOARD_CS_PIN, GPIO_PIN_RESET); 
  for (i = 0; i < nbDevices; i++)
  {
    status = HAL_SPI_TransmitReceive(&SpiHandle, pByteToTransmit, pReceivedByte, 1, SPIx_TIMEOUT_MAX);
    if (status != HAL_OK)
    {
      break;
    }
    pByteToTransmit++;
    pReceivedByte++;
  }
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_CS_PORT, BSP_MOTOR_CONTROL_BOARD_CS_PIN, GPIO_PIN_SET); 
  
  return (uint8_t) status;  
}


/**
  * @}
  */

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
