/**
  ******************************************************************************
  * @file    stm32f0xx_3dPrinter_misc.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 29, 2015
  * @brief   Miscelleanous functions of 3D Printer BSP driver 
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
#include "stm32f0xx_3dprinter_misc.h"
#include "motorcontrol.h"
#include "stm32f0xx_3dprinter_motor.h"
#include "stm32f0xx_3dprinter_adc.h"
#include "stm32f0xx_3dprinter_uart.h"

#ifdef MOTOR_L6474
#include "l6474.h"
#elif defined(MOTOR_A4985)
#include "A4985.h"
#endif

#include "string.h"
#include <stdio.h>

/* Private defines ----------------------------------------------------------*/
#define HEAT_TIMER_PRESCALER  (1024)
#define SERVO_TIMER_PRESCALER (8)
#define HEAT_TIMER_FREQUENCY  (1000)

/*  global constant ----------------------------------------------------------*/
GPIO_TypeDef* gArrayGpioPort[BSP_MISC_MAX_PIN_NUMBER] = {
  BSP_MOTOR_CONTROL_BOARD_PWM_X_PORT,    //X_STEP_PIN       0       
  BSP_MOTOR_CONTROL_BOARD_DIR_X_PORT,    //X_DIR_PIN        1
  BSP_MOTOR_CONTROL_BOARD_RESET_X_PORT,  //X_ENABLE_PIN    
  0,                       //X_MIN_PIN
  BSP_STOP_X_PORT,                                     //X_MAX_PIN
  BSP_MOTOR_CONTROL_BOARD_PWM_Y_PORT,    //Y_STEP_PIN       5
  BSP_MOTOR_CONTROL_BOARD_DIR_Y_PORT,    //Y_DIR_PIN       
  BSP_MOTOR_CONTROL_BOARD_RESET_Y_PORT,  //Y_ENABLE_PIN    
  0,                       //Y_MIN_PIN
  BSP_STOP_Y_PORT,                                     //Y_MAX_PIN
  BSP_MOTOR_CONTROL_BOARD_PWM_Z_PORT,    //Z_STEP_PIN      10
  BSP_MOTOR_CONTROL_BOARD_DIR_Z_PORT,    //Z_DIR_PIN       
  BSP_MOTOR_CONTROL_BOARD_RESET_Z_PORT,  //Z_ENABLE_PIN    
  BSP_STOP_W_PORT,                       //Z_MIN_PIN
  BSP_STOP_Z_PORT,                                     //Z_MAX_PIN
  0,                                     //Y2_STEP_PIN     15
  0,                                     //Y2_DIR_PIN      
  0,                                     //Y2_ENABLE_PIN   
  0,                                     //Z2_STEP_PIN     
  0,                                     //Z2_DIR_PIN      
  0,                                     //Z2_ENABLE_PIN   20
  BSP_MOTOR_CONTROL_BOARD_PWM_E1_PORT,   //E0_STEP_PIN     
  BSP_MOTOR_CONTROL_BOARD_DIR_E1_PORT,   //E0_DIR_PIN      
  BSP_MOTOR_CONTROL_BOARD_RESET_E1_PORT, //E0_ENABLE_PIN   
  0,//BSP_MOTOR_CONTROL_BOARD_PWM_E2_PORT,   //E1_STEP_PIN
  0,//BSP_MOTOR_CONTROL_BOARD_DIR_E2_PORT,   //E1_DIR_PIN      25
  0,//BSP_MOTOR_CONTROL_BOARD_RESET_E2_PORT, //E1_ENABLE_PIN
  0,                                     //SDPOWER         
  0,                                     //SDSS            
  0,                                     //LED_PIN        
  BSP_FAN_E1_PORT,                       //FAN_PIN         30
  0,                                     //PS_ON_PIN      
  0,                                     //KILL_PIN        
  BSP_HEAT_E1_PORT,                      //HEATER_0_PIN    
//TODO: make this a conditional compile to avoid hard-coding
  0,//BSP_HEAT_E2_PORT,                      //HEATER_1_PIN
  0,//BSP_HEAT_E3_PORT,                      //HEATER_2_PIN    35
  BSP_THERM_E1_PORT,                     //TEMP_0_PIN      
  0,//BSP_THERM_E2_PORT,                     //TEMP_1_PIN
  0,//BSP_THERM_E3_PORT,                     //TEMP_2_PIN
  BSP_HEAT_BED1_PORT,                     //HEATER_BED_PIN  
  BSP_THERM_BED1_PORT,                    //TEMP_BED_PIN    40
  0,//BSP_SERVO0_PORT,                       //SERVO0_PIN
  0,                                     //SERVO1_PIN  
  0,                                     //SERVO2_PIN  
  0,                                     //SERVO3_PIN 
  0,//BSP_MOTOR_CONTROL_BOARD_PWM_E3_PORT,   //E2_STEP_PIN     45
  0,//BSP_MOTOR_CONTROL_BOARD_DIR_E3_PORT,   //E2_DIR_PIN
  0,//BSP_MOTOR_CONTROL_BOARD_RESET_E3_PORT,  //E2_ENABLE_PIN
  0,//BSP_STOP_U_PORT,                        //U_MIN_PIN
  0,//BSP_STOP_V_PORT,                        //V_MIN_PIN
  0,//BSP_STOP_W_PORT,                         //W_MIN_PIN   50
  0,//BSP_HEAT_BED2_PORT,                     //HEATER_BED2_PIN
  0,//BSP_HEAT_BED3_PORT                      //HEATER_BED3_PIN
};  

uint16_t gArrayGpioPin[BSP_MISC_MAX_PIN_NUMBER] = {
  BSP_MOTOR_CONTROL_BOARD_PWM_X_PIN,    //X_STEP_PIN        0       
  BSP_MOTOR_CONTROL_BOARD_DIR_X_PIN,    //X_DIR_PIN         1
  BSP_MOTOR_CONTROL_BOARD_RESET_X_PIN,  //X_ENABLE_PIN    
  0,                       				//X_MIN_PIN
  BSP_STOP_X_PIN,                                    //X_MAX_PIN
  BSP_MOTOR_CONTROL_BOARD_PWM_Y_PIN,    //Y_STEP_PIN        5
  BSP_MOTOR_CONTROL_BOARD_DIR_Y_PIN,    //Y_DIR_PIN               
  BSP_MOTOR_CONTROL_BOARD_RESET_Y_PIN,  //Y_ENABLE_PIN    
  0,                       				//Y_MIN_PIN
  BSP_STOP_Y_PIN,                                    //Y_MAX_PIN
  BSP_MOTOR_CONTROL_BOARD_PWM_Z_PIN,    //Z_STEP_PIN       10
  BSP_MOTOR_CONTROL_BOARD_DIR_Z_PIN,    //Z_DIR_PIN       
  BSP_MOTOR_CONTROL_BOARD_RESET_Z_PIN,  //Z_ENABLE_PIN    
  BSP_STOP_W_PIN,                       //Z_MIN_PIN
  BSP_STOP_Z_PIN,                       //Z_MAX_PIN
  0,                                    //Y2_STEP_PIN      15
  0,                                    //Y2_DIR_PIN      
  0,                                    //Y2_ENABLE_PIN   
  0,                                    //Z2_STEP_PIN     
  0,                                    //Z2_DIR_PIN      
  0,                                    //Z2_ENABLE_PIN    20
  BSP_MOTOR_CONTROL_BOARD_PWM_E1_PIN,   //E0_STEP_PIN     
  BSP_MOTOR_CONTROL_BOARD_DIR_E1_PIN,   //E0_DIR_PIN             
  BSP_MOTOR_CONTROL_BOARD_RESET_E1_PIN, //E0_ENABLE_PIN       
  0,//BSP_MOTOR_CONTROL_BOARD_PWM_E2_PIN,   //E1_STEP_PIN
  0,//BSP_MOTOR_CONTROL_BOARD_DIR_E2_PIN,   //E1_DIR_PIN       25
  0,//BSP_MOTOR_CONTROL_BOARD_RESET_E2_PIN, //E1_ENABLE_PIN
  0,                                    //SDPOWER         
  0,                                    //SDSS            
  0,                                    //LED_PIN        
  BSP_FAN_E1_PIN,                       //FAN_PIN          30   
  0,                                    //PS_ON_PIN  
  0,                                    //KILL_PIN        
  BSP_HEAT_E1_PIN,                      //HEATER_0_PIN    
  0,//BSP_HEAT_E2_PIN,                      //HEATER_1_PIN
  0,//BSP_HEAT_E3_PIN,                      //HEATER_2_PIN     35
  BSP_THERM_E1_PIN,                     //TEMP_0_PIN      
  0,//BSP_THERM_E2_PIN,                     //TEMP_1_PIN
  0,//BSP_THERM_E3_PIN,                     //TEMP_2_PIN
  BSP_HEAT_BED1_PIN,                     //HEATER_BED_PIN   
  BSP_THERM_BED1_PIN,                    //TEMP_BED_PIN     40    
  0,//BSP_SERVO0_PIN,                       //SERVO0_PIN
  0,                                    //SERVO1_PIN  
  0,                                    //SERVO2_PIN  
  0,                                    //SERVO3_PIN  
  0,//BSP_MOTOR_CONTROL_BOARD_PWM_E3_PIN,   //E2_STEP_PIN      45
  0,//BSP_MOTOR_CONTROL_BOARD_DIR_E3_PIN,   //E2_DIR_PIN
  0,//BSP_MOTOR_CONTROL_BOARD_RESET_E3_PIN, //E2_ENABLE_PIN
  0,//BSP_STOP_U_PIN,                       //U_MIN_PIN
  0,//BSP_STOP_V_PIN,                       //V_MIN_PIN
  0,//BSP_STOP_W_PIN,                        //W_MIN_PIN     50
  0,//BSP_HEAT_BED2_PIN,                     //HEATER_BED2_PIN
  0,//BSP_HEAT_BED3_PIN                      //HEATER_BED3_PIN
};  

/* Type definition ------------------------------------------------------------*/
typedef struct {
	 uint32_t 		gpioPin;
	 GPIO_TypeDef* 	gpioPort;
	 uint8_t		speed;
	 uint8_t		up_val;
	 uint8_t		count;
	 uint8_t		level; /* Up = 1 ; down = 0 */
	 uint8_t		activePwm;
} tFanStruct;


/* Global variable ------------------------------------------------------------*/
static tFanStruct fanE1;
#ifdef BSP_HEAT_E2_PIN
static tFanStruct fanE2;
static tFanStruct fanE3;
#endif//BSP_HEAT_E2_PIN

/* Imported variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef hTimPwmX;
extern TIM_HandleTypeDef hTimPwmY;
extern TIM_HandleTypeDef hTimPwmZ;
extern TIM_HandleTypeDef hTimPwmE1;
#ifdef BSP_HEAT_E2_PIN
extern TIM_HandleTypeDef hTimPwmE2;
extern TIM_HandleTypeDef hTimPwmE3;
#endif//BSP_HEAT_E2_PIN
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef hTimPwmHeatBed;
#ifdef BSP_HEAT_BED2_PIN
TIM_HandleTypeDef hTimPwmHeatBed2;
TIM_HandleTypeDef hTimPwmHeatBed3;
#endif//BSP_HEAT_BED2_PIN
TIM_HandleTypeDef hTimPwmHeatE1;
#ifdef BSP_HEAT_E2_PIN
TIM_HandleTypeDef hTimPwmHeatE2;
TIM_HandleTypeDef hTimPwmHeatE3;
#endif//BSP_HEAT_E2_PIN
/// Number of motor devices
uint8_t bspMiscNbMotorDevices = 4;

/// Number of the last error
static volatile uint16_t bspMiscLastError;

/// Timer handler for tick
TIM_HandleTypeDef hTimTick;

/// Timer handler for tick2
TIM_HandleTypeDef hTimTick2;


/// Timer handler for servo
TIM_HandleTypeDef hTimServo;

/* Private constant ----------------------------------------------------------*/
static uint8_t bspTickEnabled = 0;

/* Imported function ----------------------------------------------------------*/
void BSP_MiscFlagInterruptHandler(void);
void SystemClock_Config(void);


/** @defgroup  stm32f0XX_3DPRINTER_MOTOR_Public_Functions
  * @{
  */   
/******************************************************//**
 * @brief  Global Init of the motors part of the 3D printer
 * @param[in] nbDevices Number of motor devices to use (from 1 to MAX_NUMBER_OF_DEVICES)
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 **********************************************************/
void BSP_MiscOverallInit(uint8_t nbDevices)
{

	/* Init fan struct */
	memset( &fanE1, 0x0, sizeof(tFanStruct));
#ifdef BSP_HEAT_E2_PIN
	memset( &fanE2, 0x0, sizeof(tFanStruct));
	memset( &fanE3, 0x0, sizeof(tFanStruct));
#endif//BSP_HEAT_E2_PIN
   /* STM32xx HAL library initialization */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* System interrupt init*/
  /* This part overwrite configuration of priority grouping realized in HAL_Init()
   * Objective is to avoid change in HAL_Init()
   * Due to change in Priority Grouping, the system tick init shall be do again.
   */
  //TODO: check this, stm32F0 does not support priority grouping
  /* Set Interrupt Group Priority */
//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);
  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  HAL_InitTick(TICK_INT_PRIORITY);
  
  //----- Init of the Motor control library to use nB device */
  bspMiscNbMotorDevices = nbDevices;
#ifdef MOTOR_L6474
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, nbDevices);
#elif defined(MOTOR_A4985)
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_A4985, nbDevices);
#endif
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
//  BSP_MotorControl_AttachFlagInterrupt(BSP_MiscFlagInterruptHandler);

  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(BSP_MiscErrorHandler); 
}

/******************************************************//**
 * @brief  Handler of the motor flag interrupt
 * @param None
 * @retval None
 **********************************************************/
void BSP_MiscFlagInterruptHandler(void)
{
  uint16_t statusRegister;
  uint8_t loop;
  
  for (loop =0; loop < bspMiscNbMotorDevices; loop++)
  {
    /* Get the value of the status register via the L6474 command GET_STATUS */
    statusRegister = BSP_MotorControl_CmdGetStatus(loop);
#ifdef MOTOR_L6474
    /* Check HIZ flag: if set, power brigdes are disabled */
    if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
    {
      // HIZ state
      // Action to be customized            
    }

    /* Check direction bit */
    if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
    {
      // Forward direction is set
      // Action to be customized            
    }  
    else
    {
      // Backward direction is set
      // Action to be customized            
    }  

    /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
    /* This often occures when a command is sent to the L6474 */
    /* while it is in HIZ state */
    if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
    {
        // Command received by SPI can't be performed
       // Action to be customized            
    }  

    /* Check WRONG_CMD flag: if set, the command does not exist */
    if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
    {
       //command received by SPI does not exist 
       // Action to be customized          
    }  

    /* Check UVLO flag: if not set, there is an undervoltage lock-out */
    if ((statusRegister & L6474_STATUS_UVLO) == 0)
    {
       //undervoltage lock-out 
       // Action to be customized          
    }  

    /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
    if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
    {
      //thermal warning threshold is reached
      // Action to be customized          
    }    

    /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
    if ((statusRegister & L6474_STATUS_TH_SD) == 0)
    {
      //thermal shut down threshold is reached 
      // Action to be customized          
    }    

    /* Check OCD  flag: if not set, there is an overcurrent detection */
    if ((statusRegister & L6474_STATUS_OCD) == 0)
    {
      //overcurrent detection 
      // Action to be customized          
    }
#endif
  }
}


/******************************************************//**
 * @brief  General error handler
 * @param None
 * @retval None
 **********************************************************/
void BSP_MiscErrorHandler(uint16_t error)
{
  __disable_irq();
  static char errorTxt[15] = "Error = 0x";
  char* errorTxtPtr;
  errorTxtPtr = errorTxt + strlen(errorTxt);
  /* Backup error number */
  bspMiscLastError = error;
  sprintf(errorTxtPtr, "%x\r\n",error);
  
  
  /* Disable heater */
#ifndef MARLIN
  BSP_MiscHeatPwmSetDutyCycle(0, 0);
  BSP_MiscHeatPwmSetDutyCycle(1, 0);
#else  
  BSP_MiscHeatManualInit(0);
  BSP_MiscHeatManualInit(1);
  BSP_MiscHeatManualInit(2);
  BSP_MiscHeatManualInit(3);
#endif  
  BSP_LED_On(LED_BLUE);
  
//TODO: find a strategy to communicate error messages correctly
// #ifndef STM32_USE_USB_CDC
//  BSP_UartLockingTx((uint8_t *)&errorTxt, sizeof(errorTxt));
// #else
//  BSP_CdcLockingTx((uint8_t *)&errorTxt, sizeof(errorTxt));
// #endif //STM32_USE_USB_CDC
  
  /* Infinite loop */
  while(1)
  {
	  BSP_LED_Toggle(LED_BLUE);
	  //Systick disabled, use loop for delay
	  for(int i=0;i<10e3;i++) { }
  }
}
/******************************************************//**
 * @brief  Initialisation of the Z probe
 * @param None
 * @retval None
 **********************************************************/
#ifdef BSP_IR_ON_PIN
void BSP_MiscInitZProbeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Configure IR_ON pin */
  GPIO_InitStruct.Pin = BSP_IR_ON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BSP_IR_ON_PORT, &GPIO_InitStruct);
  
}

/******************************************************//**
 * @brief  Enabling of the Z probe
 * @param[in] enable 0 to disable, 1 to enable
 * @retval None
 **********************************************************/
void BSP_MiscInitZProbeEnable(uint8_t enable)
{
  if (enable == 0)  
  {
    HAL_GPIO_WritePin(BSP_IR_ON_PORT, BSP_IR_ON_PIN, GPIO_PIN_RESET); 
  }
  else
  {
    HAL_GPIO_WritePin(BSP_IR_ON_PORT, BSP_IR_ON_PIN, GPIO_PIN_SET); 
  }
}
#endif
/******************************************************//**
 * @brief  Initialisation of the stops
 * @param[in] id 0 for X_Stop, 1 for Y_Stop, 2 for Z_Stop
 * @retval None
 **********************************************************/
void BSP_MiscStopInit(uint8_t id)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  //Default to input, add EXTI to W pin
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  switch (id)
  {
    case 0:
      /* Configure X_STOP pin */
      gpioPin = BSP_STOP_X_PIN;
      gpioPort = BSP_STOP_X_PORT;    
#if defined(STOP_X__PULL_UP)
      GPIO_InitStruct.Pull = GPIO_PULLUP;
#elif defined(STOP_X__PULL_DOWN)
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#else
      GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif
      break;
    case 1:
      /* Configure Y_STOP pin */
      gpioPin = BSP_STOP_Y_PIN;
      gpioPort = BSP_STOP_Y_PORT;    
#if defined(STOP_Y__PULL_UP)
      GPIO_InitStruct.Pull = GPIO_PULLUP;
#elif defined(STOP_Y__PULL_DOWN)
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#else
      GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif
      break;      
    case 2:
      /* Configure Z_STOP pin */
      gpioPin = BSP_STOP_Z_PIN;
      gpioPort = BSP_STOP_Z_PORT;
#if defined(STOP_Z__PULL_UP)
      GPIO_InitStruct.Pull = GPIO_PULLUP;
#elif defined(STOP_Z__PULL_DOWN)
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#else
      GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif
      break;   
//TODO: make this a conditional compile
//    case 3:
//      /* Configure U_STOP pin */
//      gpioPin = BSP_STOP_U_PIN;
//      gpioPort = BSP_STOP_U_PORT;
//#ifndef MARLIN
//      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//#else
//      GPIO_InitStruct.Pull = GPIO_PULLUP;
//#endif
//      break;
//    case 4:
//      /* Configure V_STOP pin */
//      gpioPin = BSP_STOP_V_PIN;
//      gpioPort = BSP_STOP_V_PORT;
//#ifndef MARLIN
//      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//#else
//      GPIO_InitStruct.Pull = GPIO_PULLUP;
//#endif
//      break;
    case 5:
      /* Configure W_STOP pin */
      gpioPin = BSP_STOP_W_PIN;
      gpioPort = BSP_STOP_W_PORT;
      GPIO_InitStruct.Mode |= GPIO_MODE_IT_RISING_FALLING;
#if defined(STOP_W__PULL_UP)
      GPIO_InitStruct.Pull = GPIO_PULLUP;
#elif defined(STOP_W__PULL_DOWN)
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#else
      GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif
      break;        
    default:
      return;
  }
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);      
  switch(id) {
	  case 5:
	  /* Configure W_STOP pin */
		  NVIC_EnableIRQ(BSP_STOP_W_IRQN);
		  NVIC_SetPriority(BSP_STOP_W_IRQN,3);//Set lowest priority
	  break;
  }
}

/******************************************************//**
 * @brief  Get Stop status
 * @param[in] id 0 for X_Stop, 1 for Y_Stop, 2 for Z_Stop
  * @retval status 0 if GPIO is set, 1 if it reset
 **********************************************************/
uint8_t BSP_MiscStopGetStatus(uint8_t id)
{
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  uint8_t status = 0;
  
  switch (id)
  {
    case 0:
      /* Configure X_STOP pin */
      gpioPin = BSP_STOP_X_PIN;
      gpioPort = BSP_STOP_X_PORT;    
      break;
    case 1:
      /* Configure Y_STOP pin */
      gpioPin = BSP_STOP_Y_PIN;
      gpioPort = BSP_STOP_Y_PORT;    
      break;      
    case 2:
      /* Configure Z_STOP pin */
      gpioPin = BSP_STOP_Z_PIN;
      gpioPort = BSP_STOP_Z_PORT;    
      break;   
    default:
      return (status);
  }
  status = !((uint8_t) HAL_GPIO_ReadPin(gpioPort, gpioPin)); 
  return (status);
}

/******************************************************//**
 * @brief  Initialisation of the Heats
 * @param[in] id 0 for E1_FAN, 1 for E2, 2 for E3, 3 for E4
 * @retval None
 **********************************************************/
void BSP_MiscFanInit(uint8_t id)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  tFanStruct* pfan;

  switch (id)
  {
    case 0:
    	/* Configure E1 Fan pin */
    	gpioPin  = BSP_FAN_E1_PIN;
    	gpioPort = BSP_FAN_E1_PORT;
    	pfan = &fanE1;
      break;
#ifdef BSP_FAN_E2_PIN
    case 1:
      /* Configure E2 Fan pin */
    	gpioPin  = BSP_FAN_E2_PIN;
    	gpioPort = BSP_FAN_E2_PORT;
    	pfan = &fanE2;
      break;      
    case 2:
      /* Configure E3 Fan pin */
    	gpioPin  = BSP_FAN_E3_PIN;
    	gpioPort = BSP_FAN_E3_PORT;
    	pfan = &fanE3;
      break;   
#endif
    default:
      return;
  }

  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);

  pfan->gpioPin  = gpioPin;
  pfan->gpioPort = gpioPort;
  pfan->activePwm   = 0;
}

#define BSP_FAN_PWM_COUNTS ((uint8_t) (1000/BSP_FAN_PWM_FREQ))
/******************************************************//**
 * @brief  Initialisation of the Heats
 * @param[in] id 0 for E1_FAN, 1 for E2, 2 for E3, 3 for E4
 * @param[in] speed from 0 (min speed) to 255 (full speed) 
 * @retval None
 **********************************************************/
void BSP_MiscFanSetSpeed(uint8_t id,uint8_t speed)
{
	tFanStruct* pfan;

#ifndef MARLIN
	speed = 255; // Force speed to never stop the fan !!!
#endif 
 
	if ( (speed>0) && (speed<50) )
		speed = 50;

	if( id == 0)
    	/* Configure E1 Fan */
    	pfan = &fanE1;
#ifdef BSP_HEAT_E2_PIN
	else if ( id == 1)
    	/* Configure E2 Fan */
    	pfan = &fanE2;
	else if ( id == 2)
		/* Configure E3 Fan */
	    pfan = &fanE3;
#endif//BSP_HEAT_E2_PIN
	else // Error case
		return;
  
  if (speed == 0)
  {
     HAL_GPIO_WritePin(pfan->gpioPort, pfan->gpioPin, GPIO_PIN_RESET);
     pfan->activePwm = 0;
     pfan->speed = speed;
  }
  else if (speed == 255)
  {
     HAL_GPIO_WritePin(pfan->gpioPort, pfan->gpioPin, GPIO_PIN_SET);
     pfan->activePwm = 0;
     pfan->speed = speed;
  }
  /* Else : PWM */
  else if( speed != pfan->speed)
  {
	  //HAL_GPIO_WritePin(pfan->gpioPort, pfan->gpioPin, GPIO_PIN_SET);
	  pfan->speed = speed;
	  pfan->up_val = (uint8_t)((speed * BSP_FAN_PWM_COUNTS)/255);
	  pfan->count = pfan->up_val;
	  pfan->level = 1; /* Up state*/
	  pfan->activePwm = 1;
  }
}

static inline void refresh(void)
{
	volatile uint32_t *IWDG_KR = &(IWDG->KR);
	volatile uint32_t *IWDG_RLR = &(IWDG->RLR);
	 if(*IWDG_RLR)
		*IWDG_KR = 0x0000AAAAU;
}

/******************************************************//**
 * @brief  Management of the Heats under IT
 * @param[in] None
 * @retval None
 **********************************************************/
void HAL_SYSTICK_Callback(void)
{
	refresh();
	if( fanE1.activePwm)
	{
		fanE1.count--;

		if( !fanE1.count)
		{
			if(fanE1.level) /* signal level was high */
			{
				HAL_GPIO_WritePin(fanE1.gpioPort, fanE1.gpioPin, GPIO_PIN_RESET);
				fanE1.count = BSP_FAN_PWM_COUNTS - fanE1.up_val;
				fanE1.level = 0;
			}
			else/* signal level was low */
			{
				HAL_GPIO_WritePin(fanE1.gpioPort, fanE1.gpioPin, GPIO_PIN_SET);
				fanE1.count = fanE1.up_val;
				fanE1.level = 1;
			}
		}
	}
#ifdef BSP_HEAT_E2_PIN
	else if( fanE2.activePwm)
	{
		fanE2.count--;
		if( fanE2.count > 0)
		{
			if(fanE2.level) /* signal level was high */
			{
				HAL_GPIO_WritePin(fanE2.gpioPort, fanE2.gpioPin, GPIO_PIN_RESET);
				fanE2.count = 100 - fanE2.up_val;
				fanE2.level = 0;
			}
			else/* signal level was low */
			{
				HAL_GPIO_WritePin(fanE2.gpioPort, fanE2.gpioPin, GPIO_PIN_SET);
				fanE2.count = fanE2.up_val;
				fanE2.level = 1;
			}
		}
	}
	else if( fanE3.activePwm)
	{
		fanE3.count--;
		if( fanE3.count > 0)
		{
			if(fanE3.level) /* signal level was high */
			{
				HAL_GPIO_WritePin(fanE3.gpioPort, fanE3.gpioPin, GPIO_PIN_RESET);
				fanE3.count = 100 - fanE3.up_val;
				fanE3.level = 0;
			}
			else/* signal level was low */
			{
				HAL_GPIO_WritePin(fanE3.gpioPort, fanE3.gpioPin, GPIO_PIN_SET);
				fanE3.count = fanE3.up_val;
				fanE3.level = 1;
			}
		}
	}
#endif//BSP_HEAT_E2_PIN

}


/******************************************************//**
 * @brief  Initialisation of the Tick timer 
 * @param None
  * @retval None
 **********************************************************/
void BSP_MiscTickInit(void)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
#ifndef MARLIN
  hTimTick.Instance = BSP_MISC_TIMER_TICK;
  hTimTick.Init.Prescaler = TICK_TIMER_PRESCALER - 1;
  hTimTick.Init.CounterMode = TIM_COUNTERMODE_UP;
  hTimTick.Init.Period = 0;
  hTimTick.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&hTimTick);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_OC_ConfigChannel(&hTimTick, &sConfigOC, BSP_MISC_CHAN_TIMER_TICK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&hTimTick, &sMasterConfig);
#else //#ifndef MARLIN
  hTimTick.Instance = BSP_MISC_TIMER_TICK;
  hTimTick.Init.Prescaler = TICK_TIMER_PRESCALER - 1;
  hTimTick.Init.CounterMode = TIM_COUNTERMODE_UP;
  //TODO: all timers on STM32F070 are 16-bit
//  if ((hTimTick.Instance != TIM2) && (hTimTick.Instance != TIM5))
//  {
    hTimTick.Init.Period = 0xFFFF;
//  }
//  else
//  {
//    hTimTick.Init.Period = 0xFFFFFFFF;
//  }
  //TODO: check this, unclear what's special about TIM4, it seems all timer's can handle this
  hTimTick.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  if (hTimTick.Instance != TIM4)
//  {
    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
//  }
//  else
//  {
//    sConfigOC.OCMode = TIM_OCMODE_TIMING;
//  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.Pulse = 0;
  
  HAL_TIM_OC_Init(&hTimTick);
  HAL_TIM_OC_ConfigChannel(&hTimTick, &sConfigOC, BSP_MISC_CHAN_TIMER_TICK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&hTimTick, &sMasterConfig);
#endif //else #ifndef MARLIN
}


/******************************************************//**
 * @brief  Sets the frequency of PWM_X used by device 0
 * @param[in] newPeriod in seconds
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MiscTickSetFreq(uint32_t newFreq)
{
#ifndef MARLIN
    uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t tick;
  uint32_t timPeriod = ((uint32_t)(sysFreq * newPeriod)/ TICK_TIMER_PRESCALER) - 1; 
  
  
  __HAL_TIM_SetAutoreload(&hTimTick, timPeriod);
  __HAL_TIM_SetCompare(&hTimTick, BSP_MISC_CHAN_TIMER_TICK, timPeriod >> 1);
  HAL_TIM_PWM_Start_IT(&hTimTick, BSP_MISC_CHAN_TIMER_TICK);  
  
  tick = __HAL_TIM_GetCounter(&hTimTick);
  if (timPeriod < tick)
  {
    __HAL_TIM_SetCounter(&hTimTick,0); //To prevent tick overlap workaround
  }
#else //#ifndef MARLIN

  uint32_t timPeriod;
  uint32_t timerCnt = hTimTick.Instance->CNT;
//  if(newFreq > 10000) {
//    // If newFreq > 20kHz >> step 4 times
//    newFreq = (newFreq >> 2)&0x3fff;
//  }
//  else if(newFreq > 5000)
//  {
//    // If newFreq > 10kHz >> step 2 times
//    newFreq = (newFreq >> 1)&0x7fff;
//  }
  
  timPeriod = (HAL_RCC_GetSysClockFreq()/ (TICK_TIMER_PRESCALER * (uint32_t)newFreq));
  if (timPeriod < 100) timPeriod = 100;
  if (timPeriod > 0xFFFF) timPeriod = 0xFFFF;
  __HAL_TIM_SetCompare(&hTimTick, BSP_MISC_CHAN_TIMER_TICK, timerCnt + timPeriod);
  
  if (bspTickEnabled == 0)
  {
     if ((BSP_MISC_CHAN_TIMER_TICK) == (TIM_CHANNEL_1))
    {
      __HAL_TIM_CLEAR_IT(&hTimTick, TIM_IT_CC1);
    }
    else if (BSP_MISC_CHAN_TIMER_TICK == TIM_CHANNEL_2)
    {
      __HAL_TIM_CLEAR_IT(&hTimTick, TIM_IT_CC2);
    }
    else if (BSP_MISC_CHAN_TIMER_TICK == TIM_CHANNEL_3)
    {
      __HAL_TIM_CLEAR_IT(&hTimTick, TIM_IT_CC3);
    }
    else if (BSP_MISC_CHAN_TIMER_TICK == TIM_CHANNEL_4)
    {
      __HAL_TIM_CLEAR_IT(&hTimTick, TIM_IT_CC4);
    }
    HAL_TIM_OC_Start_IT(&hTimTick,BSP_MISC_CHAN_TIMER_TICK);  
    bspTickEnabled = 1; 
  }
#endif //else #ifndef MARLIN  
}

/******************************************************//**
 * @brief  Sets the frequency of PWM_X used by device 0
 * @param[in] newPeriod in seconds
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MiscTickSetPeriod(uint32_t newTimPeriod)
{
  uint32_t timerCnt = hTimTick.Instance->CNT + newTimPeriod;
  __HAL_TIM_SetCompare(&hTimTick, BSP_MISC_CHAN_TIMER_TICK, timerCnt);
    
}
/******************************************************//**
 * @brief  Stop the PWM used for the tick
 * @param[in] None
 * @retval None
  **********************************************************/
void BSP_MiscTickStop(void)
{
  HAL_TIM_PWM_Stop_IT(&hTimTick,BSP_MISC_CHAN_TIMER_TICK);  
  bspTickEnabled = 0;
}

/******************************************************//**
 * @brief  Initialisation of the Tick2 timer 
 * @param None
  * @retval None
 **********************************************************/
void BSP_MiscTick2Init(void)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;

  hTimTick2.Instance = BSP_MISC_TIMER_TICK2;
  hTimTick2.Init.Prescaler = TICK_TIMER_PRESCALER - 1;
  hTimTick2.Init.CounterMode = TIM_COUNTERMODE_UP;
  hTimTick2.Init.Period = 0;
  hTimTick2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&hTimTick2);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_OC_ConfigChannel(&hTimTick2, &sConfigOC, BSP_MISC_CHAN_TIMER_TICK2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&hTimTick2, &sMasterConfig);

}


/******************************************************//**
 * @brief  Sets the frequency of PWM_X used by device 0
 * @param[in] newPeriod in seconds
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MiscTick2SetFreq(float newPeriod)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t tick;
  uint32_t timPeriod = ((uint32_t)(sysFreq * newPeriod)/ TICK_TIMER_PRESCALER) - 1; 
  
  __HAL_TIM_SetAutoreload(&hTimTick2, timPeriod);
  __HAL_TIM_SetCompare(&hTimTick2, BSP_MISC_CHAN_TIMER_TICK2, timPeriod >> 1);
  HAL_TIM_PWM_Start_IT(&hTimTick2, BSP_MISC_CHAN_TIMER_TICK2);  
  
  tick = __HAL_TIM_GetCounter(&hTimTick2);
  if (timPeriod < tick)
  {
    __HAL_TIM_SetCounter(&hTimTick2,0); //To prevent tick overlap workaround
  }
}
/******************************************************//**
 * @brief  Stop the PWM used for the tick2
 * @param[in] None
 * @retval None
  **********************************************************/
void BSP_MiscTick2Stop(void)
{
  HAL_TIM_PWM_Stop_IT(&hTimTick2,BSP_MISC_CHAN_TIMER_TICK2);  
}

/******************************************************//**
 * @brief  Sets SW step clock mode (where step clocks are
 * not handled through HW PWMs but commutated by SW
 * @param None
 * @retval None
**********************************************************/
void BSP_MiscSetStepClockToSwMode(void)
{
  TIM_HandleTypeDef *pHTim;
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t deviceId;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;  
  
  for (deviceId = 0; deviceId < bspMiscNbMotorDevices; deviceId++) 
  {
    switch (deviceId)
    {
      case 0:
      default:
        pHTim = &hTimPwmX;
        pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_X;
        gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_X_PIN;
        gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_X_PORT;
        break;
      case  1:
        pHTim = &hTimPwmY;
        pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Y;
        gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_Y_PIN;
        gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_Y_PORT;        
        break;
      case 2:
        pHTim = &hTimPwmZ;
        pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_Z;
        gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_Z_PIN;
        gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_Z_PORT;           
        break;
      case 3:
        pHTim = &hTimPwmE1;
        pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E1;
        gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_E1_PIN;
        gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_E1_PORT;            
        break;
#ifdef BSP_HEAT_E2_PIN
      case 4:
        pHTim = &hTimPwmE2;
        pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E2;
        gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_E2_PIN;
        gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_E2_PORT;             
        break;
      case 5:
        pHTim = &hTimPwmE3;
        pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_E3;
        gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_E3_PIN;
        gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_E3_PORT;            
        break;
#endif//BSP_HEAT_E2_PIN
    }
    HAL_TIM_PWM_DeInit(pHTim);
  
  /* GPIO configuration */
    GPIO_InitStruct.Pin = gpioPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);    
    HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET); 

  }
}
/******************************************************//**
 * @brief Generate a step clock pulse
 * @param None
 * @retval None
**********************************************************/
void BSP_MiscGenerateStepClockPulse(uint8_t deviceId)
{
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;  

  switch (deviceId)
  {
    case 0:
    default:
      gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_Z_PIN;
      gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_Z_PORT; 
      break;
    case  1:
      gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_Y_PIN;
      gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_Y_PORT;        
      break;
    case 2:
      gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_X_PIN;
      gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_X_PORT;         
      break;
    case 3:
      gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_E1_PIN;
      gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_E1_PORT;            
      break;
#ifdef BSP_HEAT_E2_PIN
    case 4:
      gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_E2_PIN;
      gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_E2_PORT;             
      break;
    case 5:
      gpioPin = BSP_MOTOR_CONTROL_BOARD_PWM_E3_PIN;
      gpioPort = BSP_MOTOR_CONTROL_BOARD_PWM_E3_PORT;            
      break;
#endif//BSP_HEAT_E2_PIN
  }  
  
  HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET); 
}
                                    
 /******************************************************//**
 * @brief Generate a step clock pulse
 * @param[in] device Id 
 * @param[in] current value in mA
 * @retval None
**********************************************************/
void BSP_MiscSetOcdThreshold(uint8_t deviceId, uint32_t current)
{
  int32_t currentThreshold = 375;
  uint32_t ocdThresholdParam = 0;
  //TODO:check functionality that this actually still does hwat it's supposed to
#ifdef MOTOR_L6474
  while ((current >= currentThreshold) && (ocdThresholdParam < L6474_OCD_TH_6000mA))
  {
    currentThreshold += 375;
    ocdThresholdParam++;
  }
  
  BSP_MotorControl_CmdSetParam(deviceId, L6474_OCD_TH, ocdThresholdParam);   
#endif
}

/******************************************************//**
 * @brief  Initialises the Gpios uses by the specified Heats
 * @param[in] heatId (from 0 to 4)
 * @retval None
 * @note 0 for bead heat, 1 for E1 heat, 2 for E2 heat...
 **********************************************************/
void BSP_MiscHeatManualInit(uint8_t heatId)
{
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  switch (heatId)
  {
    case 0:
      gpioPin = BSP_HEAT_BED1_PIN;
      gpioPort = BSP_HEAT_BED1_PORT; 
      break;
    case  1:
      gpioPin = BSP_HEAT_E1_PIN;
      gpioPort = BSP_HEAT_E1_PORT; 
      break;
#ifdef BSP_HEAT_E2_PIN
    case 2:
      gpioPin = BSP_HEAT_E2_PIN;
      gpioPort = BSP_HEAT_E2_PORT; 
      break;
    case 3:
      gpioPin = BSP_HEAT_E3_PIN;
      gpioPort = BSP_HEAT_E3_PORT; 
      break;
#endif//BSP_HEAT_E2_PIN
#ifdef BSP_HEAT_BED2_PIN
    case 5:
      gpioPin = BSP_HEAT_BED2_PIN;
      gpioPort = BSP_HEAT_BED2_PORT; 
      break;
    case 6:
      gpioPin = BSP_HEAT_BED3_PIN;
      gpioPort = BSP_HEAT_BED3_PORT; 
      break;
#endif//BSP_HEAT_BED2_PIN
    default:
    	return;
  }
  /* GPIO configuration */
    GPIO_InitStruct.Pin = gpioPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);    
    HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET); 
}

/******************************************************//**
 * @brief  Initialises the PWM uses by the specified Heats
 * @param[in] heatId (from 0 to 4)
 * @retval None
 * @note 0 for bead heat, 1 for E1 heat, 2 for E2 heat...
 **********************************************************/
void BSP_MiscHeatPwmInit(uint8_t heatId)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_HandleTypeDef *pHTim;
  uint32_t  channel;
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period;
  
  switch (heatId)
  {
    case 0:
    default:
      pHTim = &hTimPwmHeatBed;
      pHTim->Instance = BSP_MISC_TIMER_PWM_HEAT_BED;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_BED;
      break;
    case  1:
      pHTim = &hTimPwmHeatE1;
      pHTim->Instance = BSP_MISC_TIMER_PWM_HEAT_E1;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_E1;
      break;
#ifdef BSP_HEAT_E2_PIN
    case 2:
      pHTim = &hTimPwmHeatE2;
      pHTim->Instance = BSP_MISC_TIMER_PWM_HEAT_E2;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_E2;
      break;
    case 3:
      pHTim = &hTimPwmHeatE3;
      pHTim->Instance = BSP_MISC_TIMER_PWM_HEAT_E3;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_E3;
      break;
#endif//BSP_HEAT_E2_PIN
#ifdef BSP_HEAT_BED2_PIN
    case 5:
      pHTim = &hTimPwmHeatBed2;
      pHTim->Instance = BSP_MISC_TIMER_PWM_HEAT_BED2;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_BED2;
      break;
    case 6:
      pHTim = &hTimPwmHeatBed3;
      pHTim->Instance = BSP_MISC_TIMER_PWM_HEAT_BED3;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_BED3;
      break;
#endif//BSP_HEAT_BED2_PIN
  }
  period = (sysFreq/ (HEAT_TIMER_PRESCALER * HEAT_TIMER_FREQUENCY)) - 1;
  
  pHTim->Init.Prescaler = HEAT_TIMER_PRESCALER -1;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.Period = period;
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
  
  HAL_TIM_PWM_Start(pHTim, channel);    
  
}
                                    
/******************************************************//**
 * @brief  Initialises the PWM uses by the specified Heats
 * @param[in] heatId (from 0 to 4)
 * @param[in] newDuty from 0 to 255
 * @retval None
 * @note 0 for bead heat, 1 for E1 heat, 2 for E2 heat...
 **********************************************************/
void BSP_MiscHeatPwmSetDutyCycle(uint8_t heatId, uint8_t newDuty)
{
  TIM_HandleTypeDef *pHTim;
  uint32_t  channel;
  uint32_t pulse;
  
  switch (heatId)
  {
  case 0:
  default:
      pHTim = &hTimPwmHeatBed;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_BED;
      break;
    case  1:
      pHTim = &hTimPwmHeatE1;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_E1;
      break;
#ifdef BSP_HEAT_E2_PIN
    case 2:
      pHTim = &hTimPwmHeatE2;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_E2;
      break;
    case 3:
      pHTim = &hTimPwmHeatE3;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_E3;
      break;
#endif//BSP_HEAT_E2_PIN
#ifdef BSP_HEAT_BED2_PIN
    case 5:
      pHTim = &hTimPwmHeatBed2;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_BED2;
      break;
    case 6:
      pHTim = &hTimPwmHeatBed3;
      channel = BSP_MISC_CHAN_TIMER_PWM_HEAT_BED3;
      break;
#endif
  }
  
  if (newDuty == 0) 
  {
    pulse = 0 ;
  }
  else 
  {
    pulse = pHTim->Init.Period * newDuty /255 + 1;
  }      
  __HAL_TIM_SetCompare(pHTim, channel, pulse);
}


/******************************************************//**
 * @brief  Initialize the Pwm used to control the servo motor
 * @param[in] pwmFreq frequency in Hz of the PWM 
 * @retval None
 * @note None
 **********************************************************/
#ifdef BSP_SERVO0_PIN
void  BSP_MotorControlBoard_ServoInit(void)
{
  static TIM_OC_InitTypeDef sConfigOC;
  static TIM_MasterConfigTypeDef sMasterConfig;
  static TIM_ClockConfigTypeDef sClockSourceConfig;
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_HandleTypeDef *pHTim = NULL;
  uint32_t channel;
  
  pHTim = &hTimServo;
  pHTim->Instance = BSP_MISC_TIMER_SERVO;
  pHTim->Init.Prescaler = SERVO_TIMER_PRESCALER - 1;
  pHTim->Init.Period = 0;
  channel = BSP_MISC_CHAN_TIMER_SERVO;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(pHTim);  

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(pHTim, &sClockSourceConfig); 
  
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(pHTim, &sConfigOC, channel);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(pHTim, &sMasterConfig);
  
  GPIO_InitStruct.Pin = BSP_SERVO0_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BSP_SERVO0_PORT, &GPIO_InitStruct);      
  HAL_GPIO_WritePin(BSP_SERVO0_PORT, BSP_SERVO0_PIN, GPIO_PIN_RESET);   
}
/******************************************************//**
 * @brief  Set the value of the Servo motor
 * @param[in] value
 * @retval None
 * @note None
 **********************************************************/
void  BSP_MotorControlBoard_ServoSetTimerValue(uint32_t value)
{
  __HAL_TIM_SET_AUTORELOAD(&hTimServo, value - 1);
  HAL_TIM_OC_Start_IT(&hTimServo, BSP_MISC_CHAN_TIMER_SERVO);
}
/******************************************************//**
 * @brief  Stop the timer used by the servo
 * @param None
 * @retval None
 * @note None
 **********************************************************/
void  BSP_MotorControlBoard_ServoStop(void)
{
	//TODO: disable sevo support
  HAL_NVIC_ClearPendingIRQ(BSP_MISC_SERVO_IRQn);
  HAL_TIM_OC_Stop_IT(&hTimServo, BSP_MISC_CHAN_TIMER_SERVO);
  HAL_GPIO_WritePin(BSP_SERVO0_PORT, BSP_SERVO0_PIN, GPIO_PIN_SET);   
}
#endif
/******************************************************//**
 * @brief  Initialisation of the user GPIOs
 * @param[in] id 0 for USER_1, 1 for USER_2, 2 for USER_3, 3 for USER_4
 * @param[in] mode GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP ... look at definitions 
 * in stm32f0xx_hal_gpio.h
 * @param[in] pull GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN
 * @retval None
 **********************************************************/
#ifdef BSP_USER_1_PIN
void BSP_MiscUserGpioInit(uint8_t id, uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;

  switch (id)
  {
    case 0:
      /* Configure user 1 pin */
      gpioPin = BSP_USER_1_PIN;
      gpioPort = BSP_USER_1_PORT;    
      break;
    case 1:
      /* Configure user 2 pin */
      gpioPin = BSP_USER_2_PIN;
      gpioPort = BSP_USER_2_PORT;      
      break;      
    case 2:
      /* Configure user 3 pin */
      gpioPin = BSP_USER_3_PIN;
      gpioPort = BSP_USER_3_PORT;       
      break;   
    case 3:
      /* Configure user 4 pin */
      gpioPin = BSP_USER_4_PIN;
      gpioPort = BSP_USER_4_PORT;      
      break;  
    default:
      return;
  }
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);      
}
#endif


/**
  * @}
  */    

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
