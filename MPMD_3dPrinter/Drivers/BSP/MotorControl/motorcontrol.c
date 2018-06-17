/**
 ******************************************************************************
 * @file    motorcontrol.c
 * @author  IPC Rennes
 * @version V1.1.1
 * @date    15-January-2015
 * @brief   This file provides common functions for motor control 
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
/* Includes ------------------------------------------------------------------*/
#include "motorcontrol.h"

/** @addtogroup BSP
 * @{
 */

/** @defgroup MOTOR_CONTROL
 * @{
 */

/** @defgroup MOTOR_CONTROL_Private_Types_Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup MOTOR_CONTROL_Private_Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Private_Constants
 * @{
 */
/// Error when trying to call undefined functions via motorDrvHandle
#define MOTOR_CONTROL_ERROR_0   (0x0800)   

/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Private_Macros
 * @{
 */
/// Error when trying to call undefined functions via motorDrvHandle
#define MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(errorNb)   (BSP_MotorControl_ErrorHandler(MOTOR_CONTROL_ERROR_0|(errorNb)))   

/**
 * @}
 */   
   
/** @defgroup MOTOR_CONTROL_Private_Variables
 * @{
 */

static motorDrv_t *motorDrvHandle = 0;
static uint16_t MotorControlBoardId;

/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Weak_Private_Functions
 * @{
 */
/// Get motor handle for L6474
__weak motorDrv_t* L6474_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for Powerstep
__weak motorDrv_t* Powerstep01_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for L6206
__weak motorDrv_t* L6206_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for L6208
__weak motorDrv_t* L6208_GetMotorHandle(void){return ((motorDrv_t* )0);}
/// Get motor handle for A4985
__weak motorDrv_t* A4985_GetMotorHandle(void){return ((motorDrv_t* )0);}
/**
 * @}
 */

/** @defgroup MOTOR_CONTROL_Private_Functions
 * @{
 */

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void BSP_MotorControl_AttachErrorHandler(void (*callback)(uint16_t))
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->AttachErrorHandler != 0))
  {
    motorDrvHandle->AttachErrorHandler(callback);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(1);
  }
}

/******************************************************//**
 * @brief  Attaches a user callback to the Flag interrupt Handler.
 * The call back will be then called each time the library 
 * detects a FLAG signal falling edge.
 * @param[in] callback Name of the callback to attach 
 * to the Flag interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_MotorControl_AttachFlagInterrupt(void (*callback)(void))
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->AttachFlagInterrupt != 0))
  {
    motorDrvHandle->AttachFlagInterrupt(callback);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(2);
  }  
}

/******************************************************//**
 * @brief  Attaches a user callback to the Busy interrupt Handler.
 * The call back will be then called each time the library 
 * detects a BUSY signal falling edge.
 * @param[in] callback Name of the callback to attach 
 * to the Busy interrupt Hanlder
 * @retval None
 **********************************************************/
void BSP_MotorControl_AttachBusyInterrupt(void (*callback)(void))
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->AttachBusyInterrupt != 0))
  {
    motorDrvHandle->AttachBusyInterrupt(callback);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(3);
  }  
}

/******************************************************//**
 * @brief Motor control error handler
 * @param[in] error number of the error
 * @retval None
 **********************************************************/
void BSP_MotorControl_ErrorHandler(uint16_t error)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ErrorHandler != 0))
  {
    motorDrvHandle->ErrorHandler(error);
  }  
  else
  {
    while(1)
    {
      /* Infinite loop as Error handler must be defined*/
    }
  }
}
/******************************************************//**
 * @brief Initialises the motor driver
 * @param[in] id Component Id (L6474, Powerstep01,...)
 * @param[in] nbDevices Number of motor devices to use (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval None
 **********************************************************/
void BSP_MotorControl_Init(uint16_t id, uint8_t nbDevices)
{
  MotorControlBoardId = id;
  
  if (id == BSP_MOTOR_CONTROL_BOARD_ID_L6474)
  {
    motorDrvHandle = L6474_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01)
  {
    motorDrvHandle = Powerstep01_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_L6206)
  {
    motorDrvHandle = L6206_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_L6208)
  {
    motorDrvHandle = L6208_GetMotorHandle();
  }
  else if (id == BSP_MOTOR_CONTROL_BOARD_ID_A4985) {
	  motorDrvHandle = A4985_GetMotorHandle();
  }
  else
  {
    motorDrvHandle = 0;
  }

  if ((motorDrvHandle != 0)&&(motorDrvHandle->Init != 0))
  {
    motorDrvHandle->Init(nbDevices);
  }  
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(4);
  }  
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControl_FlagInterruptHandler(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->FlagInterruptHandler != 0))
  {
    motorDrvHandle->FlagInterruptHandler();
  }    
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(5);
  }  
}
/******************************************************//**
 * @brief Returns the acceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Acceleration in pps^2
 **********************************************************/
uint16_t BSP_MotorControl_GetAcceleration(uint8_t deviceId)
{                                                  
  uint16_t acceleration = 0;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetAcceleration != 0))
  {
    acceleration = motorDrvHandle->GetAcceleration(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(6);
  }  
  return(acceleration);    
}            

/******************************************************//**
 * @brief Get board Id  the motor driver
 * @param None
 * @retval Motor control board Id
 **********************************************************/
uint16_t BSP_MotorControl_GetBoardId(void)
{
  return (MotorControlBoardId);
}
/******************************************************//**
 * @brief Returns the current speed of the specified device
 * @param[in] deviceId from 0 to (MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval Speed in pps for stepper motor
 *               in % for Brush DC motor (0-100)   
 **********************************************************/
uint16_t BSP_MotorControl_GetCurrentSpeed(uint8_t deviceId)
{
  uint16_t currentSpeed = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetCurrentSpeed != 0))
  {
    currentSpeed = motorDrvHandle->GetCurrentSpeed(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(7);
  }  
  return(currentSpeed); 
}

/******************************************************//**
 * @brief Returns the deceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Deceleration in pps^2
 **********************************************************/
uint16_t BSP_MotorControl_GetDeceleration(uint8_t deviceId)
{                                                  
  uint16_t deceleration = 0;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDeceleration != 0))
  {
    deceleration = motorDrvHandle->GetDeceleration(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(8);
  }  
  return(deceleration);   
}          

/******************************************************//**
 * @brief Returns the device state
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval State ACCELERATING, DECELERATING, STEADY or INACTIVE for stepper motor,
                 STEADY or INACTIVE for Brush DC motor
 **********************************************************/
motorState_t BSP_MotorControl_GetDeviceState(uint8_t deviceId)
{
  motorState_t state = INACTIVE;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDeviceState != 0))
  {
    state = motorDrvHandle->GetDeviceState(deviceId);
  }      
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(9);
  }  
  return(state);   
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @param None
 * @retval BSP_MotorControl_FW_VERSION
 **********************************************************/
uint8_t BSP_MotorControl_GetFwVersion(void)
{
  uint8_t version = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetFwVersion != 0))
  {
    version = motorDrvHandle->GetFwVersion();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(10);
  }  
  return(version);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t BSP_MotorControl_GetMark(uint8_t deviceId)
{
  int32_t mark = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetMark != 0))
  {
    mark = motorDrvHandle->GetMark(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(11);
  }  
  return(mark);
}

/******************************************************//**
 * @brief  Returns the max speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval maxSpeed in pps for stepper motor
 *                  in % for Brush DC motor (0-100)
 **********************************************************/
uint16_t BSP_MotorControl_GetMaxSpeed(uint8_t deviceId)
{                                                  
  uint16_t maxSpeed = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetMaxSpeed != 0))
  {
    maxSpeed = motorDrvHandle->GetMaxSpeed(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(12);
  }    
  return(maxSpeed);
}

/******************************************************//**
 * @brief  Returns the min speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval minSpeed in pps
 **********************************************************/
uint16_t BSP_MotorControl_GetMinSpeed(uint8_t deviceId)
{                                                  
  uint16_t minSpeed = 0;

  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetMinSpeed != 0))
  {
    minSpeed = motorDrvHandle->GetMinSpeed(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(13);
  }    
  return(minSpeed);  
}                                                     

/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t BSP_MotorControl_GetPosition(uint8_t deviceId)
{
  int32_t pos = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetPosition != 0))
  {
    pos = motorDrvHandle->GetPosition(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(14);
  }      
  return(pos);
}

/******************************************************//**
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_GoHome(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GoHome != 0))
  {
    motorDrvHandle->GoHome(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(15);
  }      
} 
  
/******************************************************//**
 * @brief  Requests the motor to move to the mark position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_GoMark(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GoMark != 0))
  {
    motorDrvHandle->GoMark(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(16);
  }     
}

/******************************************************//**
 * @brief  Requests the motor to move to the specified position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void BSP_MotorControl_GoTo(uint8_t deviceId, int32_t targetPosition)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GoTo != 0))
  {
    motorDrvHandle->GoTo(deviceId, targetPosition);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(17);
  }      
}

/******************************************************//**
 * @brief  Immediatly stops the motor.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 **********************************************************/
void BSP_MotorControl_HardStop(uint8_t deviceId) 
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->HardStop != 0))
  {
    motorDrvHandle->HardStop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(18);
  }      
}

/******************************************************//**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void BSP_MotorControl_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->Move != 0))
  {
    motorDrvHandle->Move(deviceId, direction, stepCount);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(19);
  }      
}

/******************************************************//**
 * @brief Resets all motor driver devices
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControl_ResetAllDevices(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ResetAllDevices != 0))
  {
    motorDrvHandle->ResetAllDevices(); 
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(20);
  }      
}

/******************************************************//**
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the device acceleration.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note For unidirectionnal brush DC motor, direction parameter 
 * has no effect
 **********************************************************/
void BSP_MotorControl_Run(uint8_t deviceId, motorDir_t direction)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->Run != 0))
  {
    motorDrvHandle->Run(deviceId, direction); 
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(21);
  }      
}
/******************************************************//**
 * @brief  Changes the acceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool BSP_MotorControl_SetAcceleration(uint8_t deviceId,uint16_t newAcc)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetAcceleration != 0))
  {
    status = motorDrvHandle->SetAcceleration(deviceId, newAcc);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(22);
  }      
  return (status);
}            

/******************************************************//**
 * @brief  Changes the deceleration of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool BSP_MotorControl_SetDeceleration(uint8_t deviceId, uint16_t newDec)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDeceleration != 0))
  {
    status = motorDrvHandle->SetDeceleration(deviceId, newDec);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(23);
  }        
  return (status);
}        

/******************************************************//**
 * @brief  Set current position to be the Home position (ABS pos set to 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetHome(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetHome != 0))
  {
    motorDrvHandle->SetHome(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(24);
  }        
}
 
/******************************************************//**
 * @brief  Sets current position to be the Mark position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetMark(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetMark != 0))
  {
    motorDrvHandle->SetMark(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(25);
  }    
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] newMaxSpeed New max speed  to apply in pps for stepper motor,
              in % for Brush DC motor (0-100)                           
 * @retval true if the command is successfully executed, else false
 * @note For a stepper motor, the command is not performed if the device 
 * is executing a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool BSP_MotorControl_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetMaxSpeed != 0))
  {
    status = motorDrvHandle->SetMaxSpeed(deviceId, newMaxSpeed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(26);
  }     
  return (status);  
}                                                     

/******************************************************//**
 * @brief  Changes the min speed of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool BSP_MotorControl_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed)
{                                                  
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetMinSpeed != 0))
  {
    status = motorDrvHandle->SetMinSpeed(deviceId, newMinSpeed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(27);
  }     
  
  return (status);  
}                 

/******************************************************//**
 * @brief  Stops the motor by using the device deceleration
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool BSP_MotorControl_SoftStop(uint8_t deviceId)
{	
  bool status = FALSE;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SoftStop != 0))
  {
    status = motorDrvHandle->SoftStop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(28);
  }    
  return (status);  
}

/******************************************************//**
 * @brief  Handles the device state machine at each step
 * or at each tick
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void BSP_MotorControl_StepClockHandler(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->StepClockHandler != 0))
  {
    motorDrvHandle->StepClockHandler(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(29);
  }   
}
/******************************************************//**
 * @brief  Locks until the device state becomes Inactive
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
void BSP_MotorControl_WaitWhileActive(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->WaitWhileActive != 0))
  {
    motorDrvHandle->WaitWhileActive(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(30);
  }    
}

/**
  * @}
  */

/** @defgroup BSP_MotorControl_Control_Functions
  * @{
  */   

/******************************************************//**
 * @brief  Issue the Disable command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note For brush DC motor, when input of different brigdes are parallelized 
 * together, the disabling of one bridge leads to the disabling
 * of the second one
 **********************************************************/
void BSP_MotorControl_CmdDisable(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdDisable != 0))
  {
    motorDrvHandle->CmdDisable(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(31);
  }    
}

/******************************************************//**
 * @brief  Issues the Enable command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1) for stepper motor
 *            For L6208: dummy parameter for compatibility with motor.h
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note For brush DC motor, when input of different brigdes are parallelized 
 * together, the enabling of one bridge leads to the enabling
 * of the second one
 **********************************************************/
void BSP_MotorControl_CmdEnable(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdEnable != 0))
  {
    motorDrvHandle->CmdEnable(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(32);
  }      
}

/******************************************************//**
 * @brief  Issues the GetParam command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] param Register adress (BSP_MotorControl_ABS_POS, BSP_MotorControl_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t BSP_MotorControl_CmdGetParam(uint8_t deviceId,
                                      uint32_t param)
{
  uint32_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGetParam != 0))
  {
    value = motorDrvHandle->CmdGetParam(deviceId, param);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(33);
  }       
  return (value);
}

/******************************************************//**
 * @brief  Issues the GetStatus command to the motor driver of the specified 
 * device for stepper motor,
 * Get bridge status for Brush DC motor
 * @param[in] deviceId  from 0 to MAX_NUMBER_OF_DEVICES - 1 for stepper motor,
              bridgeId from 0 for bridge A, 1 for bridge B for brush DC motor
 * @retval Status Register value for stepper motor,
 *         Bridge state for brush DC motor
 * @note For stepper motor, once the GetStatus command is performed, 
 * the flags of the status register are reset. 
 * This is not the case when the status register is read with the
 * GetParam command (via the functions ReadStatusRegister or CmdGetParam).
 **********************************************************/
uint16_t BSP_MotorControl_CmdGetStatus(uint8_t deviceId)
{
  uint16_t status = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGetStatus != 0))
  {
    status = motorDrvHandle->CmdGetStatus(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(34);
  }      
  return (status);
}

/******************************************************//**
 * @brief  Issues the Nop command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdNop(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdNop != 0))
  {
    motorDrvHandle->CmdNop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(35);
  }   
}

/******************************************************//**
 * @brief  Issues the SetParam command to the motor driver of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] param Register adress (BSP_MotorControl_ABS_POS, BSP_MotorControl_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdSetParam(uint8_t deviceId,
                                   uint32_t param,
                                   uint32_t value)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSetParam != 0))
  {
    motorDrvHandle->CmdSetParam(deviceId, param, value);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(36);
  }     
}

/******************************************************//**
 * @brief  Reads the Status Register value
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with CmdGetStatus()
 **********************************************************/
uint16_t BSP_MotorControl_ReadStatusRegister(uint8_t deviceId)
{
  uint16_t status = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->ReadStatusRegister != 0))
  {
    status = motorDrvHandle->ReadStatusRegister(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(37);
  }   
  return (status);
}

/******************************************************//**
 * @brief  Releases the motor driver (pin set to High) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void BSP_MotorControl_ReleaseReset(void)
{ 
//  if ((motorDrvHandle != 0)&&(motorDrvHandle->ReleaseReset != 0))
//  {
	  A4985_ReleaseReset();
//    motorDrvHandle->ReleaseReset();
//  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(38);
  }   
}

/******************************************************//**
 * @brief  Resets the motor driver (reset pin set to low) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void BSP_MotorControl_Reset(void)
{
//  if ((motorDrvHandle != 0)&&(motorDrvHandle->Reset != 0))
//  {
	A4985_Reset();
//    motorDrvHandle->Reset();
//  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(39);
  }   
}

/******************************************************//**
 * @brief  Set the stepping mode 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum BSP_MotorControl_STEP_SEL_t
 * @retval None
 **********************************************************/
void BSP_MotorControl_SelectStepMode(uint8_t deviceId, motorStepMode_t stepMod)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SelectStepMode != 0))
  {
    motorDrvHandle->SelectStepMode(deviceId, stepMod);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(40);
  }   
}

/******************************************************//**
 * @brief  Specifies the direction 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device 
 * is in INACTIVE state
 * For L6208: In velocity mode a direction change forces the device to stop and 
 * then run in the new direction. In position mode, if the device is 
 * running, a direction change will generate an error.
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetDirection(uint8_t deviceId, motorDir_t dir)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDirection != 0))
  {
    motorDrvHandle->SetDirection(deviceId, dir);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(41);
  }     
}

/******************************************************//**
 * @brief Issues Go To Dir command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * For L6208: dummy parameter for compatibility with motor.h
 * @param[in] dir movement direction
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdGoToDir(uint8_t deviceId, motorDir_t dir, int32_t abs_pos)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGoToDir != 0))
  {
    motorDrvHandle->CmdGoToDir(deviceId, dir, abs_pos);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(42);
  }
}

/******************************************************//**
 * @brief Checks if at least one device is busy by checking 
 * busy pin position. 
 * The busy pin is shared between all devices.
 * @param None
 * @retval One if at least one device is busy, otherwise zero
 **********************************************************/
uint8_t BSP_MotorControl_CheckBusyHw(void)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CheckBusyHw != 0))
  {
    value = motorDrvHandle->CheckBusyHw();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(43);
  }
  return (value);
}

/******************************************************//**
 * @brief Checks if at least one device has an alarm flag set
 * by reading flag pin position.
 * The flag pin is shared between all devices.
 * @param None
 * @retval One if at least one device has an alarm flag set ,
 * otherwise zero
 **********************************************************/
uint8_t BSP_MotorControl_CheckStatusHw(void)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CheckStatusHw != 0))
  {
    value = motorDrvHandle->CheckStatusHw();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(44);
  }
  return (value);
}

/******************************************************//**
 * @brief Issues Go Until command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action ACTION_RESET or ACTION_COPY
 * @param[in] dir movement direction
 * @param[in] speed
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdGoUntil(uint8_t deviceId, motorAction_t action, motorDir_t dir, uint32_t speed)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdGoUntil != 0))
  {
    motorDrvHandle->CmdGoUntil(deviceId, action, dir, speed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(45);
  }
}

/******************************************************//**
 * @brief Immediately stops the motor and disable the power bridge.
 * @param[in] deviceId from 0 to MAX_NUMBER_OF_DEVICES-1  for stepper motor
 *            motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note if two Brush DC motors use the same power bridge, the 
 * power bridge will be disable only if the two motors are
 * stopped
 **********************************************************/
void BSP_MotorControl_CmdHardHiZ(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdHardHiZ != 0))
  {
    motorDrvHandle->CmdHardHiZ(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(46);
  }
}

/******************************************************//**
 * @brief Issues Release SW command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action
 * @param[in] dir movement direction
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdReleaseSw(uint8_t deviceId, motorAction_t action, motorDir_t dir)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdReleaseSw != 0))
  {
    motorDrvHandle->CmdReleaseSw(deviceId, action, dir);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(47);
  }
}

/******************************************************//**
 * @brief Issues Reset Device command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdResetDevice(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdResetDevice != 0))
  {
    motorDrvHandle->CmdResetDevice(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(48);
  }
}

/******************************************************//**
 * @brief Issues Reset Pos command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdResetPos(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdResetPos != 0))
  {
    motorDrvHandle->CmdResetPos(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(49);
  }
}

/******************************************************//**
 * @brief Issues Run command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] dir Movement direction (FORWARD, BACKWARD)
 * @param[in] speed in steps/s
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdRun(uint8_t deviceId, motorDir_t dir, uint32_t speed)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdRun != 0))
  {
    motorDrvHandle->CmdRun(deviceId, dir, speed);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(50);
  }
}

/******************************************************//**
 * @brief Issues Soft HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdSoftHiZ(uint8_t deviceId)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSoftHiZ != 0))
  {
    motorDrvHandle->CmdSoftHiZ(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(51);
  }
}

/******************************************************//**
 * @brief Issues Step Clock command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] dir Movement direction (FORWARD, BACKWARD)
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdStepClock(uint8_t deviceId, motorDir_t dir)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdStepClock != 0))
  {
    motorDrvHandle->CmdStepClock(deviceId, dir);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(52);
  }
}

/******************************************************//**
 * @brief Fetch and clear status flags of all devices 
 * by issuing a GET_STATUS command simultaneously  
 * to all devices.
 * Then, the fetched status of each device can be retrieved
 * by using the BSP_MotorControl_GetFetchedStatus function
 * provided there is no other calls to functions which 
 * use the SPI in between.
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControl_FetchAndClearAllStatus(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->FetchAndClearAllStatus != 0))
  {
    motorDrvHandle->FetchAndClearAllStatus();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(53);
  }
}

/******************************************************//**
 * @brief Get the value of the STATUS register which was 
 * fetched by using BSP_MotorControl_FetchAndClearAllStatus.
 * The fetched values are available as long as there is
 * no other calls to functions which use the SPI.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Last fetched value of the STATUS register
 **********************************************************/
uint16_t BSP_MotorControl_GetFetchedStatus(uint8_t deviceId)
{
  uint16_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetFetchedStatus != 0))
  {
    value = motorDrvHandle->GetFetchedStatus(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(54);
  }
  return (value);
}

/******************************************************//**
 * @brief Return the number of devices in the daisy chain 
 * @param None
 * @retval number of devices from 1 to MAX_NUMBER_OF_DEVICES
 **********************************************************/
uint8_t BSP_MotorControl_GetNbDevices(void)
{
  uint8_t value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetNbDevices != 0))
  {
    value = motorDrvHandle->GetNbDevices();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(55);
  }
  return (value);
}

/******************************************************//**
 * @brief Checks if the specified device is busy
 * by reading the Busy bit of its status Register
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval true if device is busy, else false
 **********************************************************/
bool BSP_MotorControl_IsDeviceBusy(uint8_t deviceId)
{
  bool value = 0;
  if ((motorDrvHandle != 0)&&(motorDrvHandle->IsDeviceBusy != 0))
  {
    value = motorDrvHandle->IsDeviceBusy(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(56);
  }
  return (value);
}

/******************************************************//**
 * @brief Sends commands stored in the queue by previously
 * BSP_MotorControl_QueueCommands
 * @param None
 * @retval None
 *********************************************************/
void BSP_MotorControl_SendQueuedCommands(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SendQueuedCommands != 0))
  {
    motorDrvHandle->SendQueuedCommands();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(57);
  }
}

/******************************************************//**
 * @brief Put commands in queue before synchronous sending
 * done by calling BSP_MotorControl_SendQueuedCommands.
 * Any call to functions that use the SPI between the calls of 
 * BSP_MotorControl_QueueCommands and BSP_MotorControl_SendQueuedCommands 
 * will corrupt the queue.
 * A command for each device of the daisy chain must be 
 * specified before calling BSP_MotorControl_SendQueuedCommands.
 * @param[in] deviceId deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to queue (all BSP_MotorControl commmands 
 * except SET_PARAM, GET_PARAM, GET_STATUS)
 * @param[in] value argument of the command to queue
 * @retval None
 *********************************************************/
void BSP_MotorControl_QueueCommands(uint8_t deviceId, uint8_t param, int32_t value)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->QueueCommands != 0))
  {
    motorDrvHandle->QueueCommands(deviceId, param, value);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(58);
  }
}

/******************************************************//**
 * @brief  Locks until all devices become not busy
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControl_WaitForAllDevicesNotBusy(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->WaitForAllDevicesNotBusy != 0))
  {
    motorDrvHandle->WaitForAllDevicesNotBusy();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(59);
  }  
}

/******************************************************//**
 * @brief Handler of the busy interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void BSP_MotorControl_BusyInterruptHandler(void)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->BusyInterruptHandler != 0))
  {
    motorDrvHandle->BusyInterruptHandler();
  }    
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(60);
  }  
}

/******************************************************//**
 * @brief Issues Soft Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void BSP_MotorControl_CmdSoftStop(uint8_t deviceId)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->CmdSoftStop != 0))
  {
    motorDrvHandle->CmdSoftStop(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(61);
  }    
}

/******************************************************//**
 * @brief  Start the step clock by using the given frequency
 * @param[in] newFreq in Hz of the step clock
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void BSP_MotorControl_StartStepClock(uint16_t newFreq)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->StartStepClock != 0))
  {
    motorDrvHandle->StartStepClock(newFreq);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(62);
  }    
}

/******************************************************//**
 * @brief  Stops the PWM uses for the step clock
 * @param  None
 * @retval None
 **********************************************************/
void BSP_MotorControl_StopStepClock(void)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->StopStepClock != 0))
  {
    motorDrvHandle->StopStepClock();
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(63);
  }    
}

/******************************************************//**
 * @brief Set the dual full bridge configuration
 * @param[in] config bridge configuration to apply from 
 * dualFullBridgeConfig_t enum
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetDualFullBridgeConfig(dualFullBridgeConfig_t config)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDualFullBridgeConfig != 0))
  {
    motorDrvHandle->SetDualFullBridgeConfig(config);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(64);
  }    
}

/******************************************************//**
 * @brief  Get the PWM frequency of the  bridge input
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B for brush DC motor
 * bridgeId must be 0 for L6208 (both bridges are set with the same frequency)
 * @retval Freq in Hz
 **********************************************************/
uint32_t BSP_MotorControl_GetBridgeInputPwmFreq(uint8_t bridgeId)
{	
  uint32_t pwmFreq = 0;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetBridgeInputPwmFreq != 0))
  {
    pwmFreq = motorDrvHandle->GetBridgeInputPwmFreq(bridgeId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(66);
  }    
  return (pwmFreq);
}

/******************************************************//**
 * @brief  Changes the PWM frequency of the  bridge input
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B for brush DC motor
 * bridgeId must be 0 for L6208 (both bridges are set with the same frequency)
 * @param[in] newFreq in Hz up to 100000Hz
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq)
{	
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetBridgeInputPwmFreq != 0))
  {
    motorDrvHandle->SetBridgeInputPwmFreq(bridgeId, newFreq);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(66);
  }    
}

/******************************************************//**
 * @brief Select the mode to stop the motor. When the motor
 * is stopped, if autoHiZ is TRUE, the power bridges are disabled
 * if autoHiZ is FALSE, the power bridges are kept enabled.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            deviceId dummy parameter for compatibility with motor.h
 * @param[in] stopMode selected stop mode
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetStopMode != 0))
  {
    motorDrvHandle->SetStopMode(deviceId, stopMode);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(67);
  } 
}

/******************************************************//**
 * @brief Get the selected stop mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval the selected stop mode
 **********************************************************/
motorStopMode_t BSP_MotorControl_GetStopMode(uint8_t deviceId)
{	
  motorStopMode_t stopMode = UNKNOW_STOP_MODE;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetStopMode != 0))
  {
    stopMode = motorDrvHandle->GetStopMode(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(68);
  }    
  return (stopMode);
}
 
/******************************************************//**
 * @brief Select the motor decay mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @param[in] decayMode (SLOW_DECAY or FAST_DECAY)
 * @retval None
 **********************************************************/
void BSP_MotorControl_SetDecayMode(uint8_t deviceId, motorDecayMode_t decayMode)
{
  if ((motorDrvHandle != 0)&&(motorDrvHandle->SetDecayMode != 0))
  {
    motorDrvHandle->SetDecayMode(deviceId, decayMode);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(69);
  } 
}
 
/******************************************************//**
 * @brief Get the motor decay mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval decay mode
 **********************************************************/
motorDecayMode_t BSP_MotorControl_GetDecayMode(uint8_t deviceId)
{	
  motorDecayMode_t decayMode = UNKNOW_DECAY;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDecayMode != 0))
  {
    decayMode = motorDrvHandle->GetDecayMode(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(70);
  }    
  return (decayMode);
}

/******************************************************//**
 * @brief Get the motor step mode
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval step mode
 **********************************************************/
motorStepMode_t BSP_MotorControl_GetStepMode(uint8_t deviceId)
{	
  motorStepMode_t stepMode = STEP_MODE_UNKNOW;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetStepMode != 0))
  {
    stepMode = motorDrvHandle->GetStepMode(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(71);
  }    
  return (stepMode);
}

/******************************************************//**
 * @brief Get the motor direction
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 *            For L6208: dummy parameter for compatibility with motor.h
 * @retval direction
 **********************************************************/
motorDir_t BSP_MotorControl_GetDirection(uint8_t deviceId)
{	
  motorDir_t dir = UNKNOW_DIR;
  
  if ((motorDrvHandle != 0)&&(motorDrvHandle->GetDirection != 0))
  {
    dir = motorDrvHandle->GetDirection(deviceId);
  }
  else
  {
    MOTOR_CONTROL_ERROR_UNDEFINED_FUNCTION(72);
  }    
  return (dir);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
