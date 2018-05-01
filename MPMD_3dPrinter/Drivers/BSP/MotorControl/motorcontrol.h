/**
  ******************************************************************************
  * @file    motorcontrol.h
  * @author  IPC Rennes
  * @version V1.1.1
  * @date    15-January-2015
  * @brief   This file provides common definitions for motor control 
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
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup MOTOR_CONTROL
  * @{
  */

/** @defgroup MOTOR_CONTROL_Exported_Types
  * @{
  */



/**
  * @}
  */

/** @defgroup MOTOR_CONTROL_Exported_Constants
  * @{
  */
///Motor control board id for L6474
#define BSP_MOTOR_CONTROL_BOARD_ID_L6474  (6474)
///Motor control board id for Powerstep01
#define BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01 (0001)
///Motor control board id for L6206
#define BSP_MOTOR_CONTROL_BOARD_ID_L6206 (6206)
///Motor control board id for L6208
#define BSP_MOTOR_CONTROL_BOARD_ID_L6208 (6208)
///Motor control board id for A4985
#define BSP_MOTOR_CONTROL_BOARD_ID_A4985 (4985)
/**
  * @}
  */


/** @defgroup MOTOR_CONTROL_Exported_Macros
  * @{
  */
#if  defined ( __GNUC__ )
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
#endif /* __GNUC__ */
/**
  * @}
  */

/** @defgroup MOTOR_CONTROL_Weak_Function_Prototypes
  * @{
  */
__weak motorDrv_t* L6474_GetMotorHandle(void);
__weak motorDrv_t* Powerstep01_GetMotorHandle(void);
__weak motorDrv_t* L6206_GetMotorHandle(void);
__weak motorDrv_t* L6208_GetMotorHandle(void); 
__weak motorDrv_t* A4985_GetMotorHandle(void);

/**
  * @}
  */   
   
/** @defgroup MOTOR_CONTROL_Exported_Functions
  * @{
  */
void BSP_MotorControl_AttachErrorHandler(void (*callback)(uint16_t));
void BSP_MotorControl_AttachFlagInterrupt(void (*callback)(void));
void BSP_MotorControl_AttachBusyInterrupt(void (*callback)(void));
void BSP_MotorControl_ErrorHandler(uint16_t error);
void BSP_MotorControl_Init(uint16_t  id, uint8_t nbDevices); 
void BSP_MotorControl_FlagInterruptHandler(void);
uint16_t BSP_MotorControl_GetAcceleration(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetBoardId(void);
uint16_t BSP_MotorControl_GetCurrentSpeed(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetDeceleration(uint8_t deviceId); 
motorState_t BSP_MotorControl_GetDeviceState(uint8_t deviceId); 
uint8_t BSP_MotorControl_GetFwVersion(void); 
int32_t BSP_MotorControl_GetMark(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetMaxSpeed(uint8_t deviceId); 
uint16_t BSP_MotorControl_GetMinSpeed(uint8_t deviceId); 
int32_t BSP_MotorControl_GetPosition(uint8_t deviceId); 
void BSP_MotorControl_GoHome(uint8_t deviceId); 
void BSP_MotorControl_GoMark(uint8_t deviceId); 
void BSP_MotorControl_GoTo(uint8_t deviceId, int32_t targetPosition); 
void BSP_MotorControl_HardStop(uint8_t deviceId); 
void BSP_MotorControl_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount); 
void BSP_MotorControl_ResetAllDevices(void); 
void BSP_MotorControl_Run(uint8_t deviceId, motorDir_t direction); 
bool BSP_MotorControl_SetAcceleration(uint8_t deviceId,uint16_t newAcc); 
bool BSP_MotorControl_SetDeceleration(uint8_t deviceId, uint16_t newDec); 
void BSP_MotorControl_SetHome(uint8_t deviceId); 
void BSP_MotorControl_SetMark(uint8_t deviceId); 
bool BSP_MotorControl_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed); 
bool BSP_MotorControl_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed); 
bool BSP_MotorControl_SoftStop(uint8_t deviceId); 
void BSP_MotorControl_StepClockHandler(uint8_t deviceId); 
void BSP_MotorControl_WaitWhileActive(uint8_t deviceId); 
void BSP_MotorControl_CmdDisable(uint8_t deviceId); 
void BSP_MotorControl_CmdEnable(uint8_t deviceId); 
uint32_t BSP_MotorControl_CmdGetParam(uint8_t deviceId, uint32_t param); 
uint16_t BSP_MotorControl_CmdGetStatus(uint8_t deviceId);
void BSP_MotorControl_CmdNop(uint8_t deviceId); 
void BSP_MotorControl_CmdSetParam(uint8_t deviceId, uint32_t param, uint32_t value);
uint16_t BSP_MotorControl_ReadStatusRegister(uint8_t deviceId); 
void BSP_MotorControl_ReleaseReset(void) ;
void BSP_MotorControl_Reset(void); 
void BSP_MotorControl_SelectStepMode(uint8_t deviceId,  motorStepMode_t stepMod); 
void BSP_MotorControl_SetDirection(uint8_t deviceId, motorDir_t dir);
void BSP_MotorControl_CmdGoToDir(uint8_t deviceId, motorDir_t dir, int32_t abs_pos);
uint8_t BSP_MotorControl_CheckBusyHw(void);
uint8_t BSP_MotorControl_CheckStatusHw(void);
void BSP_MotorControl_CmdGoUntil(uint8_t deviceId, motorAction_t action, motorDir_t dir, uint32_t speed);
void BSP_MotorControl_CmdHardHiZ(uint8_t deviceId);
void BSP_MotorControl_CmdReleaseSw(uint8_t deviceId, motorAction_t action, motorDir_t dir);
void BSP_MotorControl_CmdResetDevice(uint8_t deviceId);
void BSP_MotorControl_CmdResetPos(uint8_t deviceId);
void BSP_MotorControl_CmdRun(uint8_t deviceId, motorDir_t dir, uint32_t speed);
void BSP_MotorControl_CmdSoftHiZ(uint8_t deviceId);
void BSP_MotorControl_CmdStepClock(uint8_t deviceId, motorDir_t dir);
void BSP_MotorControl_FetchAndClearAllStatus(void);
uint16_t BSP_MotorControl_GetFetchedStatus(uint8_t deviceId);
uint8_t BSP_MotorControl_GetNbDevices(void);
bool BSP_MotorControl_IsDeviceBusy(uint8_t deviceId);
void BSP_MotorControl_SendQueuedCommands(void);
void BSP_MotorControl_QueueCommands(uint8_t deviceId, uint8_t param, int32_t value);
void BSP_MotorControl_WaitForAllDevicesNotBusy(void);
void BSP_MotorControl_BusyInterruptHandler(void);
void BSP_MotorControl_CmdSoftStop(uint8_t deviceId);
void BSP_MotorControl_StartStepClock(uint16_t newFreq);
void BSP_MotorControl_StopStepClock(void);
void BSP_MotorControl_SetDualFullBridgeConfig(dualFullBridgeConfig_t config);
uint32_t BSP_MotorControl_GetBridgeInputPwmFreq(uint8_t bridgeId);
void BSP_MotorControl_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);
void BSP_MotorControl_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode);
motorStopMode_t BSP_MotorControl_GetStopMode(uint8_t deviceId);
void BSP_MotorControl_SetDecayMode(uint8_t deviceId, motorDecayMode_t decayMode);
motorDecayMode_t BSP_MotorControl_GetDecayMode(uint8_t deviceId);
motorStepMode_t BSP_MotorControl_GetStepMode(uint8_t deviceId);
motorDir_t BSP_MotorControl_GetDirection(uint8_t deviceId);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* __MOTOR_CONTROL_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
