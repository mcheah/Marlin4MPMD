/**
  ******************************************************************************
  * @file    l6474.h 
  * @author  IPC Rennes
  * @version V1.5.0
  * @date    November 12, 2014
  * @brief   Header for L6474 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2014 STMicroelectronics
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
#ifndef __L6474_H
#define __L6474_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "l6474_target_config.h"
#include "motor.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup L6474
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup L6474_Exported_Constants
  * @{
  */   

/// Current FW version
#define L6474_FW_VERSION (5)

/// L6474 max number of bytes of command & arguments to set a parameter
#define L6474_CMD_ARG_MAX_NB_BYTES              (4)

/// L6474 command + argument bytes number for GET_STATUS command
#define L6474_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// L6474 response bytes number
#define L6474_RSP_NB_BYTES_GET_STATUS           (2)

/// L6474 value mask for ABS_POS register
#define L6474_ABS_POS_VALUE_MASK    ((uint32_t) 0x003FFFFF)
   
/// L6474 sign bit mask for ABS_POS register
#define L6474_ABS_POS_SIGN_BIT_MASK ((uint32_t) 0x00200000)

/**
  * @}
  */

/** @addtogroup L6474_Exported_Variables
  * @{
  */    
    extern motorDrv_t   l6474Drv;
/**
  * @}
  */
     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup L6474_Exported_Types
  * @{
  */   

/** @defgroup L6474_Fast_Decay_Time_Options  
  * @{
  */
///TOFF_FAST values for T_FAST register
typedef enum {
  L6474_TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
  L6474_TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
  L6474_TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
  L6474_TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
  L6474_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  L6474_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  L6474_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  L6474_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  L6474_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  L6474_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  L6474_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  L6474_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  L6474_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  L6474_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  L6474_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  L6474_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} L6474_TOFF_FAST_t;
/**
  * @}
  */

/** @defgroup L6474_Fall_Step_Time_Options 
  * @{
  */
///FAST_STEP values for T_FAST register
typedef enum {
  L6474_FAST_STEP_2us = ((uint8_t) 0x00),
  L6474_FAST_STEP_4us = ((uint8_t) 0x01),
  L6474_FAST_STEP_6us = ((uint8_t) 0x02),
  L6474_FAST_STEP_8us = ((uint8_t) 0x03),
  L6474_FAST_STEP_10us = ((uint8_t) 0x04),
  L6474_FAST_STEP_12us = ((uint8_t) 0x05),
  L6474_FAST_STEP_14us = ((uint8_t) 0x06),
  L6474_FAST_STEP_16us = ((uint8_t) 0x07),
  L6474_FAST_STEP_18us = ((uint8_t) 0x08),
  L6474_FAST_STEP_20us = ((uint8_t) 0x09),
  L6474_FAST_STEP_22us = ((uint8_t) 0x0A),
  L6474_FAST_STEP_24us = ((uint8_t) 0x0B),
  L6474_FAST_STEP_26us = ((uint8_t) 0x0C),
  L6474_FAST_STEP_28us = ((uint8_t) 0x0D),
  L6474_FAST_STEP_30us = ((uint8_t) 0x0E),
  L6474_FAST_STEP_32us = ((uint8_t) 0x0F)
} L6474_FAST_STEP_t;
/**
  * @}
  */

/** @defgroup L6474_Overcurrent_Threshold_options
  * @{
  */
///OCD_TH register
typedef enum {
  L6474_OCD_TH_375mA  = ((uint8_t) 0x00),
  L6474_OCD_TH_750mA  = ((uint8_t) 0x01),
  L6474_OCD_TH_1125mA = ((uint8_t) 0x02),
  L6474_OCD_TH_1500mA = ((uint8_t) 0x03),
  L6474_OCD_TH_1875mA = ((uint8_t) 0x04),
  L6474_OCD_TH_2250mA = ((uint8_t) 0x05),
  L6474_OCD_TH_2625mA = ((uint8_t) 0x06),
  L6474_OCD_TH_3000mA = ((uint8_t) 0x07),
  L6474_OCD_TH_3375mA = ((uint8_t) 0x08),
  L6474_OCD_TH_3750mA = ((uint8_t) 0x09),
  L6474_OCD_TH_4125mA = ((uint8_t) 0x0A),
  L6474_OCD_TH_4500mA = ((uint8_t) 0x0B),
  L6474_OCD_TH_4875mA = ((uint8_t) 0x0C),
  L6474_OCD_TH_5250mA = ((uint8_t) 0x0D),
  L6474_OCD_TH_5625mA = ((uint8_t) 0x0E),
  L6474_OCD_TH_6000mA = ((uint8_t) 0x0F)
} L6474_OCD_TH_t;
/**
  * @}
  */

/** @defgroup L6474_STEP_MODE_Register_Masks 
  * @{
  */  
///STEP_MODE register
typedef enum {
  L6474_STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
  L6474_STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
} L6474_STEP_MODE_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_STEP_SEL_Options_For_STEP_MODE_Register
  * @{
  */
///STEP_SEL field of STEP_MODE register
typedef enum {
  L6474_STEP_SEL_1    = ((uint8_t) 0x08),  //full step
  L6474_STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
  L6474_STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
  L6474_STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
  L6474_STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
} L6474_STEP_SEL_t;
/**
  * @}
  */

/** @defgroup L6474_SYNC_SEL_Options_For_STEP_MODE_Register 
  * @{
  */
///SYNC_SEL field of STEP_MODE register
typedef enum {
  L6474_SYNC_SEL_1_2    = ((uint8_t) 0x80),
  L6474_SYNC_SEL_1      = ((uint8_t) 0x90),
  L6474_SYNC_SEL_2      = ((uint8_t) 0xA0),
  L6474_SYNC_SEL_4      = ((uint8_t) 0xB0),
  L6474_SYNC_SEL_8      = ((uint8_t) 0xC0),
  L6474_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} L6474_SYNC_SEL_t;
/**
  * @}
  */

/** @defgroup L6474_ALARM_EN_Register_Options
  * @{
  */
///ALARM_EN register
typedef enum {
  L6474_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
  L6474_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
  L6474_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
  L6474_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
  L6474_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
  L6474_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} L6474_ALARM_EN_t;
/**
  * @}
  */

/** @defgroup L6474_CONFIG_Register_Masks
  * @{
  */
///CONFIG register
typedef enum {
  L6474_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
  L6474_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
  L6474_CONFIG_OC_SD    = ((uint16_t) 0x0080),
  L6474_CONFIG_POW_SR   = ((uint16_t) 0x0300),
  L6474_CONFIG_TOFF      = ((uint16_t) 0x7C00)
} L6474_CONFIG_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_Clock_Source_Options_For_CONFIG_Register
  * @{
  */
///Clock source option for CONFIG register
typedef enum {
  L6474_CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
  L6474_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
  L6474_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
  L6474_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
  L6474_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
  L6474_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
  L6474_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
  L6474_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
  L6474_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
  L6474_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
  L6474_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
  L6474_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} L6474_CONFIG_OSC_MGMT_t;
/**
  * @}
  */

/** @defgroup L6474_External_Torque_Regulation_Options_For_CONFIG_Register
  * @{
  */
///External Torque regulation options for CONFIG register
typedef enum {
  L6474_CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
  L6474_CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
} L6474_CONFIG_EN_TQREG_t;
/**
  * @}
  */

/** @defgroup L6474_Over_Current_Shutdown_Options_For_CONFIG_Register
  * @{
  */
///Over Current Shutdown options for CONFIG register
typedef enum {
  L6474_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
  L6474_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
} L6474_CONFIG_OC_SD_t;
/**
  * @}
  */

/** @defgroup L6474_Power_Bridge_Output_Slew_Rate_Options
  * @{
  */
/// POW_SR values for CONFIG register
typedef enum {
  L6474_CONFIG_SR_320V_us    =((uint16_t)0x0000),
  L6474_CONFIG_SR_075V_us    =((uint16_t)0x0100),
  L6474_CONFIG_SR_110V_us    =((uint16_t)0x0200),
  L6474_CONFIG_SR_260V_us    =((uint16_t)0x0300)
} L6474_CONFIG_POW_SR_t;
/**
  * @}
  */

/** @defgroup L6474_Off_Time_Options
  * @{
  */
/// TOFF values for CONFIG register
typedef enum {
  L6474_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  L6474_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  L6474_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  L6474_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  L6474_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  L6474_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  L6474_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  L6474_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  L6474_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  L6474_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
  L6474_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  L6474_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  L6474_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  L6474_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  L6474_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  L6474_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  L6474_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  L6474_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  L6474_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  L6474_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  L6474_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  L6474_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  L6474_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  L6474_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  L6474_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  L6474_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  L6474_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  L6474_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  L6474_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  L6474_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  L6474_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} L6474_CONFIG_TOFF_t;
/**
  * @}
  */

/** @defgroup L6474_STATUS_Register_Bit_Masks
  * @{
  */
///STATUS Register Bit Masks
typedef enum {
  L6474_STATUS_HIZ         = (((uint16_t) 0x0001)),
  L6474_STATUS_DIR         = (((uint16_t) 0x0010)),
  L6474_STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),
  L6474_STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),
  L6474_STATUS_UVLO        = (((uint16_t) 0x0200)),
  L6474_STATUS_TH_WRN      = (((uint16_t) 0x0400)),
  L6474_STATUS_TH_SD       = (((uint16_t) 0x0800)),
  L6474_STATUS_OCD         = (((uint16_t) 0x1000))
} L6474_STATUS_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_Direction_Field_Of_STATUS_Register
  * @{
  */  
///Diretion field of STATUS register
typedef enum {
  L6474_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  L6474_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} L6474_STATUS_DIR_t;
/**
  * @}
  */

/** @defgroup L6474_Internal_Register_Addresses
  * @{
  */
/// Internal L6474 register addresses
typedef enum {
  L6474_ABS_POS        = ((uint8_t) 0x01),
  L6474_EL_POS         = ((uint8_t) 0x02),
  L6474_MARK           = ((uint8_t) 0x03),
  L6474_RESERVED_REG01 = ((uint8_t) 0x04),
  L6474_RESERVED_REG02 = ((uint8_t) 0x05),
  L6474_RESERVED_REG03 = ((uint8_t) 0x06),
  L6474_RESERVED_REG04 = ((uint8_t) 0x07),
  L6474_RESERVED_REG05 = ((uint8_t) 0x08),
  L6474_RESERVED_REG06 = ((uint8_t) 0x15),
  L6474_TVAL           = ((uint8_t) 0x09),
  L6474_RESERVED_REG07 = ((uint8_t) 0x0A),
  L6474_RESERVED_REG08 = ((uint8_t) 0x0B),
  L6474_RESERVED_REG09 = ((uint8_t) 0x0C),
  L6474_RESERVED_REG10 = ((uint8_t) 0x0D),
  L6474_T_FAST         = ((uint8_t) 0x0E),
  L6474_TON_MIN        = ((uint8_t) 0x0F),
  L6474_TOFF_MIN       = ((uint8_t) 0x10),
  L6474_RESERVED_REG11 = ((uint8_t) 0x11),
  L6474_ADC_OUT        = ((uint8_t) 0x12),
  L6474_OCD_TH         = ((uint8_t) 0x13),
  L6474_RESERVED_REG12 = ((uint8_t) 0x14),
  L6474_STEP_MODE      = ((uint8_t) 0x16),
  L6474_ALARM_EN       = ((uint8_t) 0x17),
  L6474_CONFIG         = ((uint8_t) 0x18),
  L6474_STATUS         = ((uint8_t) 0x19),
  L6474_RESERVED_REG13 = ((uint8_t) 0x1A),
  L6474_RESERVED_REG14 = ((uint8_t) 0x1B),
  L6474_INEXISTENT_REG = ((uint8_t) 0x1F)
} L6474_Registers_t;
/**
  * @}
  */

/** @defgroup L6474_Command_Set
  * @{
  */
/// L6474 command set
typedef enum {
  L6474_NOP           = ((uint8_t) 0x00),
  L6474_SET_PARAM     = ((uint8_t) 0x00),
  L6474_GET_PARAM     = ((uint8_t) 0x20),
  L6474_ENABLE        = ((uint8_t) 0xB8),
  L6474_DISABLE       = ((uint8_t) 0xA8),
  L6474_GET_STATUS    = ((uint8_t) 0xD0),
  L6474_RESERVED_CMD1 = ((uint8_t) 0xEB),
  L6474_RESERVED_CMD2 = ((uint8_t) 0xF8)
} L6474_Commands_t;
/**
  * @}
  */


/** @defgroup Device_Commands
  * @{
  */
/// Device commands 
typedef enum {
  RUN_CMD, 
  MOVE_CMD, 
  SOFT_STOP_CMD, 
  NO_CMD
} deviceCommand_t;
/**
  * @}
  */


/** @defgroup Device_Parameters
  * @{
  */

/// Device Parameters Structure Type
typedef struct {
    /// accumulator used to store speed increase smaller than 1 pps
    volatile uint32_t accu;           
    /// Position in steps at the start of the goto or move commands
    volatile int32_t currentPosition; 
    /// position in step at the end of the accelerating phase
    volatile uint32_t endAccPos;      
    /// nb steps performed from the beggining of the goto or the move command
    volatile uint32_t relativePos;    
    /// position in step at the start of the decelerating phase
    volatile uint32_t startDecPos;    
    /// nb steps to perform for the goto or move commands
    volatile uint32_t stepsToTake;   
    
    /// acceleration in pps^2 
    volatile uint16_t acceleration;  
    /// deceleration in pps^2
    volatile uint16_t deceleration;  
    /// max speed in pps (speed use for goto or move command)
    volatile uint16_t maxSpeed;      
    /// min speed in pps
    volatile uint16_t minSpeed;      
    /// current speed in pps    
    volatile uint16_t speed;         
    
    /// command under execution
    volatile deviceCommand_t commandExecuted; 
    /// FORWARD or BACKWARD direction
    volatile motorDir_t direction;                 
    /// Current State of the device
    volatile motorState_t motionState;       
}deviceParams_t; 

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/


/** @defgroup L6474_Exported_Functions
  * @{
  */   

/** @defgroup Device_Control_Functions
  * @{
  */   
void L6474_AttachErrorHandler(void (*callback)(uint16_t));  //Attach a user callback to the error handler
void L6474_AttachFlagInterrupt(void (*callback)(void));     //Attach a user callback to the flag Interrupt
void L6474_Init(uint8_t nbDevices);                         //Start the L6474 library
uint16_t L6474_GetAcceleration(uint8_t deviceId);           //Return the acceleration in pps^2
uint16_t L6474_GetCurrentSpeed(uint8_t deviceId);           //Return the current speed in pps
uint16_t L6474_GetDeceleration(uint8_t deviceId);           //Return the deceleration in pps^2
motorState_t L6474_GetDeviceState(uint8_t deviceId);        //Return the device state
motorDrv_t* L6474_GetMotorHandle(void);                     //Return handle of the motor driver handle
uint8_t L6474_GetFwVersion(void);                           //Return the FW version
int32_t L6474_GetMark(uint8_t deviceId);                    //Return the mark position 
uint16_t L6474_GetMaxSpeed(uint8_t deviceId);               //Return the max speed in pps
uint16_t L6474_GetMinSpeed(uint8_t deviceId);               //Return the min speed in pps
int32_t L6474_GetPosition(uint8_t deviceId);                //Return the ABS_POSITION (32b signed)
void L6474_GoHome(uint8_t deviceId);                        //Move to the home position
void L6474_GoMark(uint8_t deviceId);                        //Move to the Mark position
void L6474_GoTo(uint8_t deviceId, int32_t targetPosition);  //Go to the specified position
void L6474_HardStop(uint8_t deviceId);                      //Stop the motor and disable the power bridge
void L6474_Move(uint8_t deviceId,                           //Move the motor of the specified number of steps
                motorDir_t direction,
                uint32_t stepCount);    
uint16_t L6474_ReadId(void);                                //Read Id to get driver instance
void L6474_ResetAllDevices(void);                           //Reset all L6474 devices
void L6474_Run(uint8_t deviceId, motorDir_t direction);     //Run the motor 
bool L6474_SetAcceleration(uint8_t deviceId,uint16_t newAcc); //Set the acceleration in pps^2
bool L6474_SetDeceleration(uint8_t deviceId,uint16_t newDec); //Set the deceleration in pps^2
void L6474_SetHome(uint8_t deviceId);                         //Set current position to be the home position
void L6474_SetMark(uint8_t deviceId);                         //Set current position to be the Markposition
bool L6474_SetMaxSpeed(uint8_t deviceId,uint16_t newMaxSpeed); //Set the max speed in pps
bool L6474_SetMinSpeed(uint8_t deviceId,uint16_t newMinSpeed); //Set the min speed in pps   
bool L6474_SoftStop(uint8_t deviceId);                         //Progressively stops the motor 
void L6474_WaitWhileActive(uint8_t deviceId);                  //Wait for the device state becomes Inactive
/**
  * @}
  */

/** @defgroup L6474_Control_Functions
  * @{
  */   
void L6474_CmdDisable(uint8_t deviceId);              //Send the L6474_DISABLE command
void L6474_CmdEnable(uint8_t deviceId);               //Send the L6474_ENABLE command
uint32_t L6474_CmdGetParam(uint8_t deviceId,          //Send the L6474_GET_PARAM command
                           uint32_t param);
uint16_t L6474_CmdGetStatus(uint8_t deviceId);        // Send the L6474_GET_STATUS command
void L6474_CmdNop(uint8_t deviceId);                  //Send the L6474_NOP command
void L6474_CmdSetParam(uint8_t deviceId,              //Send the L6474_SET_PARAM command
                       uint32_t param,       
                       uint32_t value);
uint16_t L6474_ReadStatusRegister(uint8_t deviceId);  // Read the L6474_STATUS register without
                                                        // clearing the flags
void L6474_Reset(void);                               //Set the L6474 reset pin 
void L6474_ReleaseReset(void);                        //Release the L6474 reset pin 
void L6474_SelectStepMode(uint8_t deviceId,           // Step mode selection
                          motorStepMode_t stepMod);     
void L6474_SetDirection(uint8_t deviceId,             //Set the L6474 direction pin
                        motorDir_t direction);      
/**
  * @}
  */

/** @defgroup MotorControl_Board_Linked_Functions
  * @{
  */   
///Delay of the requested number of milliseconds
void BSP_MotorControlBoard_Delay(uint32_t delay);         
///Enable Irq
void BSP_MotorControlBoard_EnableIrq(void);               
///Disable Irq
void BSP_MotorControlBoard_DisableIrq(void);              
///Initialise GPIOs used for L6474s
void BSP_MotorControlBoard_GpioInit(uint8_t nbDevices);   
///Set PWM_X frequency and start it
void BSP_MotorControlBoard_PwmXSetFreq(uint16_t newFreq); 
///Set PWM_Y frequency and start it  
void BSP_MotorControlBoard_PwmYSetFreq(uint16_t newFreq); 
///Set PWM_Z frequency and start it
void BSP_MotorControlBoard_PwmZSetFreq(uint16_t newFreq); 
///Set PWM_E1 frequency and start it
void BSP_MotorControlBoard_PwmE1SetFreq(uint16_t newFreq); 
///Set PWM_E2 frequency and start it
void BSP_MotorControlBoard_PwmE2SetFreq(uint16_t newFreq); 
///Set PWM_E3 frequency and start it
void BSP_MotorControlBoard_PwmE3SetFreq(uint16_t newFreq); 
///Init the PWM of the specified device
void BSP_MotorControlBoard_PwmInit(uint8_t deviceId);    
///Stop the PWM of the specified device
void BSP_MotorControlBoard_PwmStop(uint8_t deviceId);    
///Reset the L6474 reset pin 
void BSP_MotorControlBoard_ReleaseReset(void);           
///Set the L6474 reset pin 
void BSP_MotorControlBoard_Reset(void);                  
///Set direction GPIO
void BSP_MotorControlBoard_SetDirectionGpio(uint8_t deviceId, uint8_t gpioState); 
///Initialise the SPI used for L6474s
uint8_t BSP_MotorControlBoard_SpiInit(void);   
///Write bytes to the L6474s via SPI
uint8_t BSP_MotorControlBoard_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices); 
/**
  * @}
  */


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

#endif /* #ifndef __L6474_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
