/**
  ******************************************************************************
  * @file    A4985.h 
  * @author  IPC Rennes
  * @version V1.5.0
  * @date    November 12, 2014
  * @brief   Header for A4985 driver (fully integrated microstepping motor driver)
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
#ifndef __A4985_H
#define __A4985_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "A4985_target_config.h"
#include "motor.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup A4985
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup A4985_Exported_Constants
  * @{
  */   

/// Current FW version
#define A4985_FW_VERSION (5)

/// A4985 max number of bytes of command & arguments to set a parameter
#define A4985_CMD_ARG_MAX_NB_BYTES              (4)

/// A4985 command + argument bytes number for GET_STATUS command
#define A4985_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// A4985 response bytes number
#define A4985_RSP_NB_BYTES_GET_STATUS           (2)

/// A4985 value mask for ABS_POS register
#define A4985_ABS_POS_VALUE_MASK    ((uint32_t) 0x003FFFFF)
   
/// A4985 sign bit mask for ABS_POS register
#define A4985_ABS_POS_SIGN_BIT_MASK ((uint32_t) 0x00200000)

/**
  * @}
  */

/** @addtogroup A4985_Exported_Variables
  * @{
  */    
    extern motorDrv_t   A4985Drv;
/**
  * @}
  */
     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup A4985_Exported_Types
  * @{
  */   

/** @defgroup A4985_Fast_Decay_Time_Options  
  * @{
  */
///TOFF_FAST values for T_FAST register
typedef enum {
  A4985_TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
  A4985_TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
  A4985_TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
  A4985_TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
  A4985_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  A4985_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  A4985_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  A4985_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  A4985_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  A4985_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  A4985_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  A4985_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  A4985_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  A4985_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  A4985_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  A4985_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} A4985_TOFF_FAST_t;
/**
  * @}
  */

/** @defgroup A4985_Fall_Step_Time_Options 
  * @{
  */
///FAST_STEP values for T_FAST register
typedef enum {
  A4985_FAST_STEP_2us = ((uint8_t) 0x00),
  A4985_FAST_STEP_4us = ((uint8_t) 0x01),
  A4985_FAST_STEP_6us = ((uint8_t) 0x02),
  A4985_FAST_STEP_8us = ((uint8_t) 0x03),
  A4985_FAST_STEP_10us = ((uint8_t) 0x04),
  A4985_FAST_STEP_12us = ((uint8_t) 0x05),
  A4985_FAST_STEP_14us = ((uint8_t) 0x06),
  A4985_FAST_STEP_16us = ((uint8_t) 0x07),
  A4985_FAST_STEP_18us = ((uint8_t) 0x08),
  A4985_FAST_STEP_20us = ((uint8_t) 0x09),
  A4985_FAST_STEP_22us = ((uint8_t) 0x0A),
  A4985_FAST_STEP_24us = ((uint8_t) 0x0B),
  A4985_FAST_STEP_26us = ((uint8_t) 0x0C),
  A4985_FAST_STEP_28us = ((uint8_t) 0x0D),
  A4985_FAST_STEP_30us = ((uint8_t) 0x0E),
  A4985_FAST_STEP_32us = ((uint8_t) 0x0F)
} A4985_FAST_STEP_t;
/**
  * @}
  */

/** @defgroup A4985_Overcurrent_Threshold_options
  * @{
  */
///OCD_TH register
typedef enum {
  A4985_OCD_TH_375mA  = ((uint8_t) 0x00),
  A4985_OCD_TH_750mA  = ((uint8_t) 0x01),
  A4985_OCD_TH_1125mA = ((uint8_t) 0x02),
  A4985_OCD_TH_1500mA = ((uint8_t) 0x03),
  A4985_OCD_TH_1875mA = ((uint8_t) 0x04),
  A4985_OCD_TH_2250mA = ((uint8_t) 0x05),
  A4985_OCD_TH_2625mA = ((uint8_t) 0x06),
  A4985_OCD_TH_3000mA = ((uint8_t) 0x07),
  A4985_OCD_TH_3375mA = ((uint8_t) 0x08),
  A4985_OCD_TH_3750mA = ((uint8_t) 0x09),
  A4985_OCD_TH_4125mA = ((uint8_t) 0x0A),
  A4985_OCD_TH_4500mA = ((uint8_t) 0x0B),
  A4985_OCD_TH_4875mA = ((uint8_t) 0x0C),
  A4985_OCD_TH_5250mA = ((uint8_t) 0x0D),
  A4985_OCD_TH_5625mA = ((uint8_t) 0x0E),
  A4985_OCD_TH_6000mA = ((uint8_t) 0x0F)
} A4985_OCD_TH_t;
/**
  * @}
  */

/** @defgroup A4985_STEP_MODE_Register_Masks 
  * @{
  */  
///STEP_MODE register
typedef enum {
  A4985_STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
  A4985_STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
} A4985_STEP_MODE_Masks_t;
/**
  * @}
  */

/** @defgroup A4985_STEP_SEL_Options_For_STEP_MODE_Register
  * @{
  */
///STEP_SEL field of STEP_MODE register
typedef enum {
  A4985_STEP_SEL_1    = ((uint8_t) 0x08),  //full step
  A4985_STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
  A4985_STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
  A4985_STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
  A4985_STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
} A4985_STEP_SEL_t;
/**
  * @}
  */

/** @defgroup A4985_SYNC_SEL_Options_For_STEP_MODE_Register 
  * @{
  */
///SYNC_SEL field of STEP_MODE register
typedef enum {
  A4985_SYNC_SEL_1_2    = ((uint8_t) 0x80),
  A4985_SYNC_SEL_1      = ((uint8_t) 0x90),
  A4985_SYNC_SEL_2      = ((uint8_t) 0xA0),
  A4985_SYNC_SEL_4      = ((uint8_t) 0xB0),
  A4985_SYNC_SEL_8      = ((uint8_t) 0xC0),
  A4985_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} A4985_SYNC_SEL_t;
/**
  * @}
  */

/** @defgroup A4985_ALARM_EN_Register_Options
  * @{
  */
///ALARM_EN register
typedef enum {
  A4985_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
  A4985_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
  A4985_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
  A4985_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
  A4985_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
  A4985_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} A4985_ALARM_EN_t;
/**
  * @}
  */

/** @defgroup A4985_CONFIG_Register_Masks
  * @{
  */
///CONFIG register
typedef enum {
  A4985_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
  A4985_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
  A4985_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
  A4985_CONFIG_OC_SD    = ((uint16_t) 0x0080),
  A4985_CONFIG_POW_SR   = ((uint16_t) 0x0300),
  A4985_CONFIG_TOFF      = ((uint16_t) 0x7C00)
} A4985_CONFIG_Masks_t;
/**
  * @}
  */

/** @defgroup A4985_Clock_Source_Options_For_CONFIG_Register
  * @{
  */
///Clock source option for CONFIG register
typedef enum {
  A4985_CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
  A4985_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
  A4985_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
  A4985_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
  A4985_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
  A4985_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
  A4985_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
  A4985_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
  A4985_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
  A4985_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
  A4985_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
  A4985_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
  A4985_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} A4985_CONFIG_OSC_MGMT_t;
/**
  * @}
  */

/** @defgroup A4985_External_Torque_Regulation_Options_For_CONFIG_Register
  * @{
  */
///External Torque regulation options for CONFIG register
typedef enum {
  A4985_CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
  A4985_CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
} A4985_CONFIG_EN_TQREG_t;
/**
  * @}
  */

/** @defgroup A4985_Over_Current_Shutdown_Options_For_CONFIG_Register
  * @{
  */
///Over Current Shutdown options for CONFIG register
typedef enum {
  A4985_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
  A4985_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
} A4985_CONFIG_OC_SD_t;
/**
  * @}
  */

/** @defgroup A4985_Power_Bridge_Output_Slew_Rate_Options
  * @{
  */
/// POW_SR values for CONFIG register
typedef enum {
  A4985_CONFIG_SR_320V_us    =((uint16_t)0x0000),
  A4985_CONFIG_SR_075V_us    =((uint16_t)0x0100),
  A4985_CONFIG_SR_110V_us    =((uint16_t)0x0200),
  A4985_CONFIG_SR_260V_us    =((uint16_t)0x0300)
} A4985_CONFIG_POW_SR_t;
/**
  * @}
  */

/** @defgroup A4985_Off_Time_Options
  * @{
  */
/// TOFF values for CONFIG register
typedef enum {
  A4985_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  A4985_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  A4985_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  A4985_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  A4985_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  A4985_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  A4985_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  A4985_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  A4985_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  A4985_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
  A4985_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  A4985_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  A4985_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  A4985_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  A4985_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  A4985_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  A4985_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  A4985_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  A4985_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  A4985_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  A4985_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  A4985_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  A4985_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  A4985_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  A4985_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  A4985_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  A4985_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  A4985_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  A4985_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  A4985_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  A4985_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} A4985_CONFIG_TOFF_t;
/**
  * @}
  */

/** @defgroup A4985_STATUS_Register_Bit_Masks
  * @{
  */
///STATUS Register Bit Masks
typedef enum {
  A4985_STATUS_HIZ         = (((uint16_t) 0x0001)),
  A4985_STATUS_DIR         = (((uint16_t) 0x0010)),
  A4985_STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),
  A4985_STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),
  A4985_STATUS_UVLO        = (((uint16_t) 0x0200)),
  A4985_STATUS_TH_WRN      = (((uint16_t) 0x0400)),
  A4985_STATUS_TH_SD       = (((uint16_t) 0x0800)),
  A4985_STATUS_OCD         = (((uint16_t) 0x1000))
} A4985_STATUS_Masks_t;
/**
  * @}
  */

/** @defgroup A4985_Direction_Field_Of_STATUS_Register
  * @{
  */  
///Diretion field of STATUS register
typedef enum {
  A4985_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  A4985_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} A4985_STATUS_DIR_t;
/**
  * @}
  */

/** @defgroup A4985_Internal_Register_Addresses
  * @{
  */
/// Internal A4985 register addresses
typedef enum {
  A4985_ABS_POS        = ((uint8_t) 0x01),
  A4985_EL_POS         = ((uint8_t) 0x02),
  A4985_MARK           = ((uint8_t) 0x03),
  A4985_RESERVED_REG01 = ((uint8_t) 0x04),
  A4985_RESERVED_REG02 = ((uint8_t) 0x05),
  A4985_RESERVED_REG03 = ((uint8_t) 0x06),
  A4985_RESERVED_REG04 = ((uint8_t) 0x07),
  A4985_RESERVED_REG05 = ((uint8_t) 0x08),
  A4985_RESERVED_REG06 = ((uint8_t) 0x15),
  A4985_TVAL           = ((uint8_t) 0x09),
  A4985_RESERVED_REG07 = ((uint8_t) 0x0A),
  A4985_RESERVED_REG08 = ((uint8_t) 0x0B),
  A4985_RESERVED_REG09 = ((uint8_t) 0x0C),
  A4985_RESERVED_REG10 = ((uint8_t) 0x0D),
  A4985_T_FAST         = ((uint8_t) 0x0E),
  A4985_TON_MIN        = ((uint8_t) 0x0F),
  A4985_TOFF_MIN       = ((uint8_t) 0x10),
  A4985_RESERVED_REG11 = ((uint8_t) 0x11),
  A4985_ADC_OUT        = ((uint8_t) 0x12),
  A4985_OCD_TH         = ((uint8_t) 0x13),
  A4985_RESERVED_REG12 = ((uint8_t) 0x14),
  A4985_STEP_MODE      = ((uint8_t) 0x16),
  A4985_ALARM_EN       = ((uint8_t) 0x17),
  A4985_CONFIG         = ((uint8_t) 0x18),
  A4985_STATUS         = ((uint8_t) 0x19),
  A4985_RESERVED_REG13 = ((uint8_t) 0x1A),
  A4985_RESERVED_REG14 = ((uint8_t) 0x1B),
  A4985_INEXISTENT_REG = ((uint8_t) 0x1F)
} A4985_Registers_t;
/**
  * @}
  */

/** @defgroup A4985_Command_Set
  * @{
  */
/// A4985 command set
typedef enum {
  A4985_NOP           = ((uint8_t) 0x00),
  A4985_SET_PARAM     = ((uint8_t) 0x00),
  A4985_GET_PARAM     = ((uint8_t) 0x20),
  A4985_ENABLE        = ((uint8_t) 0xB8),
  A4985_DISABLE       = ((uint8_t) 0xA8),
  A4985_GET_STATUS    = ((uint8_t) 0xD0),
  A4985_RESERVED_CMD1 = ((uint8_t) 0xEB),
  A4985_RESERVED_CMD2 = ((uint8_t) 0xF8)
} A4985_Commands_t;
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


/** @defgroup A4985_Exported_Functions
  * @{
  */   

/** @defgroup Device_Control_Functions
  * @{
  */   
void A4985_AttachErrorHandler(void (*callback)(uint16_t));  //Attach a user callback to the error handler
void A4985_AttachFlagInterrupt(void (*callback)(void));     //Attach a user callback to the flag Interrupt
void A4985_Init(uint8_t nbDevices);                         //Start the A4985 library
uint16_t A4985_GetAcceleration(uint8_t deviceId);           //Return the acceleration in pps^2
uint16_t A4985_GetCurrentSpeed(uint8_t deviceId);           //Return the current speed in pps
uint16_t A4985_GetDeceleration(uint8_t deviceId);           //Return the deceleration in pps^2
motorState_t A4985_GetDeviceState(uint8_t deviceId);        //Return the device state
motorDrv_t* A4985_GetMotorHandle(void);                     //Return handle of the motor driver handle
uint8_t A4985_GetFwVersion(void);                           //Return the FW version
int32_t A4985_GetMark(uint8_t deviceId);                    //Return the mark position 
uint16_t A4985_GetMaxSpeed(uint8_t deviceId);               //Return the max speed in pps
uint16_t A4985_GetMinSpeed(uint8_t deviceId);               //Return the min speed in pps
int32_t A4985_GetPosition(uint8_t deviceId);                //Return the ABS_POSITION (32b signed)
void A4985_GoHome(uint8_t deviceId);                        //Move to the home position
void A4985_GoMark(uint8_t deviceId);                        //Move to the Mark position
void A4985_GoTo(uint8_t deviceId, int32_t targetPosition);  //Go to the specified position
void A4985_HardStop(uint8_t deviceId);                      //Stop the motor and disable the power bridge
void A4985_Move(uint8_t deviceId,                           //Move the motor of the specified number of steps
                motorDir_t direction,
                uint32_t stepCount);    
uint16_t A4985_ReadId(void);                                //Read Id to get driver instance
void A4985_ResetAllDevices(void);                           //Reset all A4985 devices
void A4985_Run(uint8_t deviceId, motorDir_t direction);     //Run the motor 
bool A4985_SetAcceleration(uint8_t deviceId,uint16_t newAcc); //Set the acceleration in pps^2
bool A4985_SetDeceleration(uint8_t deviceId,uint16_t newDec); //Set the deceleration in pps^2
void A4985_SetHome(uint8_t deviceId);                         //Set current position to be the home position
void A4985_SetMark(uint8_t deviceId);                         //Set current position to be the Markposition
bool A4985_SetMaxSpeed(uint8_t deviceId,uint16_t newMaxSpeed); //Set the max speed in pps
bool A4985_SetMinSpeed(uint8_t deviceId,uint16_t newMinSpeed); //Set the min speed in pps   
bool A4985_SoftStop(uint8_t deviceId);                         //Progressively stops the motor 
void A4985_WaitWhileActive(uint8_t deviceId);                  //Wait for the device state becomes Inactive
/**
  * @}
  */

/** @defgroup A4985_Control_Functions
  * @{
  */   
void A4985_CmdDisable(uint8_t deviceId);              //Send the A4985_DISABLE command
void A4985_CmdEnable(uint8_t deviceId);               //Send the A4985_ENABLE command
//uint32_t A4985_CmdGetParam(uint8_t deviceId,          //Send the A4985_GET_PARAM command
//                           uint32_t param);
//uint16_t A4985_CmdGetStatus(uint8_t deviceId);        // Send the A4985_GET_STATUS command
void A4985_CmdNop(uint8_t deviceId);                  //Send the A4985_NOP command
//void A4985_CmdSetParam(uint8_t deviceId,              //Send the A4985_SET_PARAM command
//                       uint32_t param,
//                       uint32_t value);
//uint16_t A4985_ReadStatusRegister(uint8_t deviceId);  // Read the A4985_STATUS register without
//                                                        // clearing the flags
void A4985_Reset(void);                               //Set the A4985 reset pin 
void A4985_ReleaseReset(void);                        //Release the A4985 reset pin 
void A4985_SelectStepMode(uint8_t deviceId,           // Step mode selection
                          motorStepMode_t stepMod);
void A4985_SetDirection(uint8_t deviceId,             //Set the A4985 direction pin
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
///Initialise GPIOs used for A4985s
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
///Reset the A4985 reset pin 
void BSP_MotorControlBoard_ReleaseReset(void);           
///Set the A4985 reset pin 
void BSP_MotorControlBoard_Reset(void);                  
///Set direction GPIO
void BSP_MotorControlBoard_SetDirectionGpio(uint8_t deviceId, uint8_t gpioState); 
///Initialise the SPI used for A4985s
//uint8_t BSP_MotorControlBoard_SpiInit(void);
///Write bytes to the A4985s via SPI
//uint8_t BSP_MotorControlBoard_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices);
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

#endif /* #ifndef __A4985_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
