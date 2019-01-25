/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MARLIN_CONFIG_H
#define MARLIN_CONFIG_H


//#include "fastio.h"
#include "Configuration_STM.h"

#include "macros.h"
#include "boards.h"
#include "Version.h"
#include "Configuration.h"
#include "Conditionals_LCD.h"
#include "Configuration_adv.h"
#include "pins.h"
#ifndef USBCON
  #define HardwareSerial_h // trick to disable the standard HWserial
#endif

//#include "Arduino.h"

// STM32 Platform-specific includes
#include "Marlin_export.h"
#ifdef STM32_MPMD
#include "stm32f0xx_hal.h"
#elif defined(STM32_LERDGEX)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_lerdgex.h"

#ifdef __cplusplus
 extern "C" {
#endif
#include "lcd.h"
#ifdef __cplusplus
}
#endif //__cplusplus

#include "stm32f0xx_3dprinter_misc.h"

#include "stm32f0xx_3dprinter_uart.h"
#include "stm32f0xx_3dprinter_cdc.h"

#include "stm32f0xx_3dprinter_adc.h"
#endif //STM32_MPMD
#include "stm32f0xx_3dprinter_motor.h"
#include "motorcontrol.h"
#include "ff.h" /* for FATS and FIL*/
#include "arm_math.h"
// -STM32

#include "Conditionals_post.h"
#include "SanityCheck.h"

#endif // MARLIN_CONFIG_H
