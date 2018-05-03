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

/**
 * ST Microelectronics EVALR3DPRINT_V1 pin assignments
 */

#ifndef BOARD_NAME
  #define BOARD_NAME "STM EVALR3DPRINT"
#endif


#define LARGE_FLASH true

#define X_STEP_PIN         0
#define X_DIR_PIN          1
#define X_ENABLE_PIN       -1
#define X_MIN_PIN          3
#define X_MAX_PIN          -1

#define Y_STEP_PIN         5
#define Y_DIR_PIN          6
#define Y_ENABLE_PIN       -1
#define Y_MIN_PIN          -1
#define Y_MAX_PIN          8


#define Z_STEP_PIN         10
#define Z_DIR_PIN          11
#define Z_ENABLE_PIN       -1
#define Z_MIN_PIN          13
#define Z_MAX_PIN          -1

#define Z_MIN_PROBE_PIN	   13

//#define Y2_STEP_PIN        -1
//#define Y2_DIR_PIN         -1
//#define Y2_ENABLE_PIN      -1

//#define Z2_STEP_PIN        -1
//#define Z2_DIR_PIN         -1
//#define Z2_ENABLE_PIN      -1

#define E0_STEP_PIN        21
#define E0_DIR_PIN         22
#define E0_ENABLE_PIN      -1

//#define E1_STEP_PIN        24
//#define E1_DIR_PIN         25
//#define E1_ENABLE_PIN      -1

#define SDPOWER            -1
#define SDSS               -1
#define LED_PIN            -1
 
#define FAN_PIN            30 // (Sprinter config)

#define PS_ON_PIN          -1
  
#define KILL_PIN           -1

#define HEATER_0_PIN       33   // EXTRUDER 1
//#define HEATER_1_PIN       34
//#define HEATER_2_PIN       35
 
#define TEMP_0_PIN         36   // ANALOG NUMBERING
//#define TEMP_1_PIN         37   // ANALOG NUMBERING
//#define TEMP_2_PIN         38   // ANALOG NUMBERING

#define HEATER_BED_PIN     39    // BED

#define TEMP_BED_PIN       40   // ANALOG NUMBERING

#ifdef NUM_SERVOS
  #define SERVO0_PIN         41

  #if NUM_SERVOS > 1
    #define SERVO1_PIN         42
  #endif

  #if NUM_SERVOS > 2
    #define SERVO2_PIN         43
  #endif

  #if NUM_SERVOS > 3
    #define SERVO3_PIN         44
  #endif
#endif

//#define E2_STEP_PIN        45
//#define E2_DIR_PIN         46
//#define E2_ENABLE_PIN      -1

//#define U_MIN_PIN          -1
//#define V_MIN_PIN          -1
//#define W_MIN_PIN          -1

//#define HEATER_BED2_PIN    51    // BED2
//#define HEATER_BED3_PIN    52    // BED3


