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

#ifndef CONFIGURATION_STORE_H
#define CONFIGURATION_STORE_H

#include "MarlinConfig.h"
#define MAX_EXTRUDERS 2
#define MAX_MESH_LEVELING_POINTS 10
#if ENABLED(SD_SETTINGS)
  #include "cardreader.h"
  //extern CardReader *p_card;
#endif
#define CONFIG_API_MAJOR_VERSION 1
#define CONFIG_API_MINOR_VERSION 0
//data writes must be 16-bit aligned
typedef struct ConfigSettings {
	uint8_t API_MAJOR_VERSION;
	uint8_t API_MINOR_VERSION;
	uint8_t MAJOR_FW_VERSION;
	uint8_t MINOR_FW_VERSION;
	uint8_t MAJOR_FW_SUBVERSION;
	uint8_t MINOR_FW_SUBVERSION;
	float axis_steps_per_mm[NUM_AXIS];
	float max_feedrate_mm_s[NUM_AXIS];
	uint32_t max_acceleration_mm_per_s2[NUM_AXIS];
	float acceleration;
	float retract_acceleration;
	float travel_acceleration;
	float min_feedrate_mm_s;
	float min_travel_feedrate_mm_s;
	uint32_t min_segment_time;
	float max_xy_jerk;
	float max_z_jerk;
	float max_e_jerk;
	float home_offset[3];
	float zprobe_zoffset;
	float endstop_adj[3];
	float delta_height;
	float delta_diagonal_rod;
	float delta_radius;
	float delta_segments_per_second;
	float delta_diagonal_rod_trim[3];
	float delta_radius_trim[3];
	float delta_angle_trim[3];
	float preheatHotendTemp[2];
	float preheatBedTemp[2];
	float preheatFanSpeed[2];
	float kP;
	float kI;
	float kD;
	uint16_t volumetric_enabled;
	float filament_size[MAX_EXTRUDERS]; //change if number of extruders changes
	float delta_grid_spacing[2];
	float bed_level[MAX_MESH_LEVELING_POINTS][MAX_MESH_LEVELING_POINTS]; //leave room for 10x10 mesh grid if we decide to increase number of points
} ConfigSettings;

void Config_ResetDefault();

#if DISABLED(DISABLE_M503)
  void Config_PrintSettings(bool forReplay=false);
#else
  FORCE_INLINE void Config_PrintSettings(bool forReplay=false) {}
#endif

#if ENABLED(EEPROM_SETTINGS) || ENABLED(SD_SETTINGS) || ENABLED(FLASH_SETTINGS)
  void Config_StoreSettings();
  void Config_RetrieveSettings();
#else
  FORCE_INLINE void Config_StoreSettings() {}
  FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif
  static FORCE_INLINE void _EEPROM_erase() {
  	BSP_MiscEEPROMErase();
  }

  static FORCE_INLINE void _EEPROMWrite(volatile void *destination, float value)
  {
  	BSP_MiscEEPROMWriteF32((void *)destination,value);
  }

  static FORCE_INLINE void _EEPROMWrite(volatile void *destination, uint32_t value)
  {
  	BSP_MiscEEPROMWriteU32((void *)destination,value);
  }

  static FORCE_INLINE void _EEPROMWrite(volatile void *destination, uint16_t value)
  {
  	BSP_MiscEEPROMWriteU16((void *)destination,value);
  }
#define _EEPROMRead(src,dest) (dest = *src)
#endif //CONFIGURATION_STORE_H
