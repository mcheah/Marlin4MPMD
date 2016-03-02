#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

#ifdef EEPROM_SETTINGS
void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}

#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

#endif // #ifdef EEPROM_SETTINGS
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.

#define EEPROM_VERSION "V13"

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homing);
  #ifdef DELTA
  EEPROM_WRITE_VAR(i,endstop_adj);
  EEPROM_WRITE_VAR(i,delta_radius);
  EEPROM_WRITE_VAR(i,delta_diagonal_rod);
  EEPROM_WRITE_VAR(i,delta_segments_per_second);
  #endif
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,zprobe_zoffset);
  #ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
  #else
		float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
  #endif
  #ifndef DOGLCD
    int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i,lcd_contrast);
  #ifdef SCARA
  EEPROM_WRITE_VAR(i,axis_scaling);        // Add scaling for SCARA
  #endif
  #ifdef FWRETRACT
  EEPROM_WRITE_VAR(i,autoretract_enabled);
  EEPROM_WRITE_VAR(i,retract_length);
  #if EXTRUDERS > 1
  EEPROM_WRITE_VAR(i,retract_length_swap);
  #endif
  EEPROM_WRITE_VAR(i,retract_feedrate);
  EEPROM_WRITE_VAR(i,retract_zlift);
  EEPROM_WRITE_VAR(i,retract_recover_length);
  #if EXTRUDERS > 1
  EEPROM_WRITE_VAR(i,retract_recover_length_swap);
  #endif
  EEPROM_WRITE_VAR(i,retract_recover_feedrate);
  #endif

  // Save filament sizes
  EEPROM_WRITE_VAR(i, volumetric_enabled);
  EEPROM_WRITE_VAR(i, filament_size[0]);
  #if EXTRUDERS > 1
  EEPROM_WRITE_VAR(i, filament_size[1]);
  #if EXTRUDERS > 2
  EEPROM_WRITE_VAR(i, filament_size[2]);
  #endif
  #endif
  
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#elif defined(SD_SETTINGS)
void Config_StoreSettings() 
{
  char cmdStr[MAX_CMD_SIZE+MAX_COMMENT_SIZE];
  char numStr[16];
  
  p_card->openFile((char *)CONFIG_FILE_NAME,false);
  
  /* Steps per unit */
  strcpy(cmdStr,"M92 X");
  sprintf(numStr, "%.2f", axis_steps_per_unit[X_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " Y");  
  sprintf(numStr, "%.2f", axis_steps_per_unit[Y_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " Z");
  sprintf(numStr, "%.2f", axis_steps_per_unit[Z_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " E");
  sprintf(numStr, "%.2f", axis_steps_per_unit[E_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; Steps per unit");   
  p_card->write_command(cmdStr);

#ifdef SCARA
  /* Scaling factors */
  strcpy(cmdStr," M365 X");
  sprintf(numStr, "%.2f", axis_scaling[X_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " Y");
  sprintf(numStr, "%.2f", axis_scaling[Y_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Z");
  sprintf(numStr, "%.2f", axis_scaling[Z_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; Scaling factors");   
  p_card->write_command(cmdStr);
#endif
  
  /* Maximum feedrates (mm/s) */
  strcpy(cmdStr,"M203 X");
  sprintf(numStr, "%.2f", max_feedrate[X_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Y");  
  sprintf(numStr, "%.2f", max_feedrate[Y_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " Z");
  sprintf(numStr, "%.2f", max_feedrate[Z_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " E");
  sprintf(numStr, "%.2f", max_feedrate[E_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; Maximum feedrates (mm/s)");   
  p_card->write_command(cmdStr);

  /* Maximum Acceleration (mm/s2) */
  strcpy(cmdStr,"M201 X");
  sprintf(numStr, "%lu", max_acceleration_units_per_sq_second[X_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Y");  
  sprintf(numStr, "%lu", max_acceleration_units_per_sq_second[Y_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " Z");
  sprintf(numStr, "%lu", max_acceleration_units_per_sq_second[Z_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " E");
  sprintf(numStr, "%lu", max_acceleration_units_per_sq_second[E_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; Maximum Acceleration (mm/s2)");  
  p_card->write_command(cmdStr);

  /* S=acceleration, T=retract acceleration */
  strcpy(cmdStr,"M204 S");  
  sprintf(numStr, "%.2f", acceleration);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " T");
  sprintf(numStr, "%.2f", retract_acceleration);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; S=acceleration, T=retract acceleration");
  p_card->write_command(cmdStr);

  /* S=Min feedrate (mm/s),        */
  /* T=Min travel feedrate (mm/s), */
  /* B=minimum segment time (ms),  */
  /* X=maximum XY jerk (mm/s),     */
  /* Z=maximum Z jerk (mm/s),      */
  /* E=maximum E jerk (mm/s)       */
  strcpy(cmdStr,"M205 S");  
  sprintf(numStr, "%.2f", minimumfeedrate);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " T");
  sprintf(numStr, "%.2f", mintravelfeedrate);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " B");
  sprintf(numStr, "%lu", minsegmenttime);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " X");
  sprintf(numStr, "%.2f", max_xy_jerk);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Z");
  sprintf(numStr, "%.2f", max_z_jerk);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " E");
  sprintf(numStr, "%.2f", max_e_jerk);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
  p_card->write_command(cmdStr);
  
  /* Home offset (mm) */
  strcpy(cmdStr,"M206 X");
  sprintf(numStr, "%.2f", add_homing[X_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Y"); 
  sprintf(numStr, "%.2f", add_homing[Y_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Z"); 
  sprintf(numStr, "%.2f", add_homing[Z_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " ; Home offset (mm)");
  p_card->write_command(cmdStr);

#ifdef DELTA
  /* Endstop adjustement (mm) */
  strcpy(cmdStr,"M666 X");
  sprintf(numStr, "%.2f", endstop_adj[X_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Y"); 
  sprintf(numStr, "%.2f", endstop_adj[Y_AXIS]);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Z"); 
  sprintf(numStr, "%.2f", endstop_adj[Z_AXIS]);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " ; Endstop adjustement (mm)");
  p_card->write_command(cmdStr);
  
  /* L=delta_diagonal_rod, R=delta_radius, S=delta_segments_per_second */
  strcpy(cmdStr,"M665 L");
  sprintf(numStr, "%.2f", delta_diagonal_rod);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " R"); 
  sprintf(numStr, "%.2f", delta_radius);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " S"); 
  sprintf(numStr, "%.2f", delta_segments_per_second);
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " ; L=delta_diagonal_rod, R=delta_radius, S=delta_segments_per_second");
  p_card->write_command(cmdStr);
#endif

#ifdef PIDTEMP  
  /* PID settings */
  strcpy(cmdStr,"M301 P");
  sprintf(numStr, "%.2f", Kp); 
  strcat(cmdStr, numStr);
  strcat(cmdStr, " I");
  sprintf(numStr, "%.2f", unscalePID_i(Ki)); 
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " D");
  sprintf(numStr, "%.2f", unscalePID_d(Kd)); 
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; PID settings");
  p_card->write_command(cmdStr);
#endif

#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
  strcpy(cmdStr,"M");
  sprintf(numStr, "%d",CUSTOM_M_CODE_SET_Z_PROBE_OFFSET);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Z");
  sprintf(numStr, "%.2f", -zprobe_zoffset);
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; Z offset");
  p_card->write_command(cmdStr);
#endif

#ifdef FWRETRACT
  /* Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm) */
  strcpy(cmdStr,"M207 S");
  sprintf(numStr, "%.2f", retract_length); 
  strcat(cmdStr, numStr);
  strcat(cmdStr, " F");
  sprintf(numStr, "%.2f", retract_feedrate*60); 
  strcat(cmdStr, numStr);
  strcat(cmdStr, " Z");
  sprintf(numStr, "%.2f", retract_zlift); 
  strcat(cmdStr, numStr);  
  strcat(cmdStr, " ; Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
  p_card->write_command(cmdStr);

  /* Recover: S=Extra length (mm) F:Speed (mm/m) */
  strcpy(cmdStr,"M208 S");
  sprintf(numStr, "%.2f", retract_recover_length); 
  strcat(cmdStr, numStr);
  strcat(cmdStr, " F");
  sprintf(numStr, "%.2f", retract_recover_feedrate*60); 
  strcat(cmdStr, numStr); 
  strcat(cmdStr, " ; Recover: S=Extra length (mm) F:Speed (mm/m)");
  p_card->write_command(cmdStr);
  
  /* Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries */
  strcpy(cmdStr,"M209 S");
  sprintf(numStr, "%u", (unsigned long)(autoretract_enabled ? 1 : 0)); 
  strcat(cmdStr, numStr);
  strcat(cmdStr, " ; Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
  p_card->write_command(cmdStr);

  /* Multi-extruder settings */
#if EXTRUDERS > 1
  strcpy(cmdStr,"; Multi-extruder settings not supported through SD card");
  p_card->write_command(cmdStr);
#endif
  
  /* Filament settings */
  if (volumetric_enabled)
  {
    strcpy(cmdStr, "M200 D");
    sprintf(numStr, "%.2f", filament_size[0]); 
    strcat(cmdStr, numStr);
    strcat(cmdStr, " ; Filament settings");
  } 
  else
  {
    strcpy(cmdStr,"; Filament settings: Disabled");
  }
  p_card->write_command(cmdStr);
#endif
  
  p_card->closefile();
  SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[X_AXIS]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[Y_AXIS]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[Z_AXIS]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[E_AXIS]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
#ifdef SCARA
SERIAL_ECHOLNPGM("Scaling factors:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M365 X",axis_scaling[X_AXIS]);
    SERIAL_ECHOPAIR(" Y",axis_scaling[Y_AXIS]);
    SERIAL_ECHOPAIR(" Z",axis_scaling[Z_AXIS]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
#endif
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X", max_feedrate[X_AXIS]);
    SERIAL_ECHOPAIR(" Y", max_feedrate[Y_AXIS]); 
    SERIAL_ECHOPAIR(" Z", max_feedrate[Z_AXIS]); 
    SERIAL_ECHOPAIR(" E", max_feedrate[E_AXIS]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[X_AXIS] ); 
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[Y_AXIS] ); 
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[Z_AXIS] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[E_AXIS]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration ); 
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homing[X_AXIS] );
    SERIAL_ECHOPAIR(" Y" ,add_homing[Y_AXIS] );
    SERIAL_ECHOPAIR(" Z" ,add_homing[Z_AXIS] );
    SERIAL_ECHOLN("");
#ifdef DELTA
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop adjustement (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M666 X",endstop_adj[X_AXIS] );
    SERIAL_ECHOPAIR(" Y" ,endstop_adj[Y_AXIS] );
    SERIAL_ECHOPAIR(" Z" ,endstop_adj[Z_AXIS] );
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Delta settings: L=delta_diagonal_rod, R=delta_radius, S=delta_segments_per_second");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M665 L",delta_diagonal_rod );
	SERIAL_ECHOPAIR(" R" ,delta_radius );
	SERIAL_ECHOPAIR(" S" ,delta_segments_per_second );
	SERIAL_ECHOLN("");
#endif
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp); 
    SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki)); 
    SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd));
    SERIAL_ECHOLN(""); 
#endif
#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Z offset:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M",(unsigned long)CUSTOM_M_CODE_SET_Z_PROBE_OFFSET); 
    SERIAL_ECHOPAIR(" Z" ,-zprobe_zoffset);
    SERIAL_ECHOLN("");
#endif
#ifdef FWRETRACT
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M207 S",retract_length); 
    SERIAL_ECHOPAIR(" F" ,retract_feedrate*60); 
    SERIAL_ECHOPAIR(" Z" ,retract_zlift);
    SERIAL_ECHOLN(""); 
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Recover: S=Extra length (mm) F:Speed (mm/m)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M208 S",retract_recover_length);
    SERIAL_ECHOPAIR(" F", retract_recover_feedrate*60);
	SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M209 S", (unsigned long)(autoretract_enabled ? 1 : 0));
    SERIAL_ECHOLN("");
#if EXTRUDERS > 1
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Multi-extruder settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   Swap retract length (mm):    ", retract_length_swap);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   Swap rec. addl. length (mm): ", retract_recover_length_swap);
    SERIAL_ECHOLN("");
#endif
    SERIAL_ECHO_START;
    if (volumetric_enabled) {
        SERIAL_ECHOLNPGM("Filament settings:");
        SERIAL_ECHO_START;
        SERIAL_ECHOPAIR("   M200 D", filament_size[0]);
        SERIAL_ECHOLN(""); 
#if EXTRUDERS > 1
		SERIAL_ECHO_START;
        SERIAL_ECHOPAIR("   M200 T1 D", filament_size[1]);
        SERIAL_ECHOLN(""); 
#if EXTRUDERS > 2
		SERIAL_ECHO_START;
        SERIAL_ECHOPAIR("   M200 T2 D", filament_size[2]);
		SERIAL_ECHOLN("");
#endif
#endif
    } else {
        SERIAL_ECHOLNPGM("Filament settings: Disabled");
    }
#endif
}
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homing);
        #ifdef DELTA
		EEPROM_READ_VAR(i,endstop_adj);
		EEPROM_READ_VAR(i,delta_radius);
		EEPROM_READ_VAR(i,delta_diagonal_rod);
		EEPROM_READ_VAR(i,delta_segments_per_second);
        #endif
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        EEPROM_READ_VAR(i,zprobe_zoffset);
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);
        #ifndef DOGLCD
        int lcd_contrast;
        #endif
        EEPROM_READ_VAR(i,lcd_contrast);
		#ifdef SCARA
		EEPROM_READ_VAR(i,axis_scaling);
		#endif

		#ifdef FWRETRACT
		EEPROM_READ_VAR(i,autoretract_enabled);
		EEPROM_READ_VAR(i,retract_length);
		#if EXTRUDERS > 1
		EEPROM_READ_VAR(i,retract_length_swap);
		#endif
		EEPROM_READ_VAR(i,retract_feedrate);
		EEPROM_READ_VAR(i,retract_zlift);
		EEPROM_READ_VAR(i,retract_recover_length);
		#if EXTRUDERS > 1
		EEPROM_READ_VAR(i,retract_recover_length_swap);
		#endif
		EEPROM_READ_VAR(i,retract_recover_feedrate);
		#endif

		EEPROM_READ_VAR(i, volumetric_enabled);
		EEPROM_READ_VAR(i, filament_size[0]);
#if EXTRUDERS > 1
		EEPROM_READ_VAR(i, filament_size[1]);
#if EXTRUDERS > 2
		EEPROM_READ_VAR(i, filament_size[2]);
#endif
#endif
		calculate_volumetric_multipliers();
		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
    #ifdef EEPROM_CHITCHAT
      Config_PrintSettings();
    #endif
}
#elif defined(SD_SETTINGS)
void Config_RetrieveSettings() 
{
	if( !p_card->cardReaderInitialized)
	{
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("SD Card not initialized !!! Settings not retrieved");

        return;
	}

	p_card->openFile((char *)CONFIG_FILE_NAME,true);
	p_card->startFileprint();
	starttime=millis();
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
		#ifdef SCARA
		axis_scaling[i]=1;
		#endif
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homing[X_AXIS] = add_homing[Y_AXIS] = add_homing[Z_AXIS] = 0;
#ifdef DELTA
	endstop_adj[X_AXIS] = endstop_adj[Y_AXIS] = endstop_adj[Z_AXIS] = 0;
	delta_radius= DELTA_RADIUS;
	delta_diagonal_rod= DELTA_DIAGONAL_ROD;
	delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
	recalc_delta_settings(delta_radius, delta_diagonal_rod);
#endif
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef ENABLE_AUTO_BED_LEVELING
    zprobe_zoffset = -Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

#ifdef FWRETRACT
	autoretract_enabled = false;
	retract_length = RETRACT_LENGTH;
#if EXTRUDERS > 1
	retract_length_swap = RETRACT_LENGTH_SWAP;
#endif
	retract_feedrate = RETRACT_FEEDRATE;
	retract_zlift = RETRACT_ZLIFT;
	retract_recover_length = RETRACT_RECOVER_LENGTH;
#if EXTRUDERS > 1
	retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
#endif
	retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
#endif

	volumetric_enabled = false;
	filament_size[0] = DEFAULT_NOMINAL_FILAMENT_DIA;
#if EXTRUDERS > 1
	filament_size[1] = DEFAULT_NOMINAL_FILAMENT_DIA;
#if EXTRUDERS > 2
	filament_size[2] = DEFAULT_NOMINAL_FILAMENT_DIA;
#endif
#endif
	calculate_volumetric_multipliers();

SERIAL_ECHO_START;
SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");

}
