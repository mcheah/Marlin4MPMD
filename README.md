Marlin4ST
---------

A Marlin firmware for the ST Microelectronics 3D printer board STEVAL-3DP001V1  
(based on STM32 and L6474 stepper motor drivers).

This repository proposes  an example of a 3D printer firmware which relies on STM32Cube and 
integers the 3D printer algorithm from the Marlin firmware.  
It can be used unchanged on a Prusa I3 rework 5.  
For other mechanics, you will need to update the file:  
_stm32_cube\Middlewares\Third_Party\Marlin\configuration.h_  
according to your configuration.

#  Hardware and Software environment
  This example requires :
  - a 3D printer board STEVAL-3DP001V1 
  - a 3D printer mechanic (for example a Prusa I3 rework 5)

#  How to use it ? 
In order to recompile the program , you must do the following :
 - Open your preferred toolchain (IAR or SW4STM32 are natively supported) 
   - For IAR, open the project: _stm32_cube\Projects\STM32F4xx-3dPrinter\Marlin\EWARM\Project.eww_  
   - For SW4STM32 :
      - the workspace location must be: _stm32_cube\_
      - then open the project from: _ST_Marlin\stm32_cube\Projects\STM32F4xx-3dPrinter\Marlin\SW4STM32\Marlin_
 - Rebuild all files and load your image into target memory
 - Run the example
 
