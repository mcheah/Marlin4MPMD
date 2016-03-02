Marlin4ST
---------

This repository proposes an example firmware for the ST Microelectronics 3D printer board STEVAL-3DP001V1.    
The Marlin4ST firmware relies on [STM32Cube](http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/LN1897?s_searchtype=reco) and integers the 3D printer algorithms from the [Marlin](https://github.com/MarlinFirmware/Marlin) firmware.  
It can be used unchanged on the 3D Printer "Prusa I3 rework 5".  
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
 
