Intended as a port of the stm32F4 port of marlin from STMicro to be used on the monoprice mini delta.
Since the MPMD is a stm32F0 based MCU, all code will need to be re-ported over to use the correct stm-cube libraries as well as updated pin mappings


#Marlin4MPMD - v1.1.0
------------------

This repository proposes an example firmware for the ST Microelectronics 3D printer board STEVAL-3DP001V1.    
The Marlin4ST firmware relies on [STM32Cube](http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/LN1897?s_searchtype=reco) and integers the 3D printer algorithms from the [Marlin 1.1.0-RC7](https://github.com/MarlinFirmware/Marlin) firmware.  
It can be used unchanged on the 3D Printer "Bicephale Mark One".  
For other mechanics, you will need to update the file:  _stm32_cube\Middlewares\Third_Party\Marlin\configuration.h_  according to your configuration.

###  Hardware and Software environment
  This example requires :
  - a monoprice mini delta printer with exposed SWD pins


###  BELOW IS OUTDATED  ###
###  How to load the FW?
A binary of the FW can be found under folder: _\stm32_cube\Binary_.
To load it or to load your own binary into the 3D printer board, the easiest way is to use the mass storage interface provided by the embedded ST-LINK (simply drag and drop to disk). To use it, follow the procedure below:
  - Power on the 3D Printer board by connecting its connectors Vin (12V-24V) and Gnd to a DC power supply   
  - Connect the ST 3D Printer reference board to a PC with the USB cable through the ST Link USB  port. Jumper J22 (boot mode selection) must be set.
  - In your files explorer, a new drive should appear in the list of removable storages. If this is not the case, this is probably because you haven't the ST-Link USB driver. You can find it  [here](http://www.st.com/web/catalog/tools/FM147/SC1887/PF260219).
  - Just copy, the binary file to the root of this new drive. Then refresh your file explorer:  if the binary file has disappeared and no error log file has been generated, it means the binary file has been successfully loaded.
  - Reset the board to start the loaded binary file. To correctly start the default firmware requires   a SD card with a configuration file.

###  How to build the FW?
In order to recompile the program , you must do the following :
 - Open OpenSTM32 (SW4STM32) toolchain (you can download installer for your OS [here](http://www.openstm32.org/HomePage).  You shall be register to download it for free
 - the workspace location must be stm32_cube/STM32/Projects/SW4STM32
 - then In ProjectExplorer, import an existing project into workspace : stm32_cube/STM32/Projects/SW4STM32/Marlin_3DPrinter/
 - Rebuild all files and load your image into target memory
 - Run the example

###  Recent changes
###** version 1.1.0**
- First version based on Marlin 1.1.0-RC7

