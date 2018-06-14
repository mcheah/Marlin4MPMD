This is a port of [Marlin4ST](https://github.com/St3dPrinter/Marlin4ST) to work with the [Monoprice Mini Delta 3D Printer](https://mpminidelta.monoprice.com/).  Marlin4ST itself is based off of [MarlinFW](https://github.com/MarlinFirmware/Marlin) version 1.1.0-RC7.  Much of this work was necessary to accomodate the less powerful STM32F070CBT microprocessor used in the Monoprice mini delta as well as supporting the specific hardware of thise machine.
The motivation for this project was to correct some of the issues with the stock firmware of the printer, most notably the slow communication speed over USB, the inconsistent heater controls, disablement of the some of the safety features, and some of the delta calibrations.  The stock firmware is closed-source although it seems to be based off of Marlin, and as as result there's now way of confirming there are no hidden bugs or issues.
That said, this is development firmware that has been tested only handful of times by me on two printers.  Relevant Marlin safety features have been turned on, but there is no guarantee it is bug free.  USE AT YOUR OWN RISK! 3d printers have been known to fail and cause fires while unattended.  I cannot be responsible for any failures as a result of using this software, although unlikely, Monoprice may use this as an excuse to void your warranty if you brick your printer (this is extremely unlikely).

###  How to load the Firmware on a stock printer?
The binary of the Marlin4MPMD firmware is found under firmware binaries\firmware.bin.  Also in this folder is the two stock firmwares as provided by [mpminidelta.com](https://www.mpminidelta.com/firmware/motion_controller), these can be re-loaded at any time by the following instructions.
To load the binary onto your printer:
- Rename the firmware to firmware.bin, copy onto the SD card that came with your printer (or any appropriate <2GB FAT32 formatted microSD card).
- Eject the SD card safely from your computer.
- Power off your Monoprice Mini Delta Printer
- Insert the SD card into your printer and apply power
- The multi-function LED should blink white a number of times while it is flashing the new firmware.  Then the LED should turn off, and your printer should appear in device manager in Windows as a STMicroelectronics Virtual COM Port (See windows drivers for more information) and as something like /dev/ttyACM0 in Linux.
- Eject the SD card to prevent re-flashing the firmware everytime your printer powers up.  Power cycle your printer.
- If the light starts blinking white again after some time then the firmware did not flash properly and you will likely need to re-format your SD card and try again.  Start with one of the stock firmwares first to confirm that the SD card is formatted properly
###  How to interface with the updater Marlin4MPMD firmware?
On windows you will need to install the STMicro Virtual COM Port driver found in the windows driver folder.  There's a .zip file containing the appropriate drivers.
On Linux and Mac the Virtual COM port driver should be installed by default, you should see the serial interface under /dev.
Fire up your favorite g-code loading software (Pronterface and Octoprint are confirmed working, Cura seems to work, although I have had some issues with the printer freezing so it is not recommended).  Click the Connect button, the terminal window should print:
start
Printer is now online.
echo:Marlin 1.1.0-RC7
echo: Last Updated: 2016-07-26 12:00 | Author: (MCheah, MPMD)
Compiled: Jun 14 2018
Issue G28 from the terminal window to make sure that communication is working correctly.  
Calibration settings from stock firmware should be directly applicable from the stock firmware, so it is advised to write down the M503 results before re-flashing the firmware.  You may need to adjust M92, M666 and M665 for your specific printer.  The M301/M304 PID settings are not directly comparable to the stock firmware, but the default values should work fairly nicely.
There is no non-volatile storage implemented right now, so whatever calibration settings you set will need to be written in your start g-code on every print or at least on power-up.
  
###  Current known Limitations:
There are a number of stock features that have not been implemented.  They all should be possible, I just haven't bothered implementing them as this was more of a proof of concept than anything.
- There is currently no support for the SD card slot, the Marlin4ST repo made a number of changes due to their hardware support of SD cards, which the STM32070 chip does not support.  This may be as simple as including the arduino SDCard libary, but I haven't looked into this yet
- There is currently no support for the UI and wifi board on the printer.  This shouldn't be too hard to implement since the command interface between the two boards is plaintext UART, and the interface seems mildly well documented, but it hasn't been a priority since I use Octoprint most the time
- There's no support for EEPROM at the moment, I suspect that they are using In-App-Programming of the system flash to store data.  For the moement you will need to load settings in the startup G-code.
- Only one G29 calibration pattern of a 3x3 mesh is supported at the moment.  This version of marlin supports greater meshes (Change AUTO_BED_LEVELING_GRID_POINTS to greater than 3 to get this support), however it can only be hardcoded to one setting at the moment

### How to build the FW?
In order to recompile the program , you must do the following :

Open OpenSTM32 (SW4STM32) toolchain (you can download installer for your OS here. You shall be register to download it for free
the workspace location must be the root folder in the git repo
then In ProjectExplorer, import an existing project into workspace : MPMD_3dPrinter
Rebuild all files in Release Mode and load your image into target memory

There are two relevant #define options for debugging on actual hardware:
STM32_USE_BOOTLOADER in main.h determines whether the program will be compiled for loading by a bootloader.  See bootloader binary for more information.  If you are debugging on the actual printer you can comment this line out.
STM32_USE_USB_CDC in configuration.h determines whether the program will use the USB CDC Virtual COM Port for communication or USART2 (located on the 5 pin molex connector on the bottom of the board) for communications.  I had experimented with using the UART directly earlier on to see if it would be a lower latency connection for interfacing with a raspberry pi, but ended up using the USB interface instead.  There may still be some gains to be made, especially on lower-powered raspberry pi zero w boards, but it would take some work and soldering skills to make this work.

### How to debug the FW?
Debugging directly on the printer hardware requires access to the 4-pin SWD jumper on the bottom of the board.  You will need to solder on some header pins to connect directly.  I use an [ST-link V2](http://www.st.com/en/development-tools/st-link-v2.html) as my debugger, but any OpenOCD compatible Cortex M-class debugger should work fine.  
The MCU on this board has been locked down with the security read-out protection set to level 1, meaning that any attempt to re-flash any memory location or change the security bits results in a full erase of the flash memory.  Therefore care should be taken when going down the route of debugging directly on the actual printer.  I've included the original bootloader that I was able to extract so that you can restore the stock software without risking bricking your printer.  To enable debugging on the board with an ST-link debugger, you will need to start the STM32 ST-LINK utility and change the Read Out Protection level to 0.  This will trigger a full erase, but will allow you to debug the firmware directly.

###  Recent changes
##Marlin4MPMD - v1.0.0 6/14/2018
------------------
This is the first release as a proof of concept that third party software can be ported to this device without altering the mainboard  Performance has been comparable to the stock firmware when connected to a PC/Raspberry Pi over USB with significant improvements in the heating control and also in the interface latency.
