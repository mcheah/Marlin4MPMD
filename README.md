[![Join the chat at https://gitter.im/Marlin4MPMD/Lobby](https://badges.gitter.im/Marlin4MPMD/Lobby.svg)](https://gitter.im/Marlin4MPMD/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This is a port of [Marlin4ST](https://github.com/St3dPrinter/Marlin4ST) to work with the [Monoprice Mini Delta 3D Printer](https://mpminidelta.monoprice.com/).  Marlin4ST itself is based off of [MarlinFW](https://github.com/MarlinFirmware/Marlin) version 1.1.0-RC7.  Much of this work was necessary to accommodate the less powerful STM32F070CBT microprocessor used in the Monoprice mini delta as well as supporting the specific hardware of thise machine.
The motivation for this project was to correct some of the issues with the stock firmware of the printer, most notably the slow communication speed over USB, the inconsistent heater controls, disablement of the some of the safety features, and some of the delta calibrations.  The stock firmware is closed-source although it seems to be based off of Marlin, and as as result there's now way of confirming there are no hidden bugs or issues.
That said, this is development firmware that has been tested only handful of times by me on two printers.  Relevant Marlin safety features have been turned on, but there is no guarantee it is bug free.  
USE AT YOUR OWN RISK! 3d printers have been known to fail and cause fires while unattended.  I cannot be responsible for any failures as a result of using this software, although unlikely, Monoprice may use this as an excuse to void your warranty if you brick your printer (this is extremely unlikely).

*Please see the [Wiki](https://github.com/mcheah/Marlin4MPMD/wiki) for more information on installation, calibration, and other differences specific to this firmware*

The stock power supply cannot power both heaters on at the same time, so a special firmware has been created to prevent this situation to allow use of the stock power supply.  If you have upgraded to a 10A power supply, use the "firmware_10ALimit.bin" file to allow faster heating and potentially a higher max bed temperature.  Otherwise, use the "firmware_5ALimit.bin" file, which should still heat the bed faster, but will still respect the 5A limit of the power supply
###  Changes from stock monoprice firmware
See [Migration Guide](https://github.com/mcheah/Marlin4MPMD/wiki/Migration) for more details
- Mesh bed leveling ignores the P0-5 / C parameters that the stock firmware uses.  Instead, calling G29 will run a mesh leveling grid over a 7x7 area between -45 to 45mm on the X and Y axes in 15mm increments.  Points outside the probeable radius on the print bed will be extrapolated based on probed values.  This mesh grid is persistent, and saved/loaded by M500/M501.  

**New 1.2.2**
Changed the behavior of G29, now runs a single point probe to adjust delta height and detect misconfigured steps per mm settings before beginning the normal mesh probe.  Behavior of G29 is now as follows:

`G29 P0`: Only runs the single point delta height probe to adjust for height variances in your build platform or nozzle length.  This should replace `G29` inside of your g-code and to be run on every print.  Does not run the Mesh Bed Leveling Probe

`G29 P1`: Skips the single point delta height probe and directly runs the Mesh Bed Leveling Probe.  This is equivalent to what G29 used to do prior to 1.2.2.  This means that delta height errors will be dirctly accounted for in the mesh instead of changing the `M665 H` parameter

`G29 or G29 P2`: Runs the single point delta height probe followed by the Mesh Bed Leveling Probe.  Because the delta height is adjusted first, the center point of the mesh should be zero'ed out or close to it.  If you have old Mesh bed leveling data, you will likely need to adjust your saved values since they will no longer be centered about zero

<s>This means that you should probably not run G29 on startup for every print.  Instead, your startup g-code only needs a G28 command at the top and the restored mesh grid should take care of the rest.  If you find the initial height is off, then you should adjust the M665 H parameter manually to compensate.</s>  The Z parameter for G29 is still valid, but the better way is to use the M851 Z\<probe offset\> command to compensate for the switch travel of the bed.  On my printer I use a value around -0.5mm.  YMMV
- Mesh bed leveling points can be retrieved and edited directly using the M421 command.  This is extended by the standard Marlin g-code in the following ways:
  
  M421 \<no parameters\> : displays the current mesh 
  

  M421 C : clears the current mesh
  
  M421 E : re-extrapolates the non-probeable points on the mesh using the actual probe points.  **fixed in 1.2.2, E now clears these points for you**<s>It is required that you set all of these points to 0.0 manually</s>
  
  M421 I\<X Index\> J\<Y Index\> Z\<Value\> sets the mesh value at I,J to the Z value
  
  M421 I\<X Index\> J\<Y Index\> Q\<Offset> adjusts the mesh value at I,J up or down by Q offset
  
  Editing of the mesh bed level is required to account for some of the peculiarities of this printer and non-linearities of the bed leveling switches.  I strongly recommend doing a paper test at each of the probe points to verify the switch values.  In some instances (especially opposite the delta towers) the variation can be as much as 0.2-0.3mm which can throw off the first layer on large prints.
- added support for M48 for bed probe repeatability
- M665 delta parameters now includes parameters for adjusting delta height (the distance from the Zmax home position to the bed, the Zmin position in mm), individual rod length (X,Y,Z: adjustments are added to the nominal length in mm), individual tower angles (A,B,C: adjustments to the tower angles in degrees, sum of all three should be 0).  **New in 1.2.2, now can adjust individual tower radius adjust with D,E,F, units are mm's added or subracted to the nominal radius**
###  How to load the Firmware on a stock printer?
The binary of the Marlin4MPMD firmware is found under firmware binaries\firmware.bin.  Also in this folder is the two stock firmwares as provided by [mpminidelta.com](https://www.mpminidelta.com/firmware/motion_controller), these can be re-loaded at any time by the following instructions.
To load the binary onto your printer:
- Follow the guide in the [wiki](https://github.com/mcheah/Marlin4MPMD/wiki/SD-Cards) for correctly formatting and troubleshooting your SD card.  Test that you can flash the stock firmware first before installing Marlin4MPMD, if you have not formatted your card correctly you can render your printer inoperable until you have formatted it correctly
- Further details for installation are available on the [wiki](https://github.com/mcheah/Marlin4MPMD/wiki/Installation)
- Rename the appropriate firmware to firmware.bin, **Owners of newer printers should use the v43+ firmware files to set correct default steps per mm** copy onto the SD card that came with your printer (or any appropriate FAT32 formatted microSD card. Not all cards are compatible, see the [wiki](https://github.com/mcheah/Marlin4MPMD/wiki/SD-Cards) for more details).
- Also copy the fcupdate.flg file as well
- Eject the SD card safely from your computer.
- Power off your Monoprice Mini Delta Printer
- Insert the SD card into your printer and apply power
- The multi-function LED should blink white a number of times while it is flashing the new firmware.  Then the LED should turn off, and your printer should appear in device manager in Windows as a STMicroelectronics Virtual COM Port (See windows drivers for more information) and as something like /dev/ttyACM0 in Linux.
- Eject the SD card or issue `M30 firmware.bin` to prevent re-flashing the firmware everytime your printer powers up.  Power cycle your printer.
- If the light starts blinking white again after some time then the firmware did not flash properly and you will likely need to re-format your SD card and try again.  Start with one of the stock firmwares first to confirm that the SD card is formatted properly
###  How to interface with the updater Marlin4MPMD firmware?
On windows you will need to install the STMicro Virtual COM Port driver found in the windows driver folder.  There's a .zip file containing the appropriate drivers.
On Linux and Mac the Virtual COM port driver should be installed by default, you should see the serial interface under /dev.
Fire up your favorite g-code loading software (Pronterface and Octoprint are confirmed working, Cura seems to work, although I have had some issues with the printer freezing so it is not recommended).  Click the Connect button, the terminal window should print:
start

Printer is now online.

echo:Marlin4MPMD

echo: Last Updated: 2018-07-25 00:00 | Author: (MCheah, MPMD)

Compiled: Jul 31 2018

Issue G28 from the terminal window to make sure that communication is working correctly.  

Calibration settings from stock firmware should be directly applicable from the stock firmware, so it is advised to write down the M503 results before re-flashing the firmware.  You may need to adjust M92, M666 and M665 for your specific printer.  The M301/M304 PID settings are not directly comparable to the stock firmware, but the default values should work fairly nicely.
  
###  Current known Limitations:
There are a number of stock features that have not been implemented.  They all should be possible, I just haven't bothered implementing them as this was more of a proof of concept than anything.
- Only one G29 calibration pattern of a 7x7 mesh is supported at the moment.  This version allows direct editing and saving of the mesh grid, but it's currently hard-coded to a 7x7 grid as it's likely that you will not actually want to run this before every print.  **fixed in 1.2.2. Call G29 P0 in your g-code**<s>As a result, it is not able to adjust for changes in delta height without re-running a full mesh calibration, so if you need to finetune the delta height, use the M665 H parameter.</s>
- LCD GUI browsing of SD card folders isn't supported, all g-code must be stored in the root directory
- SOME wifi functionality may be available given that you have already set up the IP address previously, but loading g-code or running arbitrary functions over WIFI are not supported.  If you're reading this, I'm going to guess that you're already using something like octoprint already

###  Troubleshooting
Installation of the firmware uses the stock bootloader to replace the application on the printer.  This is perfectly safe and reversible, but if you have issues, you should first check that your SD card formatted correctly.  The stock firmware will read both FAT16 and FAT32 partitions, but the bootloader only recognizes FAT32 partitions.  See the [wiki](https://github.com/mcheah/Marlin4MPMD/wiki/SD-Cards) for more details.
Make sure that you have properly formatted the SD card as FAT32 with a 512 byte allocation size.  See https://www.mpminidelta.com/firmware/motion_controller for more info.

After verifying the partition, copy over one of the stock firmwares to the SD card.  If these files do not flash correctly, then you will not be able to install Marlin4MPMD, verify that you can re-flash the stock firmware first.

If the stock firmware can be flashed correctly, then rename one of the appropriate Marlin4MPMD firmware files (depending on which power supply you have) and try installing by SD.  The white LED should flash for several seconds and then turn off indicating the new firmware is loaded.  If you see flashing red and purple at any time, then a hard error has occurred and the printer should be reset.  Green lights are also normal, this means the printer is ready to connect over USB.  If you experience problems, please open a github issue and I will try to help.


### How to build the FW?
In order to recompile the program , you must do the following :

Open OpenSTM32 (SW4STM32) toolchain (you can download installer for your OS here. You shall be register to download it for free
the workspace location must be the root folder in the git repo
then In ProjectExplorer, import an existing project into workspace : MPMD_3dPrinter
Rebuild all files in Release Mode and load your image into target memory

There are two relevant #define options for debugging on actual hardware:
STM32_USE_BOOTLOADER in configuration_STM.h determines whether the program will be compiled for loading by a bootloader.  See bootloader binary for more information.  If you are debugging on the actual printer you can comment this line out.
STM32_USE_USB_CDC in configuration.h determines whether the program will use the USB CDC Virtual COM Port for communication or USART2 (located on the 5 pin molex connector on the bottom of the board) for communications.  I had experimented with using the UART directly earlier on to see if it would be a lower latency connection for interfacing with a raspberry pi, but ended up using the USB interface instead.  There may still be some gains to be made, especially on lower-powered raspberry pi zero w boards, but it would take some work and soldering skills to make this work.  Currently using the USB is required when the LCD screen is enabled.

### How to debug the FW?
Debugging directly on the printer hardware requires access to the 4-pin SWD jumper on the bottom of the board.  You will need to solder on some header pins to connect directly.  I use an [ST-link V2](http://www.st.com/en/development-tools/st-link-v2.html) as my debugger, but any OpenOCD compatible Cortex M-class debugger should work fine.  
The MCU on this board has been locked down with the security read-out protection set to level 1, meaning that any attempt to re-flash any memory location or change the security bits results in a full erase of the flash memory.  Therefore care should be taken when going down the route of debugging directly on the actual printer.  I've included the original bootloader that I was able to extract so that you can restore the stock software without risking bricking your printer.  To enable debugging on the board with an ST-link debugger, you will need to start the STM32 ST-LINK utility and change the Read Out Protection level to 0.  This will trigger a full erase, but will allow you to debug the firmware directly.

## Donations
I work on and support this project in my free time for fun.  I enjoy answering questions and helping people make the best out of this little printer, but the process of adding new features and bugfixes is time consuming and challenging to troubleshoot different conditions and hardware.  This is truly free software to do with as you please, but if you have found this project to be useful or would like to see a custom feature or build added, 

[Feel free to buy me a coffee](https://paypal.me/MCheah?locale.x=en_US).  

Any and all support by the community is greatly appreciated!
###  Recent changes
## Marlin4MPMD - v1.3.3 4/6/2019
- M20 now lists folders correctly and hides hidden folders
- Changed dropsegments to 0 to allow for very small microstepping movements
- Added a "Next" entry to LCD file listing to allow selecting more than 64 files, folders are still not supported
- Fixed a bug where G33 would not work if towers were rotated in software
- Added a check for fast file upload with M34.  Stay tuned for a standalone app supporting this command, transfer speeds > 20x over USB are possible...
- Added G28 S0 option to disable safe homing drop after homing
- Added support for binary-packed '.bgc' gcode files for faster file upload
- Added tweaks and fixes for LCD display including sending cancel/pause/resume commands to octoprint
# Updates for the 1.3.2 Release Candidate
- Fixed progress bar not updating during SD prints
- Fixed issues listing files on the LCD with filenames longer than 20 characters
- Fixed longstanding bug where `M421 E` would erroneously corrupt points in the bed level matrix == 0.0
- Inverted G33 D flag to represent "dryrun" when enabled
- Added ability to change the mesh grid spacing by issuing `M421 X<spacing> Y<spacing>`.  Default is 15mm
# Updates for the 1.3.1 Release Candidate
- I reverted the changes that doubled the feedrates.  This was a bug related to the Lerdge board specifically and didn't apply to the MPMD.  Oops.  That said, there was a bug in accelerations on 16x stepper boards that has been fixed as well as a bug that prevented step rates above ~70mm/sec that was fixed, so in general things should be a little bit snappier.

# Updates for the 1.3.0 Release Candidate
If you've made it to try out the latest 1.3.0 Release Candidate, here are some random notes.  I haven't had a chance to document everything yet, so please contact me on gitter.im or report an issue as you notice changes.
## List of changes
1. Rotate the towers by default to place +Y directly opposite the LCD screen and +X to the right of the LCD.  Many of you had done this in hardware by swapping the cables. To go back, issue `M665 X-120 Y-120 Z-120`
2. Settings are now stored in flash instead of on the SD card.  To load your old settings, just run `M32 M_CFG.G` followed by `M500`.  If you get an error about the API version not matching this indicates the data in flash isn't valid and should be overwritten
3. Fixed a bug that caused feedrates to be half of what they were programmed to.  This means for the same G-code the printer will now run twice as fast (matching the stock firmware).  The bug also prematurely limited the max travel speeds, you can now issue commands as fast as 150mm/s (although motors will likely start grinding at this speed).  As a result, you will have to half the feed rates in your slicer settings to compensate.
4. Added G33 command to auto calibrate the M666 settings for leveling the endstops.  This may take multiple iterations to complete.  Issue `G33 V3 R<Radius>` to set the radius to probe at and display the individual probing data.  If the delta height/radius is not adjusted right, the head might crash into the bed when probing G33.  To fix this issue `M851 R<Raise Height>` beforehand to increase the probe raise height.
5. Mesh grids will automatically be adjusted so that the center point is 0.00 adjustment.  This was done because the mesh calibration would override the delta height adjust.  If your delta height is wrong (did not run `G29 P0` or `G29 P2`) doing a `G29 P1` will potentially cause the print to fail.  The correct way to adjust for your first layer height is to either change your probe offset `M851 Z<Offset>` and issue `G29 P0`, or manually edit `M665 H`.
6. Manually changing mesh settings is made easier by detecting if you're on a mesh adjust point when changing with `M421 Z/Q`.  The effects are immediate so if you probe with G30, get an error of +1.2, if you issue `M421 Q1.2`, you should immediately be able to go to `G1 Z0` and run the paper test.  Obviously doesn't work if you're in between mesh probe points.
7. Long file names should now be supported
8. Removed `M48`, `M290`, and several other unused functions.  Added `M524` to abort SD prints
9. Better handling of cancelling prints.  Should not correctly trigger octoprint to stop using action commands, requires 1.3.9 or greater.  Have not tested with USB printing from pronterface/repetier/cura.
10. Hopefully fixed the errant thermal runaway triggering when heating the bed first on 5A firmware.  Would like testing with high bed temps to confirm.
11. Fixed mesh extrapolation issue noted in [#16](https://github.com/mcheah/Marlin4MPMD/pull/16)

## Marlin4MPMD - v1.2.2 7/31/2018
- Many reliability fixes that should hopefully prevent some of the common issues people were experiencing
  - Automatically re-initialize the SD card in case of ejecting the card unsafely
  - Automatically call M501 on powerup to restore saved settings, this requires the SD card to be present
  - Fixed issues where LCD would be unresponsive until connected by USB
- M501 no longer triggers the LCD build menu
- Updated the thermistor table to better match the stock firmware
- Updated temp sensor measurement for better accuracy and performance
- Bed switches light red when pressed
- Added M665 D,E,F individual delta radius adjust
- Added G29 P0,1,2 settings to enable/disable single point delta-height adjust
  - G29 P0/2 will detect incorrectly configured steps per mm and (hopefully) adjust automatically
- Calling M421 E will now clear out non-probeable points for you before extrapolating
- Calling G28 when above the delta clip height (approximately 107mm) will no longer drop to a safe build height to prevent crashing into tall prints that exceed the safe build height
- Prevent thermal runaway errors when slewing the extruder after pre-heating the bed for 5A power supplies.  Bed temperature will still droop significantly, so account for this before starting the print
## Marlin4MPMD - v1.2.1 7/11/2018
- Added fix for the LCD that causes the build menu to pop up appropriately when printing from USB, previously the LCD could time out and cause the print to fail when it issued commands to change movement to relative mode
- Added fixes to PID tuning to prevent overshoots and fail during autotune
- Added support M73 command to set the progress bar percent.  Use M73 P0 in your start g-code and M73 P100 in your end g-code to ensure that the build menu is called and finished (M109,M190, and M24 also trigger it, but just to be safe).  You can also use plugins such as:
[M73 Progress](https://plugins.octoprint.org/plugins/m73progress/) to sync octoprint progress with your print, or manually insert M73 commands in your g-code.  M73 is ignored when printing from SD.
## Marlin4MPMD - v1.2.0 7/09/2018
Third release brings things pretty close to parity to the stock firmware changes include:
- Added MalyanLCD support from [xC0000005's](https://github.com/xC0000005/Marlin) port of Marlin 2.0 for use with the Malyan M200.  There is a potential to use that effort for a Marlin2.0 port for MPMD, but it will likely require some more effort.  All stock GUI functions should be implemented now.
- Changed the way mesh bed leveling is done.  Uses a fixed 7x7 mesh now and is persistent through G28 homing.  You can now save/load meshes using the M500/M501 command.  You can also set individual points by the M421 command.  See above for more details
- Bugfixes for the SD card
- Added delta angle trim, use M665 XYZ

## Marlin4MPMD - v1.1.0 6/24/2018
------------------
Second release to address some of the limitations of the first release.  Most notably:
- Limited current consumption during heating to prevent overdriving the power supply and triggering a reset
- Immediately on reset, detect a hot extruder and pull away nozzle in case of power supply glitch or outage
- Added suppport for SD cards!
- Various reliability fixes and improvments to USB
- Added support for M500/M501 by saving calibration settings to M_CFG.G on the SD card
## Marlin4MPMD - v1.0.0 6/14/2018
------------------
This is the first release as a proof of concept that third party software can be ported to this device without altering the mainboard  Performance has been comparable to the stock firmware when connected to a PC/Raspberry Pi over USB with significant improvements in the heating control and also in the interface latency.
