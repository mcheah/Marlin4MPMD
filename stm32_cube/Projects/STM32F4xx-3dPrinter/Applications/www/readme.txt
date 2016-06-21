/**
  @page 3D_Printer 
    
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    STM32F4xx-3dPrinter/Applications/readme.txt  
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    June , 2015
  * @brief   3D Printer web page application based on STM32F4xx and SPWF01SA.11
  ******************************************************************************
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
  @endverbatim

@par Application Description

This application is a web page for the 3d printer board remote control.

@par Directory contents 

  - STM32F4xx-3dPrinter/Applications/www/pages/3dprinter.shtml  3d printer html web page
  - STM32F4xx-3dPrinter/Applications/www/pages/3dprinter.js     3d printer javascript used in the web page 
  - STM32F4xx-3dPrinter/Applications/www/pages/3dprinter.css    3d printer style sheet  
  - STM32F4xx-3dPrinter/Applications/www/pages/foot.png         image for style sheet, web page footer
  - STM32F4xx-3dPrinter/Applications/www/pages/header_st.png    image for style sheet, web page header
  - STM32F4xx-3dPrinter/Applications/www/pages/home_16x16.png   image for style sheet, icon for axis control
  - STM32F4xx-3dPrinter/Applications/www/pages/menu_bottom.png  image for style sheet menu
  - STM32F4xx-3dPrinter/Applications/www/pages/menu_center.png  image for style sheet menu
  - STM32F4xx-3dPrinter/Applications/www/pages/menu_top.png     image for style sheet menu  
  - STM32F4xx-3dPrinter/Applications/www/pages/outfile.img      the binary image file to be uploaded onto the 3d printer web server  
  
  - STM32F4xx-3dPrinter/Applications/www/gen.bat                batch to create a binary image file
  - STM32F4xx-3dPrinter/Applications/www/httpd_gen.exe          executable called by gen.bat
  - STM32F4xx-3dPrinter/Applications/www/cygwin1.dll            library used by httpd_gen.exe     
  
@par Hardware and Software environment

  This example requires :
    - a 3D printer board
    - a client device with WIFI and an internet navigator such as chrome, internet explorer or firefox
    - max 512Kbytes of files in the pages folder (wifi module limitation)

@par How to use it ? 

In order to use the web page, you must do the following :
 - Either use the provided outfile.img image file or create one
 - Copy this file in a web server
 - Retrieve the file over-the-air using the 
   AT+S.HTTPDFSUPDATE=<hostname>,<path>[,port] command
   with a terminal connected to 3d printer debug uart.
     •<hostname>: external web server. DNS resolvable name or IP address
     •<path>: document path
     •<port>: target host port
 - Reset the 3d printer wifi module by typing AT+CFUN=1
 - From the wifi client device, open up a navigator and type 
   <3d printer wifi module ip address>/3dprinter.shtml
   
 - How to create a new outfile.img:
     • run the gen.bat
     • the new outfile.img replaces the old one in the pages directory
     
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
