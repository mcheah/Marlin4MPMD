/**
  @page 3D_Printer 
    
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    STM32F4xx-3dPrinter/Marlin/readme.txt  
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    December 17, 2015
  * @brief   3D Printer application based on  STM32F4xx and L6474 
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

@par Example Description 

This example is an example of a 3D printer firmware which relies on STM32Cube and 
integers the 3D printer algorithm from the Marlin firmware.
It can be used unchanged on a Prusa I3 rework 5.
For other mechanics, you will need to update the file 
stm32_cube\Middlewares\Third_Party\Marlin\configuration.h
according to your configuration.

@par Hardware and Software environment

  This example requires :
    - a 3D printer board 
    - a 3D printer mechanic (for example a Prusa I3 rework 5)

@par How to use it ? 
In order to recompile the program , you must do the following :
 - Open OpenSTM32 (SW4STM32) toolchain
 - the workspace location must be stm32_cube\
 - then In ProjectExplorer, import an existing project into workspace : stm32_cube\Projects\STM32F4xx-3dPrinter\Marlin\SW4STM32\Marlin
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
