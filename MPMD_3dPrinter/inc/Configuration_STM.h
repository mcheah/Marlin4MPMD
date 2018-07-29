/**
  ******************************************************************************
  * @file    Inc/configuration_STM.h
  * @author  mcheah
  * @brief   Configuration header for STM specific parameters
  ******************************************************************************
  * @attention
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIGURATION_STM_H
#define __CONFIGURATION_STM_H

/* Exported macro ------------------------------------------------------------*/
//If uncommented, this will start user code at 0x8002000 to allow use of a bootloader
#define STM32_USE_BOOTLOADER
// USE USB CDC Serial Interface.  Comment this line out to use USART2 directly instead.
// Useful if Raspberry Pi GPIO's are hooked directly to UART interface for faster data transfer
#define STM32_USE_USB_CDC
// LCD for Malyan M200/300 printers.
//
#define MALYAN_LCD
//#define E1_ADC_PA2
//#define MINIMAL_BUILD
/* Exported functions ------------------------------------------------------- */
/* Exported Variables --------------------------------------------------------*/

#endif /* __CONFIGURATION_STM_H */

/***************************************************************END OF FILE****/
