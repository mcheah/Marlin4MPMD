/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016, 2017 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * u8g_com_stm32duino_fsmc.cpp
 *
 * Communication interface for FSMC
 */
#include "Marlin.h"
#if ENABLED(DOGLCD)
#ifdef LERDGE_TFT

#define LCD_READ_ID     0xD3   /* Read display identification information */
#define	LCD_REG       (*((volatile unsigned short *) 0x60000000)) // RS = 0	Command
#define LCD_RAM       (*((volatile unsigned short *) 0x60020000)) // RS = 1	Data

#include "U8glib.h"

void LCD_IO_Init(void);
void LCD_IO_WriteData(uint16_t RegValue);
void LCD_IO_WriteReg(uint8_t Reg);
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);

static uint8_t msgInitCount = 2; // Ignore all messages until 2nd U8G_COM_MSG_INIT
static uint8_t fsmcInit = 0;

uint8_t u8g_com_stm32hal_fsmc_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  if (msgInitCount) {
    if (msg == U8G_COM_MSG_INIT) msgInitCount--;
    if (msgInitCount) return -1;
  }

  static uint8_t isCommand;

  switch(msg) {
    case U8G_COM_MSG_STOP:
      break;
    case U8G_COM_MSG_INIT:
//      u8g_SetPIOutput(u8g, U8G_PI_RESET);

      LCD_IO_Init();
      u8g_Delay(100);

      if (arg_ptr != NULL)
        *((uint32_t *)arg_ptr) = LCD_IO_ReadData(LCD_READ_ID, 2);

      isCommand = 0;
      break;

    case U8G_COM_MSG_ADDRESS:           // define cmd (arg_val = 0) or data mode (arg_val = 1)
      isCommand = arg_val == 0 ? 1 : 0;
      break;

    case U8G_COM_MSG_RESET:
//      u8g_SetPILevel(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      if (isCommand)
        LCD_IO_WriteReg(arg_val);
      else
        LCD_IO_WriteData((uint16_t)arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ:

      for (uint8_t i = 0; i < arg_val; i += 2)
        LCD_IO_WriteData(*(uint16_t *)(((uint32_t)arg_ptr) + i));
      break;
  }
  return 1;
}

void u8g_Delay(uint16_t val) {
	delay(val);
}


void LCD_IO_Init() {
	BSP_LCD_Init();
}

void LCD_IO_WriteData(uint16_t RegValue) {
	LCD_RAM = RegValue;
}

void LCD_IO_WriteReg(uint8_t Reg) {
	LCD_REG = (uint16_t)Reg;
}

uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize) {
	volatile uint32_t data;
	LCD_REG = (uint16_t)RegValue;

	data = LCD_RAM; // dummy read

	while (--ReadSize) {
		data <<= 16;
		data |= LCD_RAM & 0xFFFF;
	}
	return (uint32_t)data;
}

#endif // LERDGE_TFT
#endif // HAS_GRAPHICAL_LCD
