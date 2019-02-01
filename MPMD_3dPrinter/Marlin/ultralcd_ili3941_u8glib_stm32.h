/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#ifndef __ULTRALCD_ILI3941_H__
#define __ULTRALCD_ILI3941_H__

#include "Marlin.h"
#if ENABLED(LERDGE_TFT)
#include <U8glib.h>

#define WIDTH 128 //Normal LCD width/height
#define HEIGHT 64
#define PAGE_HEIGHT 8

#define TFT_WIDTH 480 //TFT total Width/height
#define TFT_HEIGHT 320

#define XSCALE 3 //pixel scaling factor
#define YSCALE 5

//#define X_MIN ((TFT_WIDTH-(WIDTH*XSCALE))/2)
//#define Y_MIN ((TFT_HEIGHT-(HEIGHT*XSCALE))/2)
#define X_MIN ((TFT_WIDTH-(WIDTH*XSCALE))/2)
//#define Y_MIN ((TFT_HEIGHT-(HEIGHT*YSCALE))/2)
#define Y_MIN 0
#define X_MAX (X_MIN + XSCALE * WIDTH  - 1)
#define Y_MAX (Y_MIN + YSCALE * HEIGHT - 1)

#define LCD_COLUMN      0x2A   /* Colomn address register */
#define LCD_ROW         0x2B   /* Row address register */
#define LCD_WRITE_RAM   0x2C

static uint32_t lcd_id = 0;
#define U8G_ESC_DATA(x) (uint8_t)(x >> 8), (uint8_t)(x & 0xFF)

static const uint8_t page_first_sequence[] = {
  U8G_ESC_ADR(0), LCD_COLUMN, U8G_ESC_ADR(1), U8G_ESC_DATA(X_MIN), U8G_ESC_DATA(X_MAX),
  U8G_ESC_ADR(0), LCD_ROW,    U8G_ESC_ADR(1), U8G_ESC_DATA(Y_MIN), U8G_ESC_DATA(Y_MAX),
  U8G_ESC_ADR(0), LCD_WRITE_RAM, U8G_ESC_ADR(1),
  U8G_ESC_END
};

static const uint8_t line_sequence[] = {
  U8G_ESC_ADR(0), LCD_COLUMN, U8G_ESC_ADR(1), U8G_ESC_DATA(10), U8G_ESC_DATA(309),
  U8G_ESC_ADR(0), LCD_ROW,    U8G_ESC_ADR(1), U8G_ESC_DATA(170), U8G_ESC_DATA(171),
  U8G_ESC_ADR(0), LCD_WRITE_RAM, U8G_ESC_ADR(1),
  U8G_ESC_END
};

static const uint8_t clear_screen_sequence[] = {
  U8G_ESC_ADR(0), LCD_COLUMN, U8G_ESC_ADR(1), 0x00, 0x00, 0x01, 0xDF,
  U8G_ESC_ADR(0), LCD_ROW,    U8G_ESC_ADR(1), 0x00, 0x00, 0x01, 0x3F,
  U8G_ESC_ADR(0), LCD_WRITE_RAM, U8G_ESC_ADR(1),
  U8G_ESC_END
};

static const uint8_t ili3941_init_sequence[] = { // 0x7796 - ~ILI3941
  U8G_ESC_ADR(0),
  0x10, //sleep mode enter
  U8G_ESC_DLY(10),
  0x01, //reset
  U8G_ESC_DLY(100), U8G_ESC_DLY(100),
  0x11, //sleep mode resume
  U8G_ESC_DLY(120),
  U8G_ESC_ADR(0), 0xF0, U8G_ESC_ADR(1), 0xC3, //unknown
  U8G_ESC_ADR(0), 0xF0, U8G_ESC_ADR(1), 0x96, //unknown
  U8G_ESC_ADR(0), 0x36, U8G_ESC_ADR(1), 0x28, //memory access control, row column exchange, BGR order
  U8G_ESC_ADR(0), 0x3A, U8G_ESC_ADR(1), 0x55, //pixel format set, 16 bits per pixel
  U8G_ESC_ADR(0), 0xB4, U8G_ESC_ADR(1), 0x01, //display inversion, frame inversion full colors partial mode
  U8G_ESC_ADR(0), 0xB7, U8G_ESC_ADR(1), 0xC6, //entry mode, deep standby, VGL?
  U8G_ESC_ADR(0), 0xC1, U8G_ESC_ADR(1), 0x15, //power control, VCI x 8?
  U8G_ESC_ADR(0), 0xC2, U8G_ESC_ADR(1), 0xAF, //?
  U8G_ESC_ADR(0), 0xC3, U8G_ESC_ADR(1), 0x09, //?
  U8G_ESC_ADR(0), 0xC5, U8G_ESC_ADR(1), 0x22, //vcom, 3.55V
  U8G_ESC_ADR(0), 0xC6, U8G_ESC_ADR(1), 0x0, //?
  U8G_ESC_ADR(0), 0xE8, U8G_ESC_ADR(1), 0x40, //?
  U8G_ESC_ADR(0), 0x8A, U8G_ESC_ADR(1), 0x00, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33, //?
  U8G_ESC_ADR(0), 0xE0, U8G_ESC_ADR(1), 0xF0, 0x04, 0x08, 0x09, 0x08, 0x15, 0x2F, 0x42, 0x46, 0x28, 0x15, 0x16, 0x29, 0x2D, //positive gamma correction
  U8G_ESC_ADR(0), 0xE1, U8G_ESC_ADR(1), 0xF0, 0x04, 0x09, 0x09, 0x08, 0x15, 0x2E, 0x46, 0x46, 0x28, 0x15, 0x15, 0x29, 0x2D, //negative gamma correction
  U8G_ESC_ADR(0), 0x2A, U8G_ESC_ADR(1), 0x00,	0x00, 0x01, 0x3F, //col address
  U8G_ESC_ADR(0), 0x2B, U8G_ESC_ADR(1), 0x00, 0x00, 0x01, 0xDF, //row address
  U8G_ESC_ADR(0), 0x21,
  	  	  	  	  0x53, U8G_ESC_ADR(1), 0x24, //write ctrl display, brightness on, backlight on
  U8G_ESC_ADR(0), 0xF0, U8G_ESC_ADR(1), 0x3C, //?
  U8G_ESC_ADR(0), 0xF0, U8G_ESC_ADR(1), 0x69, //?
  U8G_ESC_DLY(150),
  U8G_ESC_ADR(0), 0x29, //display on
  0x2C, //write memory
  U8G_ESC_END
};

static int color_fg_r=0;
static int color_fg_g=48; //pipboy green
static int color_fg_b=0;

static int color_bg_r=0;
static int color_bg_g=0;
static int color_bg_b=0;
static uint16_t calcRGB16(uint8_t r, uint8_t g, uint8_t b) {
	return (b & 0x1F) | ((g & 0x2F)<<5) | ((r & 0x1F) << 11);
}

uint8_t u8g_dev_tft_480x320_upscale_from_128x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
//#if HAS_COLOR_LEDS && ENABLED(PRINTER_EVENT_LEDS)
//  uint16_t newColor;
//#endif
  u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
  uint16_t buffer[128*XSCALE];
  uint32_t i, j, k;
  uint8_t byte;

  switch(msg) {
    case U8G_DEV_MSG_INIT: {
      uint8_t err = dev->com_fn(u8g, U8G_COM_MSG_INIT, U8G_SPI_CLK_CYCLE_NONE, &lcd_id);
      if(err!=1)
    	  return 0;
      if (lcd_id == 0x040404) { return 0; } /* No connected display on FSMC */
      if (lcd_id == 0xFFFFFF) { return 0; } /* No connected display on SPI */

      memset(buffer, 0x00, sizeof(buffer));

      if ((lcd_id & 0xFFFF) == 0x7796 || (lcd_id & 0xFFFF) == 0x7396) { // ST7796
        u8g_WriteEscSeqP(u8g, dev, ili3941_init_sequence);
      }

      u8g_WriteEscSeqP(u8g, dev, clear_screen_sequence);
      for (i = 0; i < 960*2; i++) {
        u8g_WriteSequence(u8g, dev, 160, (uint8_t *)buffer);
      }
      break; }

    case U8G_DEV_MSG_STOP:
      break;

    case U8G_DEV_MSG_PAGE_FIRST:
//#if HAS_COLOR_LEDS && ENABLED(PRINTER_EVENT_LEDS)
//      newColor = (0xF800 & (((uint16_t)leds.color.r) << 8)) | (0x07E0 & (((uint16_t)leds.color.g) << 3)) | (0x001F & (((uint16_t)leds.color.b) >> 3));
//      if ((newColor != 0) && (newColor != color)) {
//        color = newColor;
//        drawButtons(u8g, dev);
//      }
//#endif
      u8g_WriteEscSeqP(u8g, dev, page_first_sequence);
      break;

    case U8G_DEV_MSG_PAGE_NEXT:
      for (j = 0; j < 8;  j++) {
        k = 0;
        for (i = 0; i < (uint32_t) pb->width;  i++) {
          byte = *(((uint8_t *)pb->buf) + i);
          if (byte & (1 << j)) {
        	for(uint8_t n=0;n<XSCALE;n++)
        		buffer[k++] = calcRGB16(color_fg_r,color_fg_g,color_fg_b);
          } else {
        	for(uint8_t n=0;n<XSCALE;n++)
        		buffer[k++] = calcRGB16(color_bg_r,color_bg_g,color_bg_b);
          }
        }
        for (k = 0; k < YSCALE; k++) {
          for(uint16_t n=0;n<XSCALE*128;n+=64)
        	  u8g_WriteSequence(u8g, dev, 128, (uint8_t *)&buffer[n]);
        }
      }
      break;

    case U8G_DEV_MSG_SLEEP_ON:
      return 1;

    case U8G_DEV_MSG_SLEEP_OFF:
      return 1;
  }
  return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}
uint8_t u8g_com_stm32hal_fsmc_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
//uint8_t u8g_dev_tft_480x320_upscale_from_128x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);
U8G_PB_DEV(u8g_dev_tft_480x320_upscale_from_128x64, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_tft_480x320_upscale_from_128x64_fn, u8g_com_stm32hal_fsmc_fn);

class U8GLIB_ILI3971_480X320_UPSCALE_FROM_128X64 : public U8GLIB {
 public:
	U8GLIB_ILI3971_480X320_UPSCALE_FROM_128X64(int d) : U8GLIB(&u8g_dev_tft_480x320_upscale_from_128x64) {  }
};


#endif // LERDGE_TFT
#endif //__ULTRALCD_ILI3941_H__
