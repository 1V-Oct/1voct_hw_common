#ifndef _SSD1306_H
#define _SSD1306_H

/**
 *  Adafruit SSD1306 library modified by William Greiman for
 *  unbuffered LiquidCrystal character mode.
 *
 * -- Further modified by JBoyton to support character scaling
 * and to allow horizontal positioning of text to any pixel.
 * Vertical text position is still limited to a row.
 * Edited to make specific to Uno R3 and 128x64 size.
 * Also added HW SPI support.
*/

#include "board_defs.h"

#define BUILD_AVR (0)

#define WITH_SSD1306_DEBUG     (1)
#define WITH_SSD1306_I2C       (1)
#define WITH_SSD1306_HW_SPI    (0)
#define WITH_SSD1306_FB        (0)
#define WITH_OLED_FB           (0)
#define WITH_SSD1306_SYS_PRINT (0)

#if WITH_SSD1306_DEBUG
#define DBG_SSD1306 LOGI
#else
#define DBG_SSD1306(_X_) ; //
#endif

#define USE_SYSTEM_PRINT 1

//#include "Arduino.h"

#define SSD1306_LCDWIDTH  128
#define SSD1306_LCDHEIGHT 64

#define SSD1306_SETCONTRAST         0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS       0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE       0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN  0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_SETSTARTPAGE 0XB0
#define SSD1306_MEMORYMODE   0x20

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC  0x1
#define SSD1306_SWITCHCAPVCC 0x2
//------------------------------------------------------------------------------

#define GPIO_OLED_DC_PORT  GPIOB // OLED Data/Command Switch
#define GPIO_OLED_DC_PIN   1
#define GPIO_OLED_CS_PORT  GPIOA // OLED Chip Select (_CS)
#define GPIO_OLED_CS_PIN   4
#define GPIO_OLED_RST_PORT GPIOB // OLED RST
#define GPIO_OLED_RST_PIN  0

#define SSD_GPIO_CLR(port, pin) HAL_GPIO_WritePin((GPIO_TypeDef *)port, (1 << pin), GPIO_PIN_RESET);
#define SSD_GPIO_SET(port, pin) HAL_GPIO_WritePin((GPIO_TypeDef *)port, (1 << pin), GPIO_PIN_SET);

#define SSD_DC_CLR()  SSD_GPIO_CLR(GPIO_OLED_DC_PORT, GPIO_OLED_DC_PIN);
#define SSD_RST_CLR() SSD_GPIO_CLR(GPIO_OLED_RST_PORT, GPIO_OLED_RST_PIN);
#define SSD_CS_CLR()  SSD_GPIO_CLR(GPIO_OLED_CS_PORT, GPIO_OLED_CS_PIN);
#define SSD_DC_SET()  SSD_GPIO_SET(GPIO_OLED_DC_PORT, GPIO_OLED_DC_PIN);
#define SSD_RST_SET() SSD_GPIO_SET(GPIO_OLED_RST_PORT, GPIO_OLED_RST_PIN);
#define SSD_CS_SET()  SSD_GPIO_SET(GPIO_OLED_CS_PORT, GPIO_OLED_CS_PIN);

typedef enum {
  Black = 0x00, /*!< Black color, no pixel */
  White = 0x01  /*!< Pixel is set. Color depends on LCD */
} COLOR;

void   ssd1306_init(void);
void   ssd1306_clear(void);
void   ssd1306_setCursor(uint8_t row, uint8_t col);
size_t ssd1306_write_char(uint8_t c);
size_t ssd1306_write_string(const char *s);
void   ssd1306_setTextSize(uint8_t size, uint8_t spacing);

#endif // _SSD1306_H
