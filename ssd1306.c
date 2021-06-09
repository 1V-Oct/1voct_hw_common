
#include "ssd1306.h"

#if BUILD_AVR
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "Wire.h"
#include "gpio_management.h"
#include "spi.h"
#else
#define PROGMEM
#endif

#include "ssdfont.h"
// #include "voct.h"
#include "vo_logger.h"
#include "vo_string.h"
#include "bsp/board.h"

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

static uint8_t m_ScreenBuffer[SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8];

static uint8_t col_, row_;               // cursor position
static uint8_t textSize_, textSpacing_;  // text size and horiz char spacing (pixels between)

#if BUILD_AVR

uint8_t digitalPinToBitMask(uint8_t pin) {
  if (pin >= 14)
    pin -= 14;
  else if (pin >= 8)
    pin -= 8;
  return 1 << pin;
}

uint8_t digitalPinToPort(uint8_t pin) {
  if (pin >= 14)
    return 2;
  else if (pin >= 8)
    return 1;
  return 0;
}

volatile uint8_t *portOutputRegister(uint8_t port) {
  switch (port) {
  case 2:
    return &PORTC;
  case 1:
    return &PORTB;
  case 0:
    return &PORTD;
  }
  return &PORTD;
}

#define SPI_CLOCK_DIV4 4

#endif  // BUILD_AVR

void spi_transfer(uint8_t c) {
#ifdef __STM32F1xx_HAL_DEF
  HAL_StatusTypeDef ret;
  ret = HAL_SPI_Transmit(&hspi1, &c, 1, 100);
  if (ret != HAL_OK) {
    LOGI("SPI_ERR: %d\n", ret);
  }
#else
#error Missing SPI Transfer Function
#endif
}

uint8_t pgm_read_byte(const uint8_t *p) {
  return *p;
}

#if WITH_SSD1306_I2C

static void i2cWrite(uint8_t cd, uint8_t c) {
  uint8_t dt[2];
  dt[0] = cd;
  dt[1] = c;
  HAL_I2C_Master_Transmit(&hi2c1, 0x78, dt, 2, 10);

  // HAL_I2C_Mem_Write(&hi2c1, 0x78, cd, 1, &c, 1, HAL_MAX_DELAY);
  // HAL_I2C_Master_Transmit(&hi2c1, 0x3c, &c)
  // DBG_SSD1306("1\n");
  // Wire.beginTransmission(0x3c);
  // DBG_SSD1306("2\n");
  // Wire.write(cd); // control or data
  // DBG_SSD1306("3\n");
  // Wire.write(c);
  // DBG_SSD1306("4\n");
  // Wire.endTransmission(0);
  // DBG_SSD1306("5\n");
}

#endif

#if WITH_SSD1306_HW_SPI

static inline void spiWrite(uint8_t c) {
#if WITH_SSD1306_HW_SPI
  (void)spi_transfer(c);
#else  // bit twiddle SPI
  for (uint8_t bit = 0x80; bit; bit >>= 1) {
    *clkport &= ~clkpinmask;
    if (c & bit)
      *mosiport |= mosipinmask;
    else
      *mosiport &= ~mosipinmask;
    *clkport |= clkpinmask;
  }
#endif
}

#endif  // WITH_SSD1306_I2C

static void sendData(uint8_t c) {
#if WITH_SSD1306_I2C
  i2cWrite(0x40, c);
#endif
#if WITH_SSD1306_HW_SPI
  SSD_CS_SET();
  SSD_DC_SET();
  SSD_CS_CLR();
  spiWrite(c);
  SSD_CS_SET();
#endif
#if 0
  *csport |= cspinmask;
  *dcport |= dcpinmask;
  *csport &= ~cspinmask;
  spiWrite(c);
  *csport |= cspinmask;
#endif
}

void sendCommand(uint8_t c) {
#if WITH_SSD1306_I2C
  i2cWrite(0x00, c);
#endif

#if WITH_SSD1306_HW_SPI
  SSD_CS_SET();
  SSD_DC_CLR();
  SSD_CS_CLR();
  spiWrite(c);
  SSD_CS_SET();
#endif
#if 0
  *csport |= cspinmask;
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
  spiWrite(c);
  *csport |= cspinmask;
#endif
}

#define GPIO_HIGH GPIO_PIN_SET
#define GPIO_LOW GPIO_PIN_RESET

#if 0
void digitalWrite(gpio_t g, uint8_t val)
{
  //HAL_SPI_Transmit();
}
static const Gpio::GPIO_OUTPUT =
#endif

//------------------------------------------------------------------------------
void ssd1306_init(void) {
  //dcGpio((uint8_t)0,(uint8_t)1);
  //csGpio(0,1);
  //rstGpio(0,1);
  //rst_ = GPIO_OLED_RST;

#if WITH_SSD1306_I2C
  // Wire.begin();
#endif

  col_ = 0;
  row_ = 0;
  textSize_ = 1;
  textSpacing_ = 1;
#if BUILD_AVR
  rstGpio.pinMode(Gpio::GPIO_OUTPUT);
#endif

#if WITH_SSD1306_HW_SPI

#if BUILD_AVR
  // set pin directions
  pinMode(dc_, GPIO_OUTPUT);
  pinMode(cs_, GPIO_OUTPUT);
  csport = portOutputRegister(digitalPinToPort(cs_));
  cspinmask = digitalPinToBitMask(cs_);
  dcport = portOutputRegister(digitalPinToPort(dc_));
  dcpinmask = digitalPinToBitMask(dc_);
#else

#endif

#if WITH_SSD1306_HW_SPI
  //spi_init();
  //spi_set_clock_divider(SPI_CLOCK_DIV4); // 8 MHz
#else  // bit twiddle SPI
  pinMode(data_, OUTPUT);
  pinMode(clk_, OUTPUT);
  clkport = portOutputRegister(digitalPinToPort(clk_));
  clkpinmask = digitalPinToBitMask(clk_);
  mosiport = portOutputRegister(digitalPinToPort(data_));
  mosipinmask = digitalPinToBitMask(data_);
#endif

#endif

  // Reset
  //digitalWrite(rst_, GPIO_HIGH);
  SSD_RST_SET();
  //_delay_us(100);
  //digitalWrite(rst_, GPIO_LOW);
  board_delay(1);
  SSD_RST_CLR();
  //_delay_us(1000);
  board_delay(10);
  //digitalWrite(rst_, GPIO_HIGH);
  SSD_RST_SET();

  // Init sequence for 128x64 OLED module
  DBG_SSD1306("SSD1306: Display Off\n");
  sendCommand(SSD1306_DISPLAYOFF);  // 0xAE
  DBG_SSD1306("SSD1306: Set clock div\n");
  sendCommand(SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
  sendCommand(0x80);                        // the suggested ratio 0x80
  sendCommand(SSD1306_SETMULTIPLEX);        // 0xA8
  sendCommand(0x3F);
  sendCommand(SSD1306_SETDISPLAYOFFSET);    // 0xD3
  sendCommand(0x0);                         // no offset
  sendCommand(SSD1306_SETSTARTLINE | 0x0);  // line #0
  sendCommand(SSD1306_CHARGEPUMP);          // 0x8D
  sendCommand(0x14);
  sendCommand(SSD1306_MEMORYMODE);  // 0x20
  sendCommand(0x00);                // was: 0x2 page mode
  sendCommand(SSD1306_SEGREMAP | 0x1);
  sendCommand(SSD1306_COMSCANDEC);
  sendCommand(SSD1306_SETCOMPINS);  // 0xDA
  sendCommand(0x12);
  sendCommand(SSD1306_SETCONTRAST);  // 0x81
  sendCommand(0xCF);
  sendCommand(SSD1306_SETPRECHARGE);  // 0xd9
  sendCommand(0xF1);
  sendCommand(SSD1306_SETVCOMDETECT);  // 0xDB
  sendCommand(0x40);
  sendCommand(SSD1306_DISPLAYALLON_RESUME);  // 0xA4
  sendCommand(SSD1306_NORMALDISPLAY);        // 0xA6

  sendCommand(SSD1306_DISPLAYON);  //--turn on oled panel
}
//------------------------------------------------------------------------------
// clear the screen
void ssd1306_clear(void) {
  sendCommand(SSD1306_COLUMNADDR);
  sendCommand(0);                     // Column start address (0 = reset)
  sendCommand(SSD1306_LCDWIDTH - 1);  // Column end address (127 = reset)

  sendCommand(SSD1306_PAGEADDR);
  sendCommand(0);  // Page start address (0 = reset)
  sendCommand(7);  // Page end address

#if WITH_SSD1306_I2C
#ifdef TWBR
  uint8_t twbrbackup = TWBR;
  TWBR = 12;  // upgrade to 400KHz!
#endif

  //Serial.println(TWBR, DEC);
  //Serial.println(TWSR & 0x3, DEC);

  // I2C
  for (uint16_t i = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8); i++) {
    sendData(0);
    // send a bunch of data in one xmission
    // Wire.beginTransmission(0x3c);
    // Wire.write(0x40);
    // for (uint8_t x=0; x<16; x++) {
    //   Wire.write(0x00);
    //   i++;
    // }
    // i--;
    // Wire.endTransmission();
  }
#ifdef TWBR
  TWBR = twbrbackup;
#endif
#else
#if 0
  *csport |= cspinmask;
  *dcport |= dcpinmask;
  *csport &= ~cspinmask;
#endif
  SSD_CS_SET();
  SSD_DC_SET();
  SSD_CS_CLR();
  for (uint16_t i = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8); i++) {
    spiWrite(0x00);
  }
  SSD_CS_SET();
#if 0
  *csport |= cspinmask;
#endif
#endif
}
//------------------------------------------------------------------------------
void ssd1306_setCursor(uint8_t row, uint8_t col) {
  if (row >= SSD1306_LCDHEIGHT / 8) {
    row = SSD1306_LCDHEIGHT / 8 - 1;
  }
  if (col >= SSD1306_LCDWIDTH) {
    col = SSD1306_LCDWIDTH - 1;
  }
  row_ = row;  // row is 8 pixels tall; must set to byte sized row
  col_ = col;  // col is 1 pixel wide; can set to any pixel column

  sendCommand(SSD1306_SETLOWCOLUMN | (col & 0XF));
  sendCommand(SSD1306_SETHIGHCOLUMN | (col >> 4));
  sendCommand(SSD1306_SETSTARTPAGE | row);
}
//------------------------------------------------------------------------------
size_t ssd1306_write_char(uint8_t c) {
  if (textSize_ == 1) {  // dedicated code since it's 4x faster than scaling

    if (col_ >= SSD1306_LCDWIDTH)
      return 0;
    col_ += 7;  // x7 font
    if (c < 32 || c > 127)
      c = 127;
    c -= 32;
    const uint8_t *base = font + 5 * c;
    for (uint8_t i = 0; i < 5; i++) {
      uint8_t b = pgm_read_byte(base + i);
      sendData(b);
    }
    for (uint8_t i = 0; i < textSpacing_; i++) {
      if (col_ >= SSD1306_LCDWIDTH)
        break;
      col_++;
      sendData(0);  // textSpacing_ pixels of blank space between characters
    }

  } else {  // scale characters (up to 8X)

    uint8_t sourceSlice, targetSlice, sourceBitMask, targetBitMask, extractedBit, targetBitCount;
    uint8_t startRow = row_;
    uint8_t startCol = col_;

    for (uint8_t irow = 0; irow < textSize_; irow++) {
      if (row_ + irow > SSD1306_LCDWIDTH - 1)
        break;
      if (irow > 0)
        ssd1306_setCursor(startRow + irow, startCol);
      for (uint8_t iSlice = 0; iSlice < 5; iSlice++) {
        sourceSlice = (uint8_t)pgm_read_byte(font + 5 * (c - 32) + iSlice);
        targetSlice = 0;
        targetBitMask = 0x01;
        sourceBitMask = 0x01 << (irow * 8 / textSize_);
        targetBitCount = textSize_ * 7 - irow * 8;
        do {
          extractedBit = sourceSlice & sourceBitMask;
          for (uint8_t i = 0; i < textSize_; i++) {
            if (extractedBit != 0)
              targetSlice |= targetBitMask;
            targetBitMask <<= 1;
            targetBitCount--;
            if (targetBitCount % textSize_ == 0) {
              sourceBitMask <<= 1;
              break;
            }
            if (targetBitMask == 0)
              break;
          }
        } while (targetBitMask != 0);
#if WITH_SSD1306_I2C
        for (uint8_t i = 0; i < textSize_; i++) {
          i2cWrite(0x40, targetSlice);
        }
#else
#if 0
        *csport |= cspinmask;
        *dcport |= dcpinmask;
        *csport &= ~cspinmask;
#endif
        SSD_CS_SET();
        SSD_DC_SET();
        SSD_CS_CLR();
        for (uint8_t i = 0; i < textSize_; i++) {
          spiWrite(targetSlice);
        }
        SSD_CS_SET();
#if 0
        *csport |= cspinmask;
#endif
#endif
      }
    }
    ssd1306_setCursor(startRow, startCol + 5 * textSize_ + textSpacing_);
  }

  return 1;
}
//------------------------------------------------------------------------------
size_t ssd1306_write_string(const char *s) {
  size_t n = vo_strlen(s);
  for (size_t i = 0; i < n; i++) {
    ssd1306_write_char(s[i]);
  }
  return n;
}

#if WITH_OLED_FB

void SSD1306_text::Fill(SSD1306_text::COLOR color) {
  /* Set memory */
  uint32_t i;
  uint8_t c;

  c = (color == Black) ? 0x00 : 0xFF;
  for (i = 0; i < sizeof(m_ScreenBuffer); i++) {
    m_ScreenBuffer[i] = c;
  }
}

#endif  // WITH_OLED_FB

//------------------------------------------------------------------------------
#if WITH_SSD1306_SYS_PRINT
void ssd1306_writeInt(int i) {  // slighly smaller than system print()
  char buffer[7];
  itoa(i, buffer, 10);
  write(buffer);
}
#endif
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void ssd1306_setTextSize(uint8_t size, uint8_t spacing) {
  textSize_ = size;
  textSpacing_ = spacing;
};

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
