#include "drv_ili9341.h"

#include "cv_inputs.h"
#include "gpio.h"
#include "sys_settings.h"
#include "vo_logger.h"
#include "gfx.h"
#include "vo_system.h"
#include "pots.h"

static volatile uint16_t *ili_blit_ptr;
static volatile uint32_t  ili_blit_size; // in 16 bit words
#define SPI_DMA_MAX_TRANSFER_SIZE (0xFFFF)
static volatile bool ili_blit_busy;

// #pragma GCC optimize ("Os")

extern SPI_HandleTypeDef hspi1;

/* clang-format off */
const uint8_t st_init_cmds[] = {

  0x01, 0, 0xff, 0x00, // Delay 1000ms (0 equals 1000)
  0x11, 0, 0xff,  0x00, // Delay 1000ms (0 equals 1000)
  0xF0,
  1,
  0xC3,
  0x36,
  1,
  0x48,
  0x3A,
  1,
  0x55,
  0xB4,
  1,
  0x01,
  0xB6,
  3,
  0x80,
  0x02,
  0x3B,
  0xE8,
  8,
  0x40,
  0x8A,
  0x00,
  0x00,
  0x29,
  0x19,
  0xA5,
  0x33,
  0xC1,
  1,
  0x06,
  0xC2,
  1,
  0xA7,
  0xC5,
  1,
  0x18,
  0xE0,
  14,
  0xF0,
  0x09,
  0x0B,
  0x06,
  0x04,
  0x15,
  0x2F,
  0x54,
  0x42,
  0x3C,
  0x17,
  0x14,
  0x18,
  0x1B,
  0xE1,
  14,
  0xE0,
  0x09,
  0x0B,
  0x06,
  0x04,
  0x03,
  0x2B,
  0x43,
  0x42,
  0x3B,
  0x16,
  0x14,
  0x17,
  0x1B,
  0xF0,
  1,
  0xC3,
  0xF0,
  1,
  0x69,
  0xe7,
  1,
  0x00,
  0x29,
  0,
  0xff,
  0x00, // Delay 1000ms (0 equals 1000)
  0,
};

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

static const uint8_t stft_reset_empty[] = {

  0x01, 0,
  0xff, 0x00, // Delay 1000ms (0 equals 1000)
  0};

static const uint8_t stft_reset_st7789[] = {
  0x01, 0x00, // Software reset
  0xff, 0x00, // Delay 1000ms (0 equals 1000)
  // 0x11, 0,
  // 0xff, 0x00,                               // Delay 1000ms (0 equals 1000)
  // 0xF0, 1, 0xC3, // ?
  // 0x09, 5,
  0x3A, 1, 0x55,
  0x36, 1, 0xa0,
  0xb2, 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,
  0xb7, 1, 0x11,
  0xb8, 1, 0x19,
  0xc0,
  1,
  0x2c,
  0xc2,
  1,
  0x01,
  0xc3,
  1,
  0x12,
  0xc4,
  1,
  0x20,
  0xc6,
  1,
  0x0f,
  // 0xd0, 2, 0xa4, 0xa1,
  // 0x2a, 4,  0x00, 0,320 >> 8, 320 & 0xFF,
  // 0x2b, 4,  0x00, 0,320 >> 8, 320 & 0xFF,
  // 0xB4, 1, 0x01,
  // 0xB6, 3, 0x80, 0x02, 0x3B,
  // 0xE8, 8, 0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33,
  // 0xC1, 1, 0x06,
  // 0xC2, 1, 0x01,
  // 0xC5, 1, 0x12,
  0xE0, 14, 0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23,
  0xE1, 14, 0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23,
  // 0xF0, 1, 0xC3,
  // 0xF0, 1, 0x69,
  // 0xe7, 1, 0x00,
  // 0xff, 0x00,                               // Delay 1000ms (0 equals 1000)
  // 0xff, 0x00,
  0xe7, 1, 0x11,
  0x20, 0,
  0x11, 0,
  0x13, 0,
  0x29, 0,
  0xff, 0x00, // Delay 1000ms (0 equals 1000)
  0};

// format below is CMD cnt DATA, DATA, ...
// 0xff is Delay in ms
const uint8_t stft_reset_ili9xxxx[] = {
  0x01, 0x00,                               // Software reset
  0xff, 0x00,                               // Delay 1000ms (0 equals 1000)
  0xcb, 0x05, 0x39, 0x2c, 0x00, 0x34, 0x02, // Power Control A
  0xcf, 0x03, 0x00, 0xc1, 0x30,             // Power Control B
  0xe8, 0x03, 0x85, 0x00, 0x78,             // Driver Timing Control A
  0xea, 0x02, 0x00, 000,                    // Driver Timing Control B
  0xed, 0x04, 0x64, 0x03, 0x12, 0x81,       // Power On Sequence Control
  0xf7, 0x01, 0x20,                         // Pump Ratio Control
  // Power Control VRH[5:0]
  0xc0, 0x01, 0x23,
  // Power Control SAP[2:0] BT[3:0]
  0xc1, 0x01, 0x10,
  // VCM Control
  0xc5, 0x02, 0x3e, 0x28,
  // VCM Control 2
  0xc7, 0x01, 0x86,
  // Memory Access Control
  0x36, 0x01, 0x48,
  // Pixel format
  0x3a, 0x01, 0x55,
  // Frame Ratio Control, Standard RGB Colour
  0xb1, 2, 0x00, 0x1f,
  0xb2, 2, 0x00, 0x1b,
  // Display Function Control
  0xb6, 0x03, 0x08, 0x82, 0x27,
  // 3Gamma Function disable
  // 0xf2, 0x01, 0x00,
  // Gamma Curve Select
  // 0x26, 0x01, 0x01,
  // Positive Gamma Correction
  // 0xe0, 15, 0x0f, 0x31, 0x2B, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00,
  // 0xe0, 15, 0x08, 0x31, 0x2B, 0x05, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00,
  // Negative Gamma Correction
  // 0xe1, 15, 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f,
  // Exit Sleep
  0x11, 0x00,
  // *** Delay 120ms
  0xff, 120,
  // Turn On Display
  0x29, 0x00,
  // Screen Rotation - Fixed for now to horizontal 2
  0x36, 0x01, (0x40 | 0x80 | 0x20 | 0x08),
  // *** END
  0x00};

#if 0
#define ILI_CS_LOW     HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
#define ILI_CS_HIGH    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
#define ILI_RESET_LOW  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
#define ILI_RESET_HIGH HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
#define ILI_DC_LOW     HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
#define ILI_DC_HIGH    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
#else
#define ILI_CS_LOW     gpio_reset_pin(LCD_CS_GPIO_Port, LCD_CS_Pin);
#define ILI_CS_HIGH    gpio_set_pin(LCD_CS_GPIO_Port, LCD_CS_Pin);
#define ILI_RESET_LOW  gpio_reset_pin(LCD_RESET_GPIO_Port, LCD_RESET_Pin);
#define ILI_RESET_HIGH gpio_set_pin(LCD_RESET_GPIO_Port, LCD_RESET_Pin);
#define ILI_DC_LOW     gpio_reset_pin(LCD_DC_GPIO_Port, LCD_DC_Pin);
#define ILI_DC_HIGH    gpio_set_pin(LCD_DC_GPIO_Port, LCD_DC_Pin);
#endif

static void ili_reset(void) {
  ILI_RESET_LOW
  // HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  ILI_RESET_HIGH
  HAL_Delay(200);
  // ILI_CS_LOW
  // HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
}

void ili_spi_init(void) {
  ILI_CS_LOW
}

void ili_enable(void) {
  ILI_RESET_HIGH
}

void ili_spi_send(uint8_t d) {
  HAL_SPI_Transmit(&hspi1, &d, 1, 1);
  // SPI1->CR2 |= 1;
  // SPI1->CR1 |= SPI_CR1_SPE;
  // SPI1->CR1 |= SPI_CR1_CSTART;
  // while( !(SPI1->SR & SPI_SR_TXP) ) {}
  // *(__IO uint8_t *)(&SPI1->TXDR) = d;
  // while(!(SPI1->SR & SPI_SR_EOT) ) {}
  // SPI1->IFCR = 0xffffffff;
  // SPI1->CR1 &= ~SPI_CR1_SPE;

  // SPI1->CR1 = 0;
}

/* Send command (char) to LCD */
void ili_write_cmd(uint8_t Command) {
  ILI_CS_LOW
  ILI_DC_LOW
  ili_spi_send(Command);
  ILI_CS_HIGH
}

/* Send Data (char) to LCD */
void ili_write_data(uint8_t Data) {
  ILI_DC_HIGH
  ILI_CS_LOW
  ili_spi_send(Data);
  ILI_CS_HIGH
}

void ili_wait_scanline(uint16_t line) {
  uint8_t  bx[3];
  uint16_t sl;

  do {
    ILI_CS_LOW
    ILI_DC_LOW            // HIGH
      ili_spi_send(0x45); // get the scanline
    ILI_DC_HIGH
    HAL_SPI_Receive(&hspi1, bx, 2, 1000);
    ILI_CS_HIGH
    sl = bx[1] << 8 | bx[2];
    LOGI("SL: %d", sl);
  } while (sl < line);
}

uint32_t stft_readid(void) {
  uint8_t  bx[4];
  uint32_t id;
#if 1
  uint16_t sl;

  ILI_CS_LOW
  ILI_DC_LOW            // HIGH
    ili_spi_send(0x04); // get the scanline
  // ILI_CS_HIGH
  ILI_DC_HIGH
  int i;
  HAL_SPI_Receive(&hspi1, bx, 4, 1000);
  // for (i = 0; i < 4; i++)
  //   HAL_SPI_Receive(&hspi1, &bx[i], 1, 1000);
  // HAL_SPI_TransmitReceive(&hspi1, &bx[i], &bx[i], 1, 1000);
  ILI_CS_HIGH
  id = *(uint32_t *)bx;
#else
  int k;
  for (k = 1; k < 4; k++) {
    ILI_CS_LOW
    ILI_DC_LOW // HIGH
      ili_spi_send(0xd9);
    ILI_DC_HIGH
    ili_spi_send(0x10 + k);
    ILI_DC_LOW
    ili_spi_send(0xd3);
    ILI_DC_HIGH
    int i;
    HAL_SPI_Receive(&hspi1, &bx[k], 1, 1000);
    // for(i = 0 ; i < 4 ; i++)
    //   HAL_SPI_Receive(&hspi1, &bx[i], 1, 1000);
    // HAL_SPI_TransmitReceive(&hspi1, &bx[i], &bx[i], 1, 1000);
    ILI_CS_HIGH
  }
#endif
  // sl = bx[1] << 8 | bx[2];
  LOGI("LCD_ID: %08x - %02x:%02x:%02x:%02x", id, bx[0], bx[1], bx[2], bx[3]);
  return id;
}

static void stft_send_init_sequence(const uint8_t *p) {
  uint8_t c;
  while ((c = *p++) != 0) {
    if (c == 0xff) {
      // delay
      uint32_t ms = *p++;
      if (ms == 0) {
        ms = 1000;
      }
      HAL_Delay(ms);
    } else {
      ili_write_cmd(c);
      size_t n = *p++;
      while (n--) {
        ili_write_data(*p++);
      }
    }
  }
}
void MX_SPI1_Init(void);

void ili_init(void) {
  ili_enable();
  ili_spi_init();
  ili_reset();
  // ili_readid();
  ili_blit_busy = false;
  stft_send_init_sequence(stft_reset_empty);
  uint32_t id = stft_readid();
  // ili_wait_scanline(120);
  // __HAL_RCC_SPI1_CLK_DISABLE();
  // __HAL_RCC_PLL3_DISABLE();
  // __HAL_RCC_PLL3_CONFIG(1, 32, 2, 2, 2);
  // __HAL_RCC_PLL3_ENABLE();
  // __HAL_RCC_SPI1_CLK_ENABLE();
  // MX_SPI1_Init();
  // if (id == 0x9341)
#if WITH_LCD_BACKLIGHT_CONTROL
    if ((id == 0x00a9c242) || (id != 0xffff && id != 0))
  stft_send_init_sequence(stft_reset_st7789);
  else
#endif
  stft_send_init_sequence(stft_reset_ili9xxxx);
}

/* Set Address - Location block - to draw into */
void ili_set_addr(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2) {
  ili_write_cmd(0x2A);
  ili_write_data(X1 >> 8);
  ili_write_data(X1);
  ili_write_data(X2 >> 8);
  ili_write_data(X2);

  ili_write_cmd(0x2B);
  ili_write_data(Y1 >> 8);
  ili_write_data(Y1);
  ili_write_data(Y2 >> 8);
  ili_write_data(Y2);

  ili_write_cmd(0x2C);
}

#define ILI_SPI SPI1

#if 0
static void ili_spi_write(uint32_t data)
{
  /* Set the number of data at current transfer */
  MODIFY_REG(ILI_SPI->CR2, SPI_CR2_TSIZE, 1);

  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&hspi1);

  /* Master transfer start */
  SET_BIT(ILI_SPI->CR1, SPI_CR1_CSTART);

  while( !(ILI_SPI->SR & SPI_SR_TXP) ) {}
  SPI2->TXDR = data;
}

static void ili_spi_write16(uint32_t data)
{
  /* Set the number of data at current transfer */
  MODIFY_REG(ILI_SPI->CR2, SPI_CR2_TSIZE, 2);

  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&hspi1);

  /* Master transfer start */
  SET_BIT(ILI_SPI->CR1, SPI_CR1_CSTART);

  while( !(ILI_SPI->SR & SPI_SR_TXP) ) {}
  *((__IO uint8_t *)&hspi1.Instance->TXDR)  = data & 0xff;
  while( !(ILI_SPI->SR & SPI_SR_TXP) ) {}
  *((__IO uint8_t *)&hspi1.Instance->TXDR)  = data >> 8;
  while( !(ILI_SPI->SR & SPI_FLAG_EOT) ) {}

  __HAL_SPI_CLEAR_EOTFLAG(&hspi1);
  __HAL_SPI_CLEAR_TXTFFLAG(&hspi1);

  /* Disable SPI peripheral */
  __HAL_SPI_DISABLE(&hspi1);

//      __HAL_SPI_CLEAR_UDRFLAG(hspi);

}
#endif
static void ili_spi_fill(uint32_t data, size_t n) {
  while (n) {
    /* Set the number of data at current transfer */
    size_t trs = (n >= 25600) ? 25600 : n;
    n -= trs;
    MODIFY_REG(ILI_SPI->CR2, SPI_CR2_TSIZE, trs * 2);

    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(&hspi1);

    /* Master transfer start */
    SET_BIT(ILI_SPI->CR1, SPI_CR1_CSTART);

    while (trs--) {
      while (!(ILI_SPI->SR & SPI_SR_TXP)) {
      }
      *((__IO uint8_t *)&hspi1.Instance->TXDR) = data & 0xff;
      while (!(ILI_SPI->SR & SPI_SR_TXP)) {
      }
      *((__IO uint8_t *)&hspi1.Instance->TXDR) = data >> 8;
    }
    while (!(ILI_SPI->SR & SPI_FLAG_EOT)) {
    }

    __HAL_SPI_CLEAR_EOTFLAG(&hspi1);
    __HAL_SPI_CLEAR_TXTFFLAG(&hspi1);

    /* Disable SPI peripheral */
    __HAL_SPI_DISABLE(&hspi1);
  }
}

// static void ili_spi_blit(uint16_t *data)

#if 0
volatile uint16_t *blit_ptr;
volatile int8_t    blit_pass = 0;
uint8_t            gfx_busy  = 0;

void ili_blit_wait(void) {
#if 1
  while (blit_pass > 0) {
    HAL_Delay(1);
  }
#else
  while (blit_pass > 0 && blit_pass <= 3) {
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
    }
    HAL_Delay(1);
  }
#endif
}
#endif

void ili_blit_wait(void) {
  while (ili_blit_busy) {
    HAL_Delay(1);
  }
}
static void ili_blit_init_dma(uint16_t *ptr, uint32_t size) {
  uint32_t transfer_size = (size > SPI_DMA_MAX_TRANSFER_SIZE) ? SPI_DMA_MAX_TRANSFER_SIZE : size;
  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)ptr, transfer_size);
  ili_blit_ptr  = ptr + transfer_size;
  ili_blit_size = size - transfer_size;
}

void ili_blit_region(uint16_t *ptr, uint32_t y, uint32_t h) {
  uint32_t size;
  
  uint16_t *p;
  uint32_t buf_start;
  uint32_t buf_size;
  buf_start = 320 * y;
  buf_size = 320 * h;
  p = gfx_buffer + buf_start;
  vo_clean_dcache((uint32_t *)ptr, buf_size * 2);
  
  ili_blit_wait();
  size          = h * 320;
  ili_blit_busy = true;
  __HAL_SPI_ENABLE(&hspi1);
  ili_set_addr(0, y, 320, y + h);
  ILI_DC_HIGH
  ILI_CS_LOW
  hspi1.Init.DataSize  = SPI_DATASIZE_16BIT;
  hspi1.Instance->CFG1 = (hspi1.Instance->CFG1 & ~0x1f) | SPI_DATASIZE_16BIT;
  ptr                  = ptr + y * 320;
  ili_blit_init_dma(ptr, size);
}
#if 0
void ili_blit_screen(uint16_t *ptr) {
  blit_pass = 1;
  __HAL_SPI_ENABLE(&hspi1);
  ili_set_addr(0, 0, 320, 240);
  ILI_DC_HIGH
  ILI_CS_LOW
#if 0
  // IO Mode
  size_t n = 3;
  while(n--) {
    HAL_SPI_Transmit(&hspi1, (uint8_t *)ptr, 25600 * 2, 1000);
    ptr += 25600;
  }
  ILI_CS_HIGH
  return;
#else
  // DMA Mode
  blit_ptr             = ptr;
  hspi1.Init.DataSize  = SPI_DATASIZE_16BIT;
  // hspi1.Instance->CFG2 = (hspi1.Instance->CFG2 | SPI_CFG2_LSBFRST);
  hspi1.Instance->CFG1 = (hspi1.Instance->CFG1 & ~0x1f) | SPI_DATASIZE_16BIT;

  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)ptr, 38400); //, 1);
  ptr += 25600;
#endif
  // while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
}
#endif
void ili_fill_rect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t col) {
  ili_set_addr(X1, Y1, X2, Y2);
  ILI_DC_HIGH
  ILI_CS_LOW
  ili_spi_fill(col, (X2 - X1) * (Y2 - Y1));
  ILI_CS_HIGH
}

#if 0
void ili_clear_screen(uint16_t col) {
  //ili_wait_scanline(120);

  // ili_fill_rect(0, 0, 320, 240, col);
  // return;

   ili_set_addr(0, 0, 320, 240);

  unsigned char Temp_Buffer2[2] = {col>>8, col};
  ILI_DC_HIGH
  ILI_CS_LOW
  // ili_spi_fill(col, 320 * 240);
  // ili_spi_fill(col, 25600);
  // ili_spi_fill(col, 25600);
  // ili_spi_fill(col, 25600);
  
  
  //320 * 240);
  for (int i = 0 ; i < 320 * 240 ; i++) {
    HAL_SPI_Transmit(&hspi1, Temp_Buffer2, 2, 1);
    // ili_spi_write16(col);
  }
}
#endif

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (ili_blit_size) {
    ili_blit_init_dma((uint16_t *)ili_blit_ptr, ili_blit_size);
  } else {
    hspi1.Init.DataSize  = SPI_DATASIZE_8BIT;
    // hspi1.Instance->CFG2 = (hspi1.Instance->CFG2 & ~SPI_CFG2_LSBFRST);
    hspi1.Instance->CFG1 = (hspi1.Instance->CFG1 & ~0x1f) | SPI_DATASIZE_8BIT;
    ILI_CS_HIGH
    __HAL_SPI_DISABLE(hspi);
    ili_blit_busy = false;
    adc_pots_scan();
  }
}
