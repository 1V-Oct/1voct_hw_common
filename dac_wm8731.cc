#include "app_config.h"
#include "vo_common.h"
#include "vo_logger.h"
#include "vo_string.h"
#include "hw/bsp/board.h"

extern I2C_HandleTypeDef hi2c2;

DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

#undef kNumOutputs
#define kNumOutputs (2)


DMA_BUFFER uint16_t rx_dma_buffer_[BLOCK_SIZE * kNumOutputs * 2 * 2];
DMA_BUFFER uint16_t tx_dma_buffer_[BLOCK_SIZE * kNumOutputs * 2 * 2];


#define WM8731_I2C_ADDR      0x34

#define WM8731_REG_LLINEIN   0
#define WM8731_REG_RLINEIN   1
#define WM8731_REG_LHEADOUT  2
#define WM8731_REG_RHEADOUT  3
#define WM8731_REG_ANALOG    4
#define WM8731_REG_DIGITAL   5
#define WM8731_REG_POWERDOWN 6
#define WM8731_REG_INTERFACE 7
#define WM8731_REG_SAMPLING  8
#define WM8731_REG_ACTIVE    9
#define WM8731_REG_RESET     15

#define WM8731_ADDR          (0x1A << 1)


static void wm8731_write_reg(uint8_t reg, uint16_t value) {
  uint16_t tmp = 0;
  uint8_t  data[2];

  tmp     = ((uint16_t)reg << 9) | value;
  data[0] = (tmp & 0xFF00) >> 8;
  data[1] = tmp & 0x00FF;

HAL_I2C_Master_Transmit(&hi2c2, WM8731_ADDR, data, 2, HAL_MAX_DELAY);
}

#if 0
#define AUDIO_BUFFER_SIZE 2200
static int16_t audio_data[2 * AUDIO_BUFFER_SIZE];

static void fill_audio_buffer(void) {
  for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
    int16_t value         = (int16_t)(32000.0 * sin(2.0 * M_PI * i / 22.0));
    audio_data[i * 2]     = value;
    audio_data[i * 2 + 1] = value;
  }
}
#endif
void wm8731_set_in_volume(int vol) {
  // -23 <= vol <= 8
  const unsigned involume = 0x17 + vol;
  wm8731_write_reg(WM8731_REG_LLINEIN, 0x100 | (involume & 0x1f)); // Left line in, unmute
}

void wm_8731_set_out_volume(int voldB) {
  // -73 <= voldB <= 6
  const unsigned volume = 121 + voldB;
  wm8731_write_reg(WM8731_REG_LHEADOUT, 0x100 | (volume & 0x7f)); // Left headphone
  wm8731_write_reg(WM8731_REG_RHEADOUT, 0x100 | (volume & 0x7f)); // Right headphone
}

void wm8731_init(void) {
  wm8731_write_reg(WM8731_REG_RESET, 0b000000000); // Reset!
  wm8731_set_in_volume(0);
  wm_8731_set_out_volume(-10);
  wm8731_write_reg(WM8731_REG_ANALOG, 0b000010010); // Analog path - select DAC, no bypass
#ifdef WM8731_HIGHPASS
  wm8731_write_reg(WM8731_REG_DIGITAL, 0b000000000); // Digital path - disable soft mute
#else
  wm8731_write_reg(WM8731_REG_DIGITAL, 0b000000001); // Digital path - disable soft mute and highpass
#endif
  wm8731_write_reg(WM8731_REG_POWERDOWN, 0b000000000); // Power down control - enable everything
  wm8731_write_reg(WM8731_REG_INTERFACE, 0b000000010); // Interface format - 16-bit I2S
  wm8731_write_reg(WM8731_REG_ACTIVE, 0b000000001);    // Active control - engage!

  // fill_audio_buffer();
  // HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
}




/**
 * @brief I2S MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2s: I2S handle pointer
 * @retval None
 */
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) {
  GPIO_InitTypeDef GPIO_InitStruct;
  ZERO_STRUCT(GPIO_InitStruct);
  if (hi2s->Instance == SPI2) {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB14     ------> I2S2_SDI
    PB15     ------> I2S2_SDO
    PC6     ------> I2S2_MCK
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB10     ------> I2S2_CK
    PB12     ------> I2S2_WS
    PB15     ------> I2S2_SDO
    */
#if BOARD_VER == 3
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_15;
#elif BOARD_VER == 4
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_15;
#else
#error Unknown board version
#endif

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // /* I2S2 DMA Init */
    // /* SPI2_TX Init */
    // hdma_spi2_tx.Instance                 = DMA1_Stream5;
    // hdma_spi2_tx.Init.Request             = DMA_REQUEST_SPI2_TX;
    // hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    // hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    // hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    // hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    // hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    // hdma_spi2_tx.Init.Mode                = DMA_CIRCULAR;
    // hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    // hdma_spi2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    // if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
    //   Error_Handler();
    // }

    // __HAL_LINKDMA(hi2s, hdmatx, hdma_spi2_tx);

    /* I2S2 DMA Init */
    /* SPI2_RX Init */
    hdma_spi2_rx.Instance                 = DMA1_Stream6;
    hdma_spi2_rx.Init.Request             = DMA_REQUEST_SPI2_RX;
    hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_spi2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmarx, hdma_spi2_rx);

    /* SPI2_TX Init */
    hdma_spi2_tx.Instance                 = DMA1_Stream5;
    hdma_spi2_tx.Init.Request             = DMA_REQUEST_SPI2_TX;
    hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmatx, hdma_spi2_tx);
  }
}


void dac_priv_init(void) {
#if WITH_DAC_WM8731
  hi2s2.Instance                = SPI2;
  hi2s2.Init.Mode               = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard           = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat         = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput         = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq          = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL               = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit           = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion        = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_LEFT;

  hi2s2.Init.MasterKeepIOState  = I2S_MASTER_KEEP_IO_STATE_DISABLE;

  // hi2s2.Init.Mode               = I2S_MODE_MASTER_FULLDUPLEX;

  // hi2s2.Init.Standard           = I2S_STANDARD_PCM_SHORT;
  // hi2s2.Init.DataFormat         = I2S_DATAFORMAT_32B;
  // hi2s2.Init.MCLKOutput         = I2S_MCLKOUTPUT_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
    Error_Handler();
  }

  hi2s2.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX; /* fixes bug in CubeMX */
  if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
    Error_Handler();
  }
#else
#error "Unknown DAC"
#endif
  wm8731_init();

  #if WITH_DAC_DMA
  // HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_dma_buffer_, rx_dma_buffer_, BLOCK_SIZE * kNumOutputs * 2);
  hi2s2.State = HAL_I2S_STATE_READY;
  if (HAL_I2S_Receive_DMA(&hi2s2, rx_dma_buffer_, BLOCK_SIZE * kNumOutputs * 2) != HAL_OK) {
    LOGE("DMA NOT STARTED");
  }
  hi2s2.State = HAL_I2S_STATE_READY;
  HAL_I2S_Transmit_DMA(&hi2s2, tx_dma_buffer_, BLOCK_SIZE * kNumOutputs * 2);
    // __HAL_DMA_ENABLE(&hdma_spi2_tx);
    // __HAL_DMA_ENABLE(&hdma_spi2_rx);

  __HAL_I2S_ENABLE(&hi2s2);
#else
  __HAL_I2S_ENABLE(&hi2s2);
#endif

}

int16_t dac_buf_in[BLOCK_SIZE * 2];
int16_t dac_buf_out[BLOCK_SIZE * 2];

void dac_fill_buffer(uint16_t buffer) {
  int16_t *sp = (int16_t *)&rx_dma_buffer_[(buffer ^ 1) * BLOCK_SIZE * kNumOutputs];
  int16_t *dp = (int16_t *)&tx_dma_buffer_[(buffer ^ 0) * BLOCK_SIZE * kNumOutputs];
  for (size_t i = 0; i < BLOCK_SIZE * 2; ++i) {
    dac_buf_in[i] = sp[i];
    dp[i] = dac_buf_out[i];
  }
  // SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk
  *((volatile uint32_t *)0xE000ED04) = 1 << 28;
}