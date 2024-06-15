#include "app_config.h"
#include "hw/bsp/board.h"
#include "vo_common.h"
#include "vo_logger.h"
#include "vo_string.h"

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

#if WITH_DAC_WM8731_SAI

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

#endif

static void wm8731_write_reg(uint8_t reg, uint16_t value) {
  uint16_t tmp = 0;
  uint8_t  data[2];

  tmp     = ((uint16_t)reg << 9) | value;
  data[0] = (tmp & 0xFF00) >> 8;
  data[1] = tmp & 0x00FF;

  HAL_I2C_Master_Transmit(&hi2c2, WM8731_ADDR, data, 2, HAL_MAX_DELAY);
#if WITH_DAC_WM8731_TWIN
  HAL_I2C_Master_Transmit(&hi2c2, WM8731_ADDR + 2, data, 2, HAL_MAX_DELAY);
#endif
}

#if 1
// #define AUDIO_BUFFER_SIZE 2560
#define AUDIO_BUFFER_SIZE 1280
DMA_BUFFER static int16_t audio_data[4 * AUDIO_BUFFER_SIZE];
DMA_BUFFER static int16_t audio_data_rx[4 * AUDIO_BUFFER_SIZE];

static uint16_t dac_val[2] = {0x8000, 0x8000};
static void     fill_audio_buffer(void) {
  // dac_val[0] += 100;
  // dac_val[0] ^= 0xffff;

  dac_val[1] ^= 0xffff;
  for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
    // audio_data[i * 2 + 0]     = __builtin_bswap16(dac_val);
    // audio_data[i * 2 + 1]     = __builtin_bswap16(dac_val);
    int16_t value = (int16_t)(32000.0 * sin(i * (2.0 * M_PI / AUDIO_BUFFER_SIZE)));
    dac_val[0]    = value;
    ;
    audio_data[i * 2 + 0] = dac_val[0];
    audio_data[i * 2 + 1] = dac_val[1];
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

  HAL_Delay(1000);
  // fill_audio_buffer();
  // HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
  // while (1) {
  //   // HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE, HAL_MAX_DELAY);
  //   // HAL_SAI_Transmit(&hsai_BlockA2, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE, HAL_MAX_DELAY);
  //   HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
  //   HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
  //   // HAL_Delay(1000);
  //   // HAL_Delay(1000);
  //   // HAL_SAI_Transmit(&hsai_BlockB1, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE, HAL_MAX_DELAY);
  //   // HAL_Delay(1000);
  //   // HAL_SAI_Transmit(&hsai_BlockB2, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE, HAL_MAX_DELAY);
  //   // HAL_Delay(1000);
  //   // LOGI("#");
  // }
}

#if WITH_DAC_WM8731_I2S
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
#else

#endif

#if WITH_DAC_WM8731_SAI

static void dac_sai_init(void) {
  hsai_BlockA1.Instance            = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode      = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro        = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt     = SAI_SYNCEXT_OUTBLOCKA_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
    Error_Handler();
  }
  hsai_BlockB1.Instance            = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode      = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.Synchro        = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt     = SAI_SYNCEXT_OUTBLOCKA_ENABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
    Error_Handler();
  }
  hsai_BlockA2.Instance            = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode      = SAI_MODESLAVE_TX;
  hsai_BlockA2.Init.Synchro        = SAI_SYNCHRONOUS_EXT_SAI1;
  hsai_BlockA2.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
    Error_Handler();
  }
  hsai_BlockB2.Instance            = SAI2_Block_B;
  hsai_BlockB2.Init.AudioMode      = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.Synchro        = SAI_SYNCHRONOUS_EXT_SAI1;
  hsai_BlockB2.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK) {
    Error_Handler();
  }
}

#define NOT_CONFIGURED_PLL (1)

static uint32_t SAI1_client = 0;
static uint32_t SAI2_client = 0;

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai) {

  GPIO_InitTypeDef GPIO_InitStruct;
#if NOT_CONFIGURED_PLL
#pragma message "Not configured PLL"
  __HAL_RCC_SAI1_CONFIG(RCC_SAI1CLKSOURCE_PLL);
  __HAL_RCC_SAI23_CONFIG(RCC_SAI2CLKSOURCE_PLL);
#else
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection     = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.Sai1ClockSelection       = RCC_SAI1CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
  PeriphClkInitStruct.Sai23ClockSelection  = RCC_SAI23CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
#endif
  /* SAI1 */
  if (hsai->Instance == SAI1_Block_A) {
    /* Peripheral clock enable */

    if (SAI1_client == 0) {
      __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client++;

    /**SAI1_A_Block_A GPIO Configuration
    PE2     ------> SAI1_MCLK_A
    PE4     ------> SAI1_FS_A
    PE5     ------> SAI1_SCK_A
    PE6     ------> SAI1_SD_A
    */
    GPIO_InitStruct.Pin       = SAI_MCLK_Pin | SAI_FS_Pin | SAI_SCK_Pin | SAI1_TX_A_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_sai1_a.Instance                 = DMA1_Stream0;
    hdma_sai1_a.Init.Request             = DMA_REQUEST_SAI1_A;
    hdma_sai1_a.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai1_a.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai1_a.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.Mode                = DMA_CIRCULAR;
    hdma_sai1_a.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_sai1_a.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK) {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai, hdmarx, hdma_sai1_a);
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai1_a);
  }
  if (hsai->Instance == SAI1_Block_B) {
    /* Peripheral clock enable */

    if (SAI1_client == 0) {
      __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client++;

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    */
    GPIO_InitStruct.Pin       = SAI1_RX_B_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(SAI1_RX_B_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_sai1_b.Instance                 = DMA1_Stream1;
    hdma_sai1_b.Init.Request             = DMA_REQUEST_SAI1_B;
    hdma_sai1_b.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sai1_b.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai1_b.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_b.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_sai1_b.Init.Mode                = DMA_CIRCULAR;
    hdma_sai1_b.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_sai1_b.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai1_b) != HAL_OK) {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai, hdmarx, hdma_sai1_b);
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai1_b);
  }
  /* SAI2 */
  if (hsai->Instance == SAI2_Block_A) {
    /* Peripheral clock enable */
    if (SAI2_client == 0) {
      __HAL_RCC_SAI2_CLK_ENABLE();
    }
    SAI2_client++;

    /**SAI2_A_Block_A GPIO Configuration
    PD11     ------> SAI2_SD_A
    */
    GPIO_InitStruct.Pin       = SAI2_TX_A_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(SAI2_TX_A_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_sai2_a.Instance                 = DMA1_Stream2;
    hdma_sai2_a.Init.Request             = DMA_REQUEST_SAI2_A;
    hdma_sai2_a.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_a.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai2_a.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai2_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai2_a.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_sai2_a.Init.Mode                = DMA_CIRCULAR;
    hdma_sai2_a.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_sai2_a.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai2_a) != HAL_OK) {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai, hdmarx, hdma_sai2_a);
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai2_a);
  }
  if (hsai->Instance == SAI2_Block_B) {
    /* Peripheral clock enable */
    if (SAI2_client == 0) {
      __HAL_RCC_SAI2_CLK_ENABLE();
    }
    SAI2_client++;

    /**SAI2_B_Block_B GPIO Configuration
    PA0     ------> SAI2_SD_B
    */
    GPIO_InitStruct.Pin       = SAI2_RX_B_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(SAI2_RX_B_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_sai2_b.Instance                 = DMA1_Stream3;
    hdma_sai2_b.Init.Request             = DMA_REQUEST_SAI2_B;
    hdma_sai2_b.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sai2_b.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai2_b.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai2_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai2_b.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_sai2_b.Init.Mode                = DMA_CIRCULAR;
    hdma_sai2_b.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_sai2_b.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai2_b) != HAL_OK) {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai, hdmarx, hdma_sai2_b);
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai2_b);
  }
}

#endif

void dac_priv_init(void) {
#if !(WITH_DAC_WM8731_I2S || WITH_DAC_WM8731_SAI)
#error No DAC IFX defined
#endif
#if WITH_DAC_WM8731_I2S
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
#endif
#if WITH_DAC_WM8731_SAI
  dac_sai_init();
#endif

  wm8731_init();
  // __HAL_SAI_DISABLE(&hsai_BlockA1);
  // __HAL_SAI_DISABLE(&hsai_BlockA2);
  // __HAL_SAI_DISABLE(&hsai_BlockB1);
  // __HAL_SAI_DISABLE(&hsai_BlockB2);

  // hsai_BlockA1.State = HAL_SAI_STATE_READY;
  // HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
  // HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);

  // hsai_BlockA2.State = HAL_SAI_STATE_READY;
  // HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
  // HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *)audio_data, 2 * AUDIO_BUFFER_SIZE);
  // __HAL_SAI_ENABLE(&hsai_BlockA1);
  // __HAL_SAI_ENABLE(&hsai_BlockA2);

  // while(1) {
  // fill_audio_buffer();

  // HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)audio_data, AUDIO_BUFFER_SIZE * 2, HAL_MAX_DELAY);
  // HAL_SAI_Receive(&hsai_BlockB1, (uint8_t *)audio_data_rx, AUDIO_BUFFER_SIZE * 2, HAL_MAX_DELAY);
  // HAL_SAI_Transmit(&hsai_BlockA2, (uint8_t *)audio_data, AUDIO_BUFFER_SIZE * 2, HAL_MAX_DELAY);
  // HAL_SAI_Receive(&hsai_BlockB2, (uint8_t *)audio_data_rx, AUDIO_BUFFER_SIZE * 2, HAL_MAX_DELAY);
  // HAL_SAI_Receive(&hsai_BlockB1, (uint8_t *)audio_data_rx, 2 * AUDIO_BUFFER_SIZE, HAL_MAX_DELAY);
  //   LOGI("x");
  // }

#if WITH_DAC_WM8731_I2S
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

#else
#endif
  __HAL_I2S_ENABLE(&hi2s2);
#endif

  hsai_BlockA1.State = HAL_SAI_STATE_READY;
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)tx_dma_buffer_, 4 * BLOCK_SIZE);
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)rx_dma_buffer_, 4 * BLOCK_SIZE);

  hsai_BlockA2.State = HAL_SAI_STATE_READY;
  HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t *)tx_dma_buffer_, 4 * BLOCK_SIZE);
  HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *)rx_dma_buffer_, 4 * BLOCK_SIZE);
}

int16_t dac_buf_in[BLOCK_SIZE * 2];
int16_t dac_buf_out[BLOCK_SIZE * 2];

void dac_fill_buffer(uint16_t buffer) {
  int16_t *sp = (int16_t *)&rx_dma_buffer_[(buffer ^ 1) * BLOCK_SIZE * kNumOutputs];
  int16_t *dp = (int16_t *)&tx_dma_buffer_[(buffer ^ 0) * BLOCK_SIZE * kNumOutputs];
  for (size_t i = 0; i < BLOCK_SIZE * 2; ++i) {
    dac_buf_in[i] = sp[i];
    dp[i]         = dac_buf_out[i];
  }
  // SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk
  *((volatile uint32_t *)0xE000ED04) = 1 << 28;
}