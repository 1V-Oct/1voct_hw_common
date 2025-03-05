
#include "app_config.h"
#include "vo_common.h"
#include "vo_logger.h"
#include "vo_string.h"
#include "hw/bsp/board.h"

extern I2C_HandleTypeDef hi2c2;

DMA_HandleTypeDef hdma_spi2_tx;
// DMA_HandleTypeDef hdma_spi2_rx;




/**
* @brief I2S MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspInitXXX(I2S_HandleTypeDef* hi2s) {
  GPIO_InitTypeDef GPIO_InitStruct;
  ZERO_STRUCT(GPIO_InitStruct);
  if (hi2s->Instance == SPI2) {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB10     ------> I2S2_CK
    PB12     ------> I2S2_WS
    PB15     ------> I2S2_SDO
    */
#if BOARD_VER == 3
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_15;
#elif (BOARD_VER == 4) || (BOARD_VER == 5)
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_15;
#else
#error Unknown board version
#endif

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2S2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance                 = DMA1_Stream5;
    hdma_spi2_tx.Init.Request             = DMA_REQUEST_SPI2_TX;
    hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_spi2_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_spi2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmatx, hdma_spi2_tx);
  }
}


void {
  HAL_I2S_Transmit_DMA(&hi2s2, tx_dma_buffer_, BLOCK_SIZE * kNumOutputs * 2);
}
void dac_fill_buffer(uint16_t buffer) {
  // dac_dwt_val += DWT->CYCCNT / 1000;
  // if (dac_dwt_cnt++ > 375) {
  //   dac_cpu_usage = dac_dwt_val;
  //   dac_dwt_val = 0;
  //   dac_dwt_cnt = 0;
  // }
  //buffer = 0;
  // uart_tx(UART_PUTS_PORT_ID, '-');
  // eng_process(buffer);
  uint16_t *p = &tx_dma_buffer_[buffer * BLOCK_SIZE * kNumOutputs * 2];
  for (size_t i = 0; i < BLOCK_SIZE; ++i) {
    for (size_t j = 0; j < kNumOutputs; ++j) {
      uint16_t sample;
      sample = uint16_t(mod_buf_vout[j][i] * -32700.f + 32700.f);
      // sample = 0x0;
      *p++   = sample << 8;
      *p++   = 0x1000 | ((3 - j) << 9) | (sample >> 8);
    }
  }
  *((volatile uint32_t *)0xE000ED04) = 1 << 28;
}