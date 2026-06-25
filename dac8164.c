// Copyright 2017 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Driver for DAC.
#if 1
#include "dac.h"
#include "vo_logger.h"

#include "engine.h"

#include "mod_buffers.h"

extern DMA_HandleTypeDef hdma_spi2_tx;
extern I2S_HandleTypeDef hi2s2;

static uint8_t dac_stop_calls = 0;
#if 0
void dac_stop(void) {
  dac_stop_calls++;
  //__HAL_DMA_DISABLE(&hdma_spi2_tx);
  LOGI("DMA Stop %d", dac_stop_calls);
  __HAL_I2S_DISABLE(&hi2s2);
  HAL_I2S_DMAStop(&hi2s2);
  HAL_Delay(300);
}

uint8_t dac_is_stopped(void) {
  return dac_stop_calls;
}
void dac_start(void) {
  dac_stop_calls--;
  LOGI("DMA Start %d * %s *", dac_stop_calls, dac_stop_calls ? "NOT STARTED" : "STARTED");
  if (dac_stop_calls) return;
  
  //__HAL_DMA_ENABLE(&hdma_spi2_tx);
  //HAL_I2S_DMAResume(&hi2s2);
  __HAL_I2S_ENABLE(&hi2s2);
  dac_init();
}
#else
void dac_pause(void) {
  dac_stop_calls++;
  HAL_I2S_DMAPause(&hi2s2);
}

void dac_resume(void) {
  dac_stop_calls--;
  if (dac_stop_calls)
    return;
  HAL_I2S_DMAResume(&hi2s2);
}

#endif
static HAL_StatusTypeDef my_wait_flag(I2S_HandleTypeDef *hi2s, uint32_t Flag, FlagStatus State, uint32_t Timeout) {
  uint32_t tickstart;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait until flag is set to status*/
  while (((__HAL_I2S_GET_FLAG(hi2s, Flag)) ? SET : RESET) != State) {
    if (Timeout != HAL_MAX_DELAY) {
      if (((HAL_GetTick() - tickstart) >= Timeout) || (Timeout == 0UL)) {
        /* Set the I2S State ready */
        hi2s->State = HAL_I2S_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}
HAL_StatusTypeDef my_simple_transmit(I2S_HandleTypeDef *hi2s, uint32_t data) {
  /* Check if the I2S is already enabled */
  if ((hi2s->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
    /* Enable I2S peripheral */
    __HAL_I2S_ENABLE(hi2s);
  }

  /* Start the transfer */
  SET_BIT(hi2s->Instance->CR1, SPI_CR1_CSTART);

  /* Wait until TXP flag is set */
  if (my_wait_flag(hi2s, I2S_FLAG_TXP, SET, 100000) != HAL_OK) {
    LOGE("Timeout");
    /* Set the error code */
    SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_TIMEOUT);
    hi2s->State = HAL_I2S_STATE_READY;
    __HAL_UNLOCK(hi2s);
    return HAL_ERROR;
  }

  /* Transmit data in 32 Bit mode */
  hi2s->Instance->TXDR = data;

  /* Wait until TXP flag is set */
  if (my_wait_flag(hi2s, I2S_FLAG_TXP, SET, 100000) != HAL_OK) {
    /* Set the error code */
    LOGE("Timeout");
    SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_TIMEOUT);
    hi2s->State = HAL_I2S_STATE_READY;
    __HAL_UNLOCK(hi2s);
    return HAL_ERROR;
  }

  /* Check if an underrun occurs */
  if (__HAL_I2S_GET_FLAG(hi2s, I2S_FLAG_UDR) == SET) {
    LOGW("Underrun");
    /* Clear underrun flag */
    __HAL_I2S_CLEAR_UDRFLAG(hi2s);

    /* Set the error code */
    SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_UDR);
  }

  hi2s->State = HAL_I2S_STATE_READY;
  __HAL_UNLOCK(hi2s);
  return HAL_OK;
}

// DMA_BUFFER
// DMA_BUFFER uint16_t tx_dma_buffer_[2 * kBlockSize * kNumOutputs * 2];

#if 1
DMA_BUFFER uint16_t tx_dma_buffer_[2 * BLOCK_SIZE * kNumOutputs * 2];
#else
// DMA Buffer in D3 Memory
uint16_t *tx_dma_buffer_ = (uint16_t *)0x38000000; // 0x38000000;
#endif

void writespi(uint16_t data) {
  /* Start the transfer */
  SET_BIT(SPI2->CR1, SPI_CR1_CSTART);
  while (!(SPI2->SR & SPI_SR_TXP)) {
  }
  SPI2->TXDR = data;
}

void i2s_write32(uint32_t data) {
  /* Start the transfer */
  SET_BIT(SPI2->CR1, SPI_CR1_CSTART);
  while (!(SPI2->SR & SPI_SR_TXP)) {
  }
  SPI2->TXDR = data;
}

void dac_init(void) {
  
#if 0  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  SPI_I2S_DeInit(SPI2);
  I2S_InitTypeDef i2s_init;
  i2s_init.I2S_AudioFreq = sample_rate * kNumCvOutputs >> 1;
  I2S_Init(SPI2, &i2s_init);
  I2S_Cmd(SPI2, ENABLE);
#endif

  // hi2s2.Instance = SPI2;
  // hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  // hi2s2.Init.Standard = I2S_STANDARD_PCM_SHORT;
  // hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  // hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  // hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;
  // hi2s2.Init.CPOL = I2S_CPOL_LOW;
  // hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  // hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  // hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  // hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  // if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
  //   Error_Handler();
  // }

  // instance_ = this;
  // block_size_ = block_size;
  // callback_ = NULL;

  int i;
  for (i = 0; i < 2 * BLOCK_SIZE * kNumOutputs * 2; i++) {
    tx_dma_buffer_[i] = 0;
  }
#if 0
  tx_dma_buffer_[0] = 0x0000;
  tx_dma_buffer_[1] = 0x0000;
  tx_dma_buffer_[2] = 0x0000;
  tx_dma_buffer_[3] = 0x0000;
  tx_dma_buffer_[4] = 0xaa00;
  tx_dma_buffer_[5] = 0x0000;
  tx_dma_buffer_[6] = 0x0000;
  tx_dma_buffer_[7] = 0x103f;
  tx_dma_buffer_[8] = 0x0000;
  tx_dma_buffer_[9] = 0x123f;
  tx_dma_buffer_[10] = 0x0000;
  tx_dma_buffer_[11] = 0x143f;
  tx_dma_buffer_[12] = 0x0000;
  tx_dma_buffer_[13] = 0x163f;
  tx_dma_buffer_[14] = 0x0000;
  tx_dma_buffer_[15] = 0x0000;
  tx_dma_buffer_[16] = 0x0000;
  tx_dma_buffer_[17] = 0x0000;
  tx_dma_buffer_[18] = 0x0000;
  tx_dma_buffer_[19] = 0x0000;
#endif

  HAL_I2S_Transmit_DMA(&hi2s2, tx_dma_buffer_, BLOCK_SIZE * kNumOutputs * 2);

#if 0
  writespi(0x10ff);
  writespi(0x0000);
  writespi(0x127f);
  writespi(0x0000);
  writespi(0x14ff);
  writespi(0x0000);
  writespi(0x163f);
  writespi(0x0000);
#endif
  // for (int i = 0 ; i < (kDacBlockSize * kNumDacOutputs) ; i+=4) {
  //   tx_dma_buffer_[i] = 0x105f;
  //   tx_dma_buffer_[i+1] = 0xff00;
  //   tx_dma_buffer_[i] = 0x12cf;
  //   tx_dma_buffer_[i+1] = 0xff00;
  // }
}

#if 0
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_Init(GPIOA, &gpio_init);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_5);
#endif

uint16_t xx    = 0;
uint16_t timer = 0;


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
      sample = (uint16_t)(mod_buf_vout[j][i] * -32700.f + 32700.f);
      // sample = 0x7fff;
      *p++   = sample << 8;
      *p++   = 0x1000 | ((3 - j) << 9) | (sample >> 8);
    }
  }
  *((volatile uint32_t *)0xE000ED04) = 1 << 28;
}

#if 0

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  dac_fill_buffer(1);
  HAL_GPIO_WritePin(DBG_PIN_GPIO_Port, DBG_PIN_Pin, GPIO_PIN_RESET);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
 //LOGI("HT");
  dac_fill_buffer(0);
  HAL_GPIO_WritePin(DBG_PIN_GPIO_Port, DBG_PIN_Pin, GPIO_PIN_RESET);
}
#endif

__CEXTERN_START

void DMA1_Stream5_IRQHandler() {
  uint32_t flags = DMA1->HISR;
  DMA1->HIFCR    = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5;
  if (flags & DMA_HISR_TCIF5) {
    dac_fill_buffer(1);
  } else if (flags & DMA_HISR_HTIF5) {
    dac_fill_buffer(0);
  }
}

__CEXTERN_END


#endif