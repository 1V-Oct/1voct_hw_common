

#include "adc_helper.h"
#include "gpio.h"
#include "vo_logger.h"
#include "pots.h"
#include "cv.h"
#include "cv_inputs.h"
#include "cv_gates.h"
#include "adc_cv.h"

extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc1;

uint32_t         adc_time;
uint32_t         adc_last_time;

#if WITH_ADC_TICK_COUNTER
volatile uint8_t adc_tick[2];
#endif


// uint8_t adc_pots_values_mod[16];

static const adc_channel_list_t adc_pots_ch_list[2] = {
  {ADC_CHANNEL_1, ADC_REGULAR_RANK_1, GPIOC, GPIO_PIN_3},
  {ADC_CHANNEL_1, ADC_REGULAR_RANK_2, GPIOC, GPIO_PIN_3},
};

static const uint8_t adc_pots_channel_map[16] = {
  6, 15, 14, 13, 12, 11, 10, 9,
  8, 0, 1, 2, 3, 4, 5, 7};

DMA_BUFFER ALIGN_32BYTES(uint16_t adc_pots_dma_buf[2]);

void adc_pots_init(void) {
  adc_init_channels(&hadc3, adc_pots_ch_list, 2);
  LOGI("ADC Pots Init OK");

  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_pots_dma_buf, 1) != HAL_OK) {
    LOGE("Cannot start ADC3 DMA");
  }
}

// bool adc_pots_is_changed(uint8_t inp_num) {
//   is_changed =
//   return adc_pots.flags[inp_num] & ADC_POT_FLAGS_CHANGED;
// }

#define POT_MAPPING_MASK (0x0000fc00)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  static uint16_t addr = 0;

  if (hadc->Instance == ADC3) {
    // LOGI("%02x %04x %04x", addr, adc_pots_dma_buf[0], adc_pots_dma_buf[1]);

    // gate
    // adc_cv_values[14] =       adc_pots_dma_buf[1];

#if (WITH_ADC_FILTER == 1)
    int32_t  nnv = adc_pots_dma_buf[0] & ADC_POTS_FULL_MASK;
    int32_t  ov  = adc_pots_values[id];
    int32_t  nv  = nnv - ov;
    uint32_t anv = __builtin_abs(nv);
    uint8_t  lz  = __builtin_clz(anv);
    if (lz > 20)
      lz -= 20;
    else
      lz = 0;
#else
    uint16_t scan_val   = adc_pots_dma_buf[0] & ADC_POTS_FULL_MASK;
    uint8_t  id         = adc_pots_channel_map[addr];
    adc_pots.scan_val[id] = scan_val;
#endif
#if (WITH_ADC_POTS_MAPPING == 1)
    int16_t diff;
    uint16_t old_pot;
    uint16_t new_pot;
    old_pot              = adc_pots.val[id];
    new_pot              = scan_val;
    uint16_t       flags = adc_pots.flags[id];
    const uint16_t mask  = flags & ADC_POTS_FULL_MASK;

    diff = old_pot - new_pot;
    if (diff < 0) diff = -diff;
    uint16_t old_pot_masked = old_pot & mask;
    uint16_t new_pot_masked = new_pot & mask;
    if (diff > 256) {
      adc_pots.delta[id] += old_pot - new_pot;
      adc_pots.val[id] = scan_val;
      flags |= ADC_POT_FLAGS_CHANGED | ADC_POT_FLAGS_MASK_LONG | ADC_POT_FLAGS_MODIFIED;
      adc_pots.flags[id] = flags;
      // LOGI("M: %d %04x (%04x) %04x (%04x) %04x %d", id, old_pot_masked, old_pot, new_pot_masked, new_pot, flags, diff);
    }
#else
    adc_pots_values[id] = nnv; // ov + (nv >> lz);
#endif

    // if (id == 0)
    //   LOGI("V: %04x NV: %4x LZ: %d")
    gpio_write_pin(SW_ADDR1_GPIO_Port, SW_ADDR1_Pin, (addr & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write_pin(SW_ADDR2_GPIO_Port, SW_ADDR2_Pin, (addr & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write_pin(SW_ADDR3_GPIO_Port, SW_ADDR3_Pin, (addr & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write_pin(SW_ADDR4_GPIO_Port, SW_ADDR4_Pin, (addr & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    addr = (addr + 1) & 0x0f;

    // if (adc_dma_done & 0x01) {
    //   adc_dma_done &= ~0x01;
    //   // requested to stop
    // } else {
    //   ADC3->CR |= (uint32_t)(ADC_CR_ADSTART);
    // }
    // return;
    if (addr != 0) {
      ADC3->CR |= (uint32_t)(ADC_CR_ADSTART);
      return;
    } else {
#if WITH_ADC_TICK_COUNTER
      adc_tick[1]++;
#endif
    }
  }
  if (hadc->Instance == ADC1) {
#if WITH_ADC_TICK_COUNTER
    adc_tick[0]++;
#endif
    for (int i = 0; i < 12; i++) {
      int32_t nnv             = adc_cv_values[i] & 0xffc0;
      adc_cv_values_cached[i] = nnv;
    }
  }
};

// void                    HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
//   if (hadc->Instance != ADC1) LOGI("HIT2");
// };;
// void                    HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc) {
//   LOGI("HIT3");
// };;
// void                    HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
//   LOGI("HIT4");
// };;

void adc_pots_scan(void) {
  // LOGI("ADC3 (Pots) Scan");
#if 1
  ADC3->CR |= (uint32_t)(ADC_CR_ADSTART);
#else
  uint16_t addr;

  for (addr = 0; addr < 16; addr++) {
    uint16_t value_adc;
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 10);
    value_adc = HAL_ADC_GetValue(&hadc3);
    adc_pots_values[addr] = value_adc;
    // gate_ = HAL_ADC_GetValue(&hadc3);

    HAL_GPIO_WritePin(SW_ADDR1_GPIO_Port, SW_ADDR1_Pin, (addr & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW_ADDR2_GPIO_Port, SW_ADDR2_Pin, (addr & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW_ADDR3_GPIO_Port, SW_ADDR3_Pin, (addr & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW_ADDR4_GPIO_Port, SW_ADDR4_Pin, (addr & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // HAL_Delay(300);
  }
#endif
}

#if 0

extern "C" {
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
  LOGI("***");
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
  //LOGI("*** ADC1")
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}
}

#endif
