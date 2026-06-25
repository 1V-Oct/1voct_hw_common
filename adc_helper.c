#include "adc_helper.h"
#include "vo_logger.h"
#include "vo_string.h"

void adc_init_channels(ADC_HandleTypeDef *hadc, const adc_channel_list_t *list, size_t n) {
  ADC_ChannelConfTypeDef        sConfig;
  GPIO_InitTypeDef              GPIO_InitStruct;

  size_t i;

  ZERO_STRUCT(GPIO_InitStruct);

  for (i = 0 ; i < n ; i++) {
    GPIO_InitStruct.Pin = list[i].pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(list[i].port, &GPIO_InitStruct);

    sConfig.Channel      = list[i].channel;
    sConfig.Rank         = list[i].rank;
//    sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;   /* Sampling time (number of clock cycles unit) */
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;   /* Sampling time (number of clock cycles unit) */
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
    sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
      LOGE("Failed to configure ADC channels");
    }
  }
}