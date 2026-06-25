#pragma once
#include "app_config.h"

__CEXTERN_START

typedef struct {
  uint32_t      channel;
  uint32_t      rank;
  GPIO_TypeDef  *port;
  uint32_t      pin;
} adc_channel_list_t;




void adc_init_channels(ADC_HandleTypeDef *hadc, const adc_channel_list_t *list, size_t n);

__CEXTERN_END
