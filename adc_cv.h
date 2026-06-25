#pragma once
#include "app_config.h"
#include "cv_cal.h"
#include "cv.h"
__CEXTERN_START

#define ADC_CV_CHANNEL_COUNT (12)

#define ADC_CV_VALUES        uint16_t adc_cv_values[ADC_CV_CHANNEL_COUNT + 1]

extern uint16_t adc_cv_values_cached[16];
extern uint8_t  adc_cv_plug[16];

extern ADC_CV_VALUES;

#define ADC_CV_RESOLUTION (16)
#define ADC_CV_VAL_SHIFT  (16 - ADC_CV_RESOLUTION)



void adc_cv_init(void);
void adc_cv_loop(void);
void adc_cv_scan(void);

void adc_cv_print(void);


// void cv_cal_voct(void);
void cv_cal_voct_full(void);

inline uint16_t adc_cv_get_value_u16(uint16_t id) {
  return adc_cv_values_cached[id] << ADC_CV_VAL_SHIFT;
}

extern volatile uint32_t adc_dma_done;

inline void adc_cv_start_dma(void) {
  ADC1->CR |= (uint32_t)(ADC_CR_ADSTART);
}
__CEXTERN_END