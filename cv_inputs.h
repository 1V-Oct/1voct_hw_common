#pragma once
#include "app_config.h"

__CEXTERN_START
// #define CV_INPUTS    float cv_in[24];

// extern CV_INPUTS

typedef enum {
  INPUT_CV = 0,
  INPUT_LVL,
  INPUT_ATT
} cv_input_type_t;

#define CV_INP_

extern uint16_t adc_cv_values_cached[16];

#define S16_TO_F (-1.0f / 32768.0f)


void         cv_init(void);
void         cv_scan(void);
void         cv_normalization_probe(uint8_t set);
float        cv_get_note(uint8_t ch);
float        cv_get_note_f(uint8_t ch);
void         cv_reset_inputs(void);
static inline float cv_get_value_f(uint16_t pot) {
#if WITH_ADC_POTS
  return (((float)adc_cv_values_cached[pot]) - 32767.0f) * S16_TO_F;
#else
  return 0;
#endif
}

void     cv_gates_init(void);
uint16_t cv_gate_get_value(uint16_t cvy);

__CEXTERN_END