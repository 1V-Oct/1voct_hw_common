#pragma once
#include "app_config.h"

float drv_gfx_get_pot_value(int i);

__CEXTERN_START

#define NUM_POTS           (16)
#define ADC_POTS_FULL_MASK (0xffc0)
typedef struct {
  uint16_t val[NUM_POTS];
  uint16_t scan_val[NUM_POTS];
  uint16_t flags[NUM_POTS];
  int16_t  delta[NUM_POTS];
} adc_pots_t;

extern adc_pots_t adc_pots;

#define ADC_POT_FLAGS_CHANGED    (1 << 0)
#define ADC_POT_FLAGS_LOCKED     (1 << 1)
#define ADC_POT_FLAGS_MODIFIED   (1 << 2)
#define ADC_POT_FLAGS_PRELOCKED  (1 << 3)
#define ADC_POT_FLAGS_POSITIONED (1 << 4)
#define ADC_POT_FLAGS_MASK_SHORT (0xf800)
#define ADC_POT_FLAGS_MASK_LONG  (0xfe00)

#if (WITH_OLD_ADC_POTS == 1)
#define ADC_POTS_VALUES     uint16_t adc_pots_values[16];
// extern uint16_t adc_pots_values_mod;
#define ADC_POTS_RESOLUTION (16)
#define ADC_POTS_VAL_SHIFT  (16 - ADC_POTS_RESOLUTION)
inline uint16_t adc_pots_get_value_u16(uint16_t pot) {
  return adc_pots_values[pot] << ADC_POTS_VAL_SHIFT;
}
#else
#define ADC_POTS_VALUES        adc_pots_t adc_pots;
#define ADC_POT_VALUE(__POT__) adc_pots.val[__POT__]
#endif

extern ADC_POTS_VALUES;

void pots_init(void);
void adc_pots_scan(void);

void adc_pots_print(void);

void pots_unlock_all(void);
void pots_lock_all(void);

void pots_reset_val(void);

void        pots_lock(uint16_t lock_mask);
inline void pots_lock_by_id(uint16_t lock_id) {
  pots_lock(1 << lock_id);
}

inline uint16_t adc_pots_get_value_u16(uint16_t pot) {
  return ADC_POT_VALUE(pot);
}
inline float adc_pots_get_value_f(uint16_t pot) {
  adc_pots.flags[pot] &= ~ADC_POT_FLAGS_CHANGED;
  return ((float)(adc_pots_get_value_u16(pot) >> 6) / 1023.f);
}

inline bool adc_pots_is_locked(uint8_t inp_num) {
  return adc_pots.flags[inp_num] & ADC_POT_FLAGS_LOCKED;
}

inline bool adc_pots_is_positioned(uint8_t inp_num) {
  return adc_pots.flags[inp_num] & ADC_POT_FLAGS_POSITIONED;
}

// bool adc_pots_is_changed(uint8_t inp_num);

inline bool adc_pots_is_changed(uint8_t inp_num) {
  return adc_pots.flags[inp_num] & ADC_POT_FLAGS_CHANGED;
}
// /**
//  * @brief Reset the table of modifications
//  *
//  */
// inline void adc_pots_reset_mods(void) {
//   adc_pots_values_mod = 0;
// }

enum {
  ATT_1 = 8,
  ATT_2 = 7,
  ATT_3 = 6,
  ATT_4 = 5,
  ATT_5 = 4,
  ATT_6 = 3,
  ATT_7 = 2,
  ATT_8 = 1,
  POT_1 = 9,
  POT_2 = 10,
  POT_3 = 11,
  POT_4 = 12,
  POT_5 = 13,
  POT_6 = 14,
  POT_7 = 0,
  POT_8 = 15
};

enum {
  ADC_POTS_L1 = 0,
  ADC_POTS_L2,
  ADC_POTS_L3,
  ADC_POTS_L4,
  ADC_POTS_L5,
  ADC_POTS_L6,
  ADC_POTS_L7,
  ADC_POTS_L8,
  ADC_POTS_A1,
  ADC_POTS_A2,
  ADC_POTS_A3,
  ADC_POTS_A4,
  ADC_POTS_A5,
  ADC_POTS_A6,
  ADC_POTS_A7,
  ADC_POTS_A8,
};

__CEXTERN_END
