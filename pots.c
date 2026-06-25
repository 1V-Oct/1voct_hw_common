#include "pots.h"


ADC_POTS_VALUES;

void pots_init(void) {
// #if WITH_ADC_POTS
  // adc_pots_init();

  // int      i;
  // uint16_t flags;
  // for (i = 0; i < NUM_POTS; i++) {
  //   flags             = ADC_POT_FLAGS_MASK_LONG;
  //   adc_pots.flags[i] = flags;
  // }
}

void pots_reset_val(void) {
  size_t i;
  // LOGI("RESETTING");
  for(i = 0 ; i < NUM_POTS ; i++) {
    adc_pots.val[i] = adc_pots.scan_val[i];
    adc_pots.delta[i] = 0;
    // LOGI("%04x %04x", adc_pots.val[i], adc_pots.scan_val[i]);
  }
}

void pots_lock(uint16_t lock_mask) {
  int      i;
  uint16_t flags;
  for (i = 0; i < NUM_POTS; i++) {
    if (lock_mask & (1 << i)) {
      // flags             = adc_pots.flags[i];
      flags             = ADC_POT_FLAGS_MASK_SHORT | ADC_POT_FLAGS_LOCKED;
      adc_pots.flags[i] = flags;
      // LOGI("L: %d %04x", i, flags);
    }
  }
}

void pots_unlock_all(void) {
  int      i;
  uint16_t flags;
  for (i = 0; i < NUM_POTS; i++) {
    flags = adc_pots.flags[i];
    if (flags & ADC_POT_FLAGS_MODIFIED)
      flags = ADC_POT_FLAGS_MASK_SHORT;
    else
      flags &= ADC_POT_FLAGS_MASK_LONG;
    // flags = ADC_POT_FLAGS_MASK_LONG;
    adc_pots.flags[i] = flags;
  }
}

void pots_lock_all(void) {
  int i;
  for (i = 0; i < 16; i++) {
    pots_lock(1 << i);
  }
}
