#pragma once

#include "app_config.h"
#include "swenc_types.h"


extern const sw_pin_t sw_pins[];

typedef struct {
  uint8_t   state[2];
} enc_state_t;

// typedef sw_state_t uint8_t;

#define SW_STATE    uint8_t   sw_state[SW_LAST];
#define ENC_STATE   uint8_t   enc_state[2];

extern SW_STATE
extern ENC_STATE


__CEXTERN_START

void sw_init(void);
void sw_scan(void);
void sw_scan_btn(void);
uint8_t sw_get_raw(uint8_t sw);
int32_t sw_enc_get_val(void);

inline bool sw_pressed(uint8_t sw) {
  return sw_state[sw] == 0x00;
}

inline bool sw_released(uint8_t sw) {
  return sw_state[sw] == 0x7f;
}

inline bool sw_just_pressed(uint8_t sw) {
  return sw_state[sw] == 0x80;
}

__CEXTERN_END