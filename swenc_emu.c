#include "app_config.h"
#include "swenc.h"
#include "drv_gfx.h"
#include "vo_logger.h"

#if WITH_SWENC_EMU
SW_STATE extern ENC_STATE

// void    sw_init(void) {}
// void    sw_scan(void) {}
// void    sw_scan_btn(void) {}
// uint8_t sw_get_raw(uint8_t sw) {}
// int32_t sw_enc_get_val(uint8_t idx) {}



SW_STATE
ENC_STATE

static int8_t enc_increment[ENC_LAST__];

void sw_init(void) {
  for (size_t i = 0; i < SW_LAST__; i++) {
    sw_state[i] = 0xff;
  }
  for (size_t i = 0; i < ENC_LAST__; i++) {
    enc_state[i][0] = enc_state[i][1] = 0xff;
  }
}

void sw_scan_btn(void) {
  // Key codes for '1' through '4'
  const uint16_t key_codes[] = {18, 19, 20, 21};
  // Keys '1' through '4' will map to SW_1 through SW_4
  for (size_t i = 0; i < SW_LAST__ && i < 4; i++) {
    uint8_t key_state = drv_gfx_get_key_state(key_codes[i]);
    sw_state[i] = (sw_state[i] << 1) | (key_state ? 0 : 1);
  }

  // Key 'Enter' will map to SW_ENC_CLICK
  uint8_t enter_key_state = drv_gfx_get_key_state(36); // kVK_Return
  sw_state[SW_ENC_CLICK] = (sw_state[SW_ENC_CLICK] << 1) | (enter_key_state ? 0 : 1);
  // if (sw_state[SW_ENC_CLICK] != 0xff) LOGI("EC %02x %d", sw_state[SW_ENC_CLICK], enter_key_state);
}

void sw_scan(void) {
  static uint8_t last_up = 0;
  static uint8_t last_down = 0;

  uint8_t up_pressed = drv_gfx_get_key_state(126); // Up Arrow
  uint8_t down_pressed = drv_gfx_get_key_state(125); // Down Arrow
  // LOGI("AR %d %D", up_pressed, down_pressed);
  if (up_pressed && !last_up) {
    enc_increment[0] += 1;
  }
  if (down_pressed && !last_down) {
    enc_increment[0] -= 1;
  }

  last_up = up_pressed;
  last_down = down_pressed;
}

int32_t sw_enc_get_val(uint8_t idx) {
  if (idx >= ENC_LAST__) return 0;
  int8_t inc = enc_increment[idx];
  enc_increment[idx] = 0;
  return inc;
}
#endif // WITH_SWENC_EMU