#include "swenc.h"
#include "gpio.h"
#include "vo_logger.h"

SW_STATE
ENC_STATE

static int8_t enc_increment[ENC_LAST__];

static inline uint8_t sw_read_gpio(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  // uint8_t val = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  // LOGI("Pin: %d", val);
  return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}

void sw_init(void) {

  size_t i;

  for (i = 0; i < SW_LAST__; i++) {
    // gpio_init(sw_pins[i].port, sw_pins[i].pin, GPIO_CONFIG(GPIO_MODE_INPUT, 0));
    // gpio_config_input()
    sw_state[i] = 0xff;
  }
  for (i = 0; i < ENC_LAST__; i++) {
    enc_state[i][0] = enc_state[i][1] = 0xff;
  }
}

void sw_scan_btn(void) {
  for (size_t i = 0; i < SW_LAST__; i++) {
    sw_state[i] = (sw_state[i] << 1) | sw_read_gpio(sw_pins[i].port, sw_pins[i].pin);
  }
}

void sw_scan(void) {
  int i;
  for (i = 0; i < ENC_LAST__; i++) {
    enc_state[i][0] = (enc_state[i][0] << 1) | sw_read_gpio(enc_pins[i][0].port, enc_pins[i][0].pin);
    enc_state[i][1] = (enc_state[i][1] << 1) | sw_read_gpio(enc_pins[i][1].port, enc_pins[i][1].pin);

    uint8_t a       = enc_state[i][0];
    uint8_t b       = enc_state[i][1];

    if ((a & 0x03) == 0x02 && (b & 0x03) == 0x00) {
      enc_increment[i] -= 1;
    } else {
      if ((b & 0x03) == 0x02 && (a & 0x03) == 0x00) {
        enc_increment[i] += 1;
      }
    }
  }
}

#if 0
uint8_t sw_get_raw(uint8_t sw) {
  return  sw_read_gpio(ENC_A_GPIO_Port, ENC_A_Pin);//sw_read_gpio(pins[sw].port, pins[sw].pin);
}
#endif

int32_t sw_enc_get_val(uint8_t idx) {
  int8_t inc         = enc_increment[idx];
  enc_increment[idx] = 0;
  return inc;
}
