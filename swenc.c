#include "swenc.h"
#include "vo_logger.h"
#include "gpio.h"

// static sw_pin_t pins[] = {
//   {SW_1_GPIO_Port, SW_1_Pin},
//   {SW_2_GPIO_Port, SW_2_Pin},
//   {SW_3_GPIO_Port, SW_3_Pin},
//   {SW_4_GPIO_Port, SW_4_Pin},
//   {ENC_CLICK_GPIO_Port, ENC_CLICK_Pin}};

SW_STATE
ENC_STATE

static int8_t enc_increment = 0;


static inline uint8_t sw_read_gpio(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  // uint8_t val = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  // LOGI("Pin: %d", val);
  return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}


void sw_init(void) {

  size_t i;
  

  for (i = 0; i < SW_LAST; i++) {
    gpio_init(sw_pins[i].port, sw_pins[i].pin, GPIO_CONFIG(GPIO_MODE_INPUT, 0));
    // gpio_config_input()
    sw_state[i] = 0xff;
  }
  enc_state[0] = enc_state[1] = 0xff;
}


void sw_scan(void) {
  for (size_t i = 0; i < SW_LAST; i++) {
    sw_state[i] = (sw_state[i] << 1) | sw_read_gpio(sw_pins[i].port, sw_pins[i].pin);
  }

  enc_state[0] = (enc_state[0] << 1) | sw_read_gpio(ENC_A_GPIO_Port, ENC_A_Pin);
  enc_state[1] = (enc_state[1] << 1) | sw_read_gpio(ENC_B_GPIO_Port, ENC_B_Pin);

  uint8_t a         = enc_state[0];
  uint8_t b         = enc_state[1];
  
  if ((a & 0x03) == 0x02 && (b & 0x03) == 0x00) {
    enc_increment -= 1;
  } else {
    if ((b & 0x03) == 0x02 && (a & 0x03) == 0x00) {
      enc_increment += 1;
    }
  }

}

#if 0
uint8_t sw_get_raw(uint8_t sw) {
  return  sw_read_gpio(ENC_A_GPIO_Port, ENC_A_Pin);//sw_read_gpio(pins[sw].port, pins[sw].pin);
}
#endif

int32_t sw_enc_get_val(void) {
  int8_t inc = enc_increment;
  enc_increment = 0;
  return inc;
}
