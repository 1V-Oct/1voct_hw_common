#pragma once
#include "app_config.h"

__CEXTERN_START

inline uint8_t gpio_read_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}

inline void gpio_reset_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  // GPIOx->BRR = GPIO_Pin;
  GPIOx->BSRR = (uint32_t)GPIO_Pin << 16;
}

inline void gpio_set_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  GPIOx->BSRR = GPIO_Pin;
}

inline void gpio_write_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
  if (PinState != GPIO_PIN_RESET) {
    GPIOx->BSRR = GPIO_Pin;
  } else {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16;
  }
}

/*
CNFy[1:0]:Portxconfigurationbits(y=0..7)
These bits are written by software to configure the corresponding I/O port. Refer to Table 20: Port bit configuration table.
In input mode (MODE[1:0]=00):
00: Analog mode
01: Floating input (reset state)
10: Input with pull-up / pull-down
11: Reserved
In output mode (MODE[1:0] >00):
00: General purpose output push-pull
01: General purpose output Open-drain
10: Alternate function output Push-pull
11: Alternate function output Open-drain
MODEy[1:0]:Portxmodebits(y=0..7)
These bits are written by software to configure the corresponding I/O port. Refer to Table 20: Port bit configuration table.
00: Input mode (reset state)
01: Output mode, max speed 10 MHz.
10: Output mode, max speed 2 MHz.
11: Output mode, max speed 50 MHz.
*/

#define GPIO_CONFIG(MODE,CNF) (((CNF) << 2) | (MODE))

inline void gpio_init(GPIO_TypeDef *GPIOx, const uint16_t pin_no, uint16_t config) {
  // if (pull == GPIO_NOPULL) {
    
  // }
  // GPIOx
  // if (pin_no >= 8) {
  //   GPIOx->CRL = config << pin_no;
  // } else {
  //   GPIOx->CRH = config << (pin_no - 8);
  // }
}

// inline void gpio_inp_pullup(GPIO_TypeDef *GPIOx, GPIO_Pin uint8_t pull) {

// }

__CEXTERN_END