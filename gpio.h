#pragma once
#include "app_config.h"

__CEXTERN_START

inline uint8_t gpio_read_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}

inline void gpio_reset_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
#if defined(STM32F103xB)
  GPIOx->BRR = GPIO_Pin;
#else
  GPIOx->BSRR = (uint32_t)GPIO_Pin << 16;
#endif
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


  // to toggle 
  GPIOA -> ODR ^= (1 << pinNummer);
  // to set 
  GPIOA -> BSRR = (1 << pinNummer);
  // to reset 
  GPIOA -> BRR = (1 << pinNummer);
  // or
  GPIOA -> BSRR = (1 << (pinNummer + 16));
*/




#define GPIO_CONFIG(MODE,CNF) (((CNF) << 2) | (MODE))

#if 0
inline void gpio_init(GPIO_TypeDef *GPIOx, const uint16_t pin_no, uint16_t config) {
  // if (pull == GPIO_NOPULL) {
    
  // }
  // GPIOx
  volatile uint32_t *base;
  uint16_t pin;

  if (pin_no >= 8) {
    base = &GPIOx->CRH;
    pin = pin_no - 8;
  } else {
    base = &GPIOx->CRL;
    pin = pin_no;
  }
  pin = pin << 2; // multiply pib by 4;
  *base &= 0b1111 << pin;
  *base |= config << pin;

  // if (pin_no >= 8) {
  //   GPIOx->CRL |= config << pin_no;
  // } else {
  //   GPIOx->CRH = config << (pin_no - 8);
  // }
}

#endif

// inline void gpio_inp_pullup(GPIO_TypeDef *GPIOx, GPIO_Pin uint8_t pull) {

// }

__CEXTERN_END