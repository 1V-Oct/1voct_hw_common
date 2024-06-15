#pragma once

typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
} sw_pin_t;

// typedef struct {
//   GPIO_TypeDef *port[2];
//   uint16_t      pin[2];
// } sw_enc_t;