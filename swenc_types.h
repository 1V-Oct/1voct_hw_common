#pragma once

typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
} sw_pin_t;
