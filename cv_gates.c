#include "cv_gates.h"
#include "gpio.h"
#include "vo_string.h"




typedef struct {
  GPIO_TypeDef *port;
  uint16_t       pin;
} gpio_pin_t;

static const gpio_pin_t cvy_gpio[] = {
  { GPIOC, GPIO_PIN_0 },
  { GPIOE, GPIO_PIN_4 },
  { GPIOC, GPIO_PIN_1 },
  { GPIOC, GPIO_PIN_2 },
};

/**
 * @brief Initializes CV Gates/Clocks inputs which are basically 1/0 state GPIO inputs
 * 
 */
void cv_gates_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  ZERO_STRUCT(GPIO_InitStruct);

  // __GPIOE_CLK_ENABLE();
  // __GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // GPIOC-> = (PORTA->regs->CRL & 0x00F00F00) | 0x88000080 |0x00033003;

}

/**
 * @brief Returns immediate state of gate
 * 
 * @param cvy gate number
 * @return uint16_t status of gate (binary 1/0)
 */
uint16_t  cv_gate_get_value(uint16_t cvy) {
  // return HAL_GPIO_ReadPin(cvy_gpio[cvy].port, cvy_gpio[cvy].pin);
  return gpio_read_pin(cvy_gpio[cvy].port, cvy_gpio[cvy].pin);

}