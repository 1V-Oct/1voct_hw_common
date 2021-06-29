#pragma once
#include "app_config.h"


__CEXTERN_START

typedef void (*uart_rx_cb_t)(uint8_t c);

void uart_register_cb(uart_rx_cb_t f);
void uart_init(uint8_t id, uint32_t baud_rate);
// void uart_register_rx_cb(uart_rx_cb_t f);
void uart_tx(uint8_t id, uint8_t c);
void uart_puts(const char *buf);
void uart_printf(const char* fmt, ...);
void uart_it(uint8_t id);

__CEXTERN_END

