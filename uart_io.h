#pragma once
#include "app_config.h"


__CEXTERN_START

typedef void (*uart_rx_cb_t)(uint8_t c);

void uart_register_cb(uart_rx_cb_t f);
void uart_init(uint8_t id, uint32_t baud_rate);
// void uart_register_rx_cb(uart_rx_cb_t f);
void uart_tx(uint8_t id, uint8_t c);
const char * uart_puts(const char *buf);
int uart_printf(const char* fmt, ...);
void uart_it(uint8_t id);
void uart_send(uint8_t id, uint8_t *buf, uint8_t len);


#define VO_UART_PARITY_EVEN (0x80000000)

#define VO_UART_BAUD_RATE_921K         921600
#define VO_UART_BAUD_RATE_460K         460800
#define VO_UART_BAUD_RATE_115K         115200

__CEXTERN_END

