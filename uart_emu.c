#include "app_config.h"
#include <stdarg.h>
#include "vo_string.h"
#include "vo_logger.h"
#include "vo_memory.h"
#if WITH_UART_EMU


void uart_init(uint8_t id, uint32_t baud_rate) {
  // No initialization needed for emulation mode
}

void uart_tx(uint8_t id, uint8_t data) {
  // uart_send(id, &data, 1);
  // crx_log(data);
} 

void uart_send(uint8_t id, uint8_t *buf, uint8_t len) {
//   USART_TypeDef *uart = uart_ports[id].uart;

//   while (len--) {
// #if WITH_UART_FIFO
//     uart_wait_flag(uart, USART_ISR_TXE_TXFNF, RESET);
// #else
//     uart_wait_flag(uart, UART_FLAG_TC, RESET);
//     uart_wait_flag(uart, UART_FLAG_TXE, RESET);
// #endif
//     // WAIT_FLAG(uart, ISR, USART_ISR_TXE_TXFNF, SET);
//     // while(((uart->ISR & USART_ISR_TXE_TXFNF) == 0));
//     uart->VO_UART_DATAREG = *buf++;
//   }
}

int uart_printf(const char *fmt, ...) {
  char              buf[400];
  __builtin_va_list args;

  va_start(args, fmt);
  vo_vsprintf(buf, fmt, args);
  va_end(args);
  
  puts(buf);
  return 0;
}

#endif // 