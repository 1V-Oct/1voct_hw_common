#include "uart_io.h"
#include "vo_string.h"
#include <stdarg.h>

static uint8_t rx_byte;

uart_rx_cb_t uart_rx_cb;

// #define VO_UART USART1
// #define VO_UART_GPIO_TX_PORT GPIOA
// #define VO_UART_SR SR
// #define VO_UART_SR_RXNE USART_ISR_RXNE
// #define VO_UART_RDR DR
// #define VO_UART_DATAREG DR
// #define VO_UART_SR SR

// #define VO_UART_SR_RXNE     USART_ISR_RXNE_RXFNE_Msk

static void uart_default_rx(uint8_t b) {
  UNUSED(b);
}

void uart_init(void) {
  // UART->CR1 = 0; //reset everything
  // UART->BRR = 256u * UART_CLOCK / baud; //reference manual 39.4.4
  // UART->CR1 = USART_CR1_RXNEIE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;
  // MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg); // 12
  // MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits); // 0
  // MODIFY_REG(huart->Instance->PRESC, USART_PRESC_PRESCALER, huart->Init.ClockPrescaler); // 0

  uart_rx_cb = uart_default_rx;

  /* UART8 interrupt Init */
#if defined(VO_UART_IRQ)
  HAL_NVIC_SetPriority(VO_UART_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(VO_UART_IRQ);
#endif
  // HAL_UART_MspInit(UART8);

  // case UART_CLOCKSOURCE_D2PCLK1:
  //   pclk = HAL_RCC_GetPCLK1Freq();
  //   pclk = 110000000

  uint32_t sysclock = VO_UART_CLOCK;
  uint32_t brr      = sysclock / VO_UART_BAUD_RATE;

  // BRR is clock divided by baud rate
  VO_UART->CR1 = 0;
  VO_UART->BRR = brr; //110000000 / baud_rate;
  // VO_UART->CR1 |= USART_CR1_RXNEIE_RXFNEIE_Msk | USART_CR1_TE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;
  VO_UART->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
#if defined(VO_UART_IRQ)
  VO_UART->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
#endif
  //HAL_UART_Receive_IT(&huart8, &rx_byte, 1);
  // = USART_CR1_RXNEIE_Msk | USART_CR1_TE_Msk | USART_CR1_TXEIE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;
}

void uart_register_rx_cb(uart_rx_cb_t f) {
  uart_rx_cb = f;
}

void uart_wait_flag(uint32_t flag, uint8_t stat) {
  while ((READ_BIT(VO_UART->VO_UART_ISR, flag) == flag ? SET : RESET) == stat) {
  }
}

void uart_it(void) {
#if 1
  while (VO_UART->ISR & USART_ISR_RXNE_RXFNE) {
    if (VO_UART->ISR & USART_ISR_ORE) {
      VO_UART->ICR |= USART_ICR_ORECF;
    }
    rx_byte = VO_UART->VO_UART_RDR;
    uart_rx_cb(rx_byte);
  }
#else
  if (VO_UART->ISR & USART_ISR_ORE) {
    VO_UART->ICR |= USART_ICR_ORECF;
    rx_byte = VO_UART->RDR;
    if (uart_rx_cb)
      uart_rx_cb(rx_byte);
  }
  if (VO_UART->ISR & USART_ISR_RXNE_RXFNE) {
    rx_byte = VO_UART->RDR;
    if (uart_rx_cb)
      uart_rx_cb(rx_byte);
  }
#endif
}

void uart_tx(uint8_t t) {
  uart_wait_flag(UART_FLAG_TC, RESET);
  uart_wait_flag(UART_FLAG_TXE, RESET);
  VO_UART->VO_UART_DATAREG = t;
}

static const char *null_str = "<null>";
void               uart_puts(const char *buf) {
  const uint8_t *p = (const uint8_t *)buf;

  if (buf == NULL)
    p = (const uint8_t *)null_str;

  while (*p) {
    uart_tx(*p++);
  };
}

void uart_printf(const char *fmt, ...) {
  char    buf[400];
  __builtin_va_list args;
  va_start(args, fmt);
  vo_vsprintf(buf, fmt, args);
  va_end(args);
  uart_puts(buf);
}
