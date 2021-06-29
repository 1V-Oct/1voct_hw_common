#include "uart_io.h"
#include "vo_string.h"
#include <stdarg.h>
#include "vo_logger.h"

static uint8_t rx_byte;

__weak void usart1_rx_cb(uint8_t c) {}
__weak void usart2_rx_cb(uint8_t c) {}
__weak void usart3_rx_cb(uint8_t c) {}

typedef struct {
  USART_TypeDef *uart;
  // void           rx_cb(uint8_t c);
} uart_t;


const uart_rx_cb_t uart_rx_cb[3] = {
  usart1_rx_cb,
  usart2_rx_cb,
  usart3_rx_cb,
};

const uart_t uart_ports[3] = {
  { USART1 },
  { USART2 },
  { USART3 },
};


#define VO_UART_IRQ USART1_IRQn


#if defined(STM32F103xB)

#define VO_UART_ISR     SR
#define VO_UART_SR_RXNE USART_SR_RXNE
#define VO_UART_CR1_RXNEIE USART_CR1_RXNEIE
#define VO_UART_RDR     DR
#define VO_UART_DATAREG DR


// this one can be defined either as constant number or call to selected Clock function
#define VO_UART_CLOCK HAL_RCC_GetPCLK2Freq()
// #define VO_UART_CLOCK HAL_RCC_GetHCLKFreq()
#elif defined(STM32H750xx)

#define VO_UART_ISR     ISR
#define VO_UART_SR_RXNE USART_ISR_RXNE
#define VO_UART_CR1_RXNEIE USART_CR1_RXNEIE_RXFNEIE
#define VO_UART_RDR     RDR
#define VO_UART_DATAREG TDR

// this one can be defined either as constant number or call to selected Clock function
#define VO_UART_CLOCK   HAL_RCC_GetPCLK2Freq()

#else
#error Unknown CPU
#endif

// #define VO_UART USART1
// #define VO_UART_GPIO_TX_PORT GPIOA
// #define VO_UART_SR SR
// #define VO_UART_SR_RXNE USART_ISR_RXNE
// #define VO_UART_RDR DR
// #define VO_UART_DATAREG DR
// #define VO_UART_SR SR

// #define VO_UART_SR_RXNE     USART_ISR_RXNE_RXFNE_Msk

// static void uart_default_rx(uint8_t b) {
//   UNUSED(b);
// }


void uart_init(uint8_t id, uint32_t baud_rate) {

  // make sure the relevant pins are appropriately set up.
  /*
  RCC_APB2ENR |= RCC_APB2ENR_IOPAEN;              // enable clock for GPIOA
  GPIOA_CRH   |= (0x0BUL  < < 4);                  // Tx (PA9) alt. out push-pull
  GPIOA_CRH   |= (0x04UL  << 8);                  // Rx (PA10) in floating
  RCC_APB2ENR |= RCC_APB2ENR_USART1EN;            // enable clock for USART1
  USART1_BRR  = 64000000L/115200L;                // set baudrate
  USART1_CR1 |= (USART1_CR1_RE | USART1_CR1_TE);  // RX, TX enable
  USART1_CR1 |= USART1_CR1_UE;                    // USART enable
*/

  // UART->CR1 = 0;
  // UART->BRR = 256u * UART_CLOCK / baud; //reference manual 39.4.4
  // UART->CR1 = USART_CR1_RXNEIE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;
  // MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg); // 12
  // MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits); // 0
  // MODIFY_REG(huart->Instance->PRESC, USART_PRESC_PRESCALER, huart->Init.ClockPrescaler); // 0

  // uart_rx_cb = uart_default_rx;

  /* UART8 interrupt Init */

  // interrupt moved to system_board.c
// #if defined(VO_UART_IRQ)
//   HAL_NVIC_SetPriority(VO_UART_IRQ, 0, 0);
//   HAL_NVIC_EnableIRQ(VO_UART_IRQ);
// #endif
  // HAL_UART_MspInit(UART8);

  // case UART_CLOCKSOURCE_D2PCLK1:
  //   pclk = HAL_RCC_GetPCLK1Freq();
  //   pclk = 110000000

  USART_TypeDef *uart = uart_ports[id].uart;

  // BRR is clock divided by baud rate
  uint32_t sysclock = VO_UART_CLOCK;
  uint32_t brr      = sysclock / baud_rate;

  uart->CR1 = 0;   //reset everything
  uart->BRR = brr;
  uart->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | VO_UART_CR1_RXNEIE;
}

static void uart_wait_flag(USART_TypeDef *uart, uint32_t flag, uint8_t stat) {
  while ((READ_BIT(uart->VO_UART_ISR, flag) == flag ? SET : RESET) == stat) {
  }
}

// void uart_register_rx_cb(uart_rx_cb_t f) {
//   uart[id].rx_cb = f;
// }

void uart_it(uint8_t id) {
  USART_TypeDef *uart = uart_ports[id].uart;
#if defined(STM32F103xB)
  if (uart->VO_UART_ISR & VO_UART_SR_RXNE) {
    rx_byte = uart->DR;
    uart_rx_cb[id](rx_byte);
  }
#elif defined(STM32H750xx)
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
#else
#error Unknown CPU
#endif
}

void uart_tx(uint8_t id, uint8_t c) {
  USART_TypeDef *uart = uart_ports[id].uart;
  uart_wait_flag(uart, UART_FLAG_TC, RESET);
  uart_wait_flag(uart, UART_FLAG_TXE, RESET);
  uart->VO_UART_DATAREG = c;
}

static const char *null_str = "<null>";

void uart_puts(const char *buf) {
  const uint8_t *p = (const uint8_t *)buf;

  if (buf == NULL)
    p = (const uint8_t *)null_str;

  while (*p) {
    uart_tx(0, *p++);
  };
}

void uart_printf(const char *fmt, ...) {
  char              buf[400];
  __builtin_va_list args;

  va_start(args, fmt);
  vo_vsprintf(buf, fmt, args);
  va_end(args);
  uart_puts(buf);
}
