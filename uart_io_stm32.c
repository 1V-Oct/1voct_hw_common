#include "uart_io.h"
#include "vo_string.h"
#include <stdarg.h>

#if (WITH_CRASH_HANDLER == 1)
#include "crash_handler.h"
#endif

// #define WITH_UART_FIFO    (1)

#if !defined(UART_USART_ENABLE)
#error Must define UART_USART_ENABLE each bit enables separate UART
#endif

#if !defined(UART_PUTS_PORT_ID)
#error Must define UART_PUTS_PORT_ID id of port to output puts
#endif

#if defined(STM32F103xB)

#define VO_UART_ISR        SR
#define VO_UART_SR_RXNE    USART_SR_RXNE
#define VO_UART_CR1_RXNEIE USART_CR1_RXNEIE
#define VO_UART_RDR        DR
#define VO_UART_DATAREG    DR

#define UART_USART1_PORT   USART1
#define UART_USART2_PORT   USART2
#define UART_USART3_PORT   USART3
// this one can be defined either as constant number or call to selected Clock function
#define VO_UART_CLOCK      HAL_RCC_GetPCLK2Freq()
// #define VO_UART_CLOCK HAL_RCC_GetHCLKFreq()
#elif defined(STM32H750xx)

#define VO_UART_ISR        ISR
#define VO_UART_SR_RXNE    USART_ISR_RXNE
#define VO_UART_CR1_RXNEIE USART_CR1_RXNEIE_RXFNEIE
#define VO_UART_RDR        RDR
#define VO_UART_DATAREG    TDR

#define UART_USART3_PORT   USART3
#define UART_USART5_PORT   UART5
#define UART_USART8_PORT   UART8

// this one can be defined either as constant number or call to selected Clock function
#define VO_UART_CLOCK      HAL_RCC_GetPCLK2Freq()

#else
#error Unknown CPU
#endif

__weak void usart1_rx_cb(uint8_t c) {}
__weak void usart2_rx_cb(uint8_t c) {}
__weak void usart3_rx_cb(uint8_t c) {}
__weak void usart5_rx_cb(uint8_t c) {}
__weak void usart8_rx_cb(uint8_t c) {}

__weak void crx_log(char c) {}

typedef struct {
  USART_TypeDef *uart;
  uart_rx_cb_t   rx_cb;
  // void           rx_cb(uint8_t c);
} uart_t;

const uart_t uart_ports[] = {
#if (UART_USART_ENABLE & 0b00000001)
  {UART_USART1_PORT, usart1_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b00000010)
  {UART_USART2_PORT, usart2_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b00000100)
  {UART_USART3_PORT, usart3_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b00001000)
  {UART_USART4_PORT, usart4_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b00010000)
  {UART_USART5_PORT, usart5_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b00100000)
  {UART_USART6_PORT, usart6_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b01000000)
  {UART_USART7_PORT, usart7_rx_cb},
#endif
#if (UART_USART_ENABLE & 0b10000000)
  {UART_USART8_PORT, usart8_rx_cb},
#endif
};

/**
 * @brief Initialize UART port
 *
 * @param id Port id to configure
 * @param baud_rate Baudrate for port if top bit set of baudrate then parity is even
 */
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

  // BRR is clock divided by baud rates
  uint32_t sysclock   = VO_UART_CLOCK;
  uint32_t brr        = sysclock / (baud_rate & 0x0fffffff);

  uart->CR1           = 0; // reset everything
  uart->BRR           = brr;
#if WITH_UART_FIFO

  if (baud_rate & VO_UART_PARITY_EVEN)
    uart->CR1 |= UART_WORDLENGTH_9B | UART_PARITY_EVEN;

  uart->CR1 |= USART_CR1_FIFOEN;
#endif
  uart->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | VO_UART_CR1_RXNEIE;
}

static void uart_wait_flag(USART_TypeDef *uart, uint32_t flag, uint8_t stat) {
  while ((READ_BIT(uart->VO_UART_ISR, flag) == flag ? SET : RESET) == stat) {
    // while ((READ_BIT(uart->VO_UART_ISR, flag) == flag) == stat) {
  }
}

#define WAIT_FLAG(uart, reg, flag, res)       \
  while (((uart->reg & flag) == flag) == res) \
    ;

// void uart_register_rx_cb(uart_rx_cb_t f) {
//   uart[id].rx_cb = f;
// }

void uart_it(uint8_t id) {
  uint8_t        rx_byte;
  USART_TypeDef *uart = uart_ports[id].uart;
#if defined(STM32F103xB)
  if (uart->VO_UART_ISR & VO_UART_SR_RXNE) {
    rx_byte = uart->DR;
    // uart_rx_cb[id](rx_byte);
    uart_ports[id].rx_cb(rx_byte);
  }
#elif defined(STM32H750xx)
#if 1
  // CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
  // CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
  // uart->CR1 &= ~USART_CR1_PEIE;
  // uart->CR3 &= ~USART_CR3_EIE;
  while (uart->ISR & USART_ISR_RXNE_RXFNE) {
    if (uart->ISR & USART_ISR_ORE) {
      uart->ICR |= USART_ICR_ORECF;
    }
    rx_byte = uart->VO_UART_RDR;
    if (uart_ports[id].rx_cb)
      uart_ports[id].rx_cb(rx_byte);
    // uart_rx_cb[id](rx_byte);
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
#if WITH_UART_FIFO
  uart_wait_flag(uart, USART_ISR_TXE_TXFNF, RESET);
#else
  uart_wait_flag(uart, UART_FLAG_TC, RESET);
  uart_wait_flag(uart, UART_FLAG_TXE, RESET);
#endif
  // WAIT_FLAG(uart, ISR, USART_ISR_TXE_TXFNF, SET);
  // while(((uart->ISR & USART_ISR_TXE_TXFNF) == 0));
  uart->VO_UART_DATAREG = c;
}

void uart_send(uint8_t id, uint8_t *buf, uint8_t len) {
  USART_TypeDef *uart = uart_ports[id].uart;

  while (len--) {
#if WITH_UART_FIFO
    uart_wait_flag(uart, USART_ISR_TXE_TXFNF, RESET);
#else
    uart_wait_flag(uart, UART_FLAG_TC, RESET);
    uart_wait_flag(uart, UART_FLAG_TXE, RESET);
#endif
    // WAIT_FLAG(uart, ISR, USART_ISR_TXE_TXFNF, SET);
    // while(((uart->ISR & USART_ISR_TXE_TXFNF) == 0));
    uart->VO_UART_DATAREG = *buf++;
  }
}
static const char *null_str = "<null>";

const char *uart_puts(const char *buf) {
  const char *p = buf;

  if (buf == NULL)
    p = null_str;
  char c;
  while ((c = *p++) != 0) {
    uart_tx(UART_PUTS_PORT_ID, c);
#if (WITH_CRASH_HANDLER == 1)
    crx_log(c);
#endif
  };
  return p;
}

int uart_printf(const char *fmt, ...) {
  char              buf[400];
  __builtin_va_list args;

  va_start(args, fmt);
  vo_vsprintf(buf, fmt, args);
  va_end(args);

  return uart_puts(buf) - buf;
}
