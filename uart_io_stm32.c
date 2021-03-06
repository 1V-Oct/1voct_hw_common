#include "uart_io.h"
#include <stdarg.h>
#include "vo_string.h"


static uint8_t rx_byte;

uart_rx_cb_t uart_rx_cb;

#define VO_UART                 USART1
#define VO_UART_GPIO_TX_PORT    GPIOA
#define VO_UART_SR              SR
#define VO_UART_SR_RXNE         USART_SR_RXNE

// #define VO_UART_SR_RXNE     USART_ISR_RXNE_RXFNE_Msk

void uart_init(void) {
  // UART->CR1 = 0; //reset everything
  // UART->BRR = 256u * UART_CLOCK / baud; //reference manual 39.4.4
  // UART->CR1 = USART_CR1_RXNEIE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;


  // MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg); // 12
  
  // MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits); // 0

  // MODIFY_REG(huart->Instance->PRESC, USART_PRESC_PRESCALER, huart->Init.ClockPrescaler); // 0
  // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // __HAL_RCC_VO_UART_CLK_ENABLE();
    
    // __HAL_RCC_USART1_CLK_ENABLE();

    // __HAL_RCC_GPIOA_CLK_ENABLE();
    /**VO_UART GPIO Configuration
    PE0     ------> VO_UART_RX
    PE1     ------> VO_UART_TX
    */
    // GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    // GPIO_InitStruct.Alternate = GPIO_AF8_VO_UART;
    // HAL_GPIO_Init(VO_UART_GPIO_TX_PORT, &GPIO_InitStruct);

    /* VO_UART interrupt Init */
    // HAL_NVIC_SetPriority(VO_UART_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(VO_UART_IRQn);




  // HAL_UART_MspInit(VO_UART);

    // case UART_CLOCKSOURCE_D2PCLK1:
    uint32_t pclk = HAL_RCC_GetHCLKFreq();
    // HAL_RCC_Get
    // pclk = 12000000;
    uint32_t baud = 115200;
    uint32_t baud_brr =((unsigned int)(pclk/(16*baud))) << 4;
    // uint32_t baud_brr = pclk / baud;
  VO_UART->CR1 = 0;

  // BRR is clock divided by baud rate
  VO_UART->BRR = baud_brr;
  // VO_UART->CR1 |= USART_CR1_RXNEIE_RXFNEIE_Msk | USART_CR1_TE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;
  VO_UART->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

  //HAL_UART_Receive_IT(&hVO_UART, &rx_byte, 1);
  // = USART_CR1_RXNEIE_Msk | USART_CR1_TE_Msk | USART_CR1_TXEIE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;

} 

void uart_register_rx_cb(uart_rx_cb_t f) {
  uart_rx_cb = f;
}


// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (huart->Instance == VO_UART)
//   {
//     uart_rx_cb(rx_byte);
//     /* Transmit one byte with 100 ms timeout */
//     // HAL_UART_Transmit(&hVO_UART, &rx_byte, 1, 100);

//     /* Receive one byte in interrupt mode */ 
//     HAL_UART_Receive_IT(&hVO_UART, &rx_byte, 1);
//   }
// }


void uart_wait_flag(uint32_t flag, uint8_t stat) {
  while ((READ_BIT(VO_UART->VO_UART_SR, flag) == flag ? SET : RESET) == stat) {}
}

void uart_it(void) {
  if (VO_UART->VO_UART_SR & VO_UART_SR_RXNE) {
    rx_byte = VO_UART->DR;
    if (uart_rx_cb) uart_rx_cb(rx_byte);
  }
}

void uart_tx(uint8_t t) {
  uart_wait_flag(UART_FLAG_TC, RESET);
  uart_wait_flag(UART_FLAG_TXE, RESET);
#if defined(STM32F103xB)
  VO_UART->DR = t;
#else
  #error Unknown architecture
#endif
}

static const char *null_str = "<null>";
void uart_puts(const char *buf) {
  const uint8_t *p = (const uint8_t *)buf;

  if (buf == NULL) p = (const uint8_t *)null_str;


  while (*p) {
    uart_tx(*p++);
  };
}


void uart_printf(const char* fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vo_vsprintf(buf, fmt, args);
  va_end(args);
  uart_puts(buf);
#if 0  
  va_list args;
  va_start(args, fmt);
  char buf[20];

  const char *t; 
  char tx;
  uint8_t in_special;
  int val;
  // float valf;
  uint8_t leading_zero;
  uint8_t places;

  t = fmt;
  places = 0;
  leading_zero = 0;
  in_special = 0;
  while((tx = *t++) != 0) {
    if (in_special) {
      switch(tx) {
        case '0':
          leading_zero = 1;
          break;
        case 'c':
          val = va_arg(args, int);
          uart_tx(val);
          in_special = 0;
          break;
        case 'd':
          val = va_arg(args, int);
          vo_itoa(val, buf, 10, places, leading_zero ? '0' : ' ');
          uart_puts(buf);
          in_special = 0;
          break;
        case 'x':
          val = va_arg(args, int);
          vo_itoa(val, buf, 16, places, leading_zero ? '0' : ' ');
          uart_puts(buf);
          in_special = 0;
          break;
        case 'f':
          break;
        case 's':
          uart_puts(va_arg(args, char *));
          in_special = 0;
          break;
        default:
          while(tx >= '0' && tx <= '9') {
            places = places * 10 + tx - '0';
            tx = *t;
            if (tx >= '0' && tx <= '9') t++;
          }
          break;
        
      }
    } else {
      if (tx == '%') {
        in_special = 1;
        places = 0;
        leading_zero = 0;
      } else { 
        uart_tx(tx);
      }
    }
  }
  //vsnprintf(buf, sizeof(buf), fmt, args);
  //uart_puts(buf);
  va_end(args);
#endif
}
