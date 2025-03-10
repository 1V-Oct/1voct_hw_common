#include "wm2812_control.h"

#if WITH_WM2812_CONTROL
#include "swenc.h"
#include "vo_global_timer.h"
#include "vo_logger.h"
#include "vo_memory.h"
#include "vo_string.h"
#include "vo_time.h"

#define BLC_DEBUG      (1)

#define WM2812_USE_PWM_DMA (0)


// 800kHz TIM3, This gives a PWM signal of 1.25us = 1250ns
#define PRESCALER      0
#define AUTORELOAD     300

// #define ONE        200                // This gives a HIGH signal of 950ns (and stays 300ns LOW)
// #define ZERO       (AUTORELOAD - 200) // This gives a HIGH signal of 350ns (and stays 900ns LOW)
#define BREAK          0 // This stays 1250ns LOW

// #define ONE        35
// #define ZERO       90
#define ONE            50
#define ZERO           100

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch4;

void usdelay(uint32_t us) {
  uint32_t cur_cyc;
  us *= 4;
  DWT->CYCCNT = 0;
  while (DWT->CYCCNT < us) {
    __NOP();
  }
}

uint8_t pol = 1;
#define CYCLELED (90 + 35)
static void send_led(uint8_t duty) {
  // uint8_t duty    = val ? 3 : 7;
  uint8_t no_duty = CYCLELED - duty;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
  // HAL_Delay(duty);
  usdelay(duty);
  // usdelay();
  if (duty > 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // HAL_Delay(no_duty);
    usdelay(no_duty);
    // usdelay(1);
    // HAL_Delay(no_duty);
  }
}

void send_individual_led(uint8_t rgb) {
  int     j;
  uint8_t col = rgb & 0xff;
  col >>= 3;
  for (j = 0; j < 8; j++) {
    send_led((col & (1 << (7 - j))) ? ONE : ZERO);
  }
}
void send_rgb_led(uint32_t rgb) {
  send_individual_led(((rgb >> 8) & 0xff) >> 2);
  send_individual_led(((rgb >> 16) & 0xff) >> 2);
  send_individual_led((rgb & 0xff));
}

void send_end(void) {
  int i;
  for (i = 0; i < 3; i++) {
    send_led(BREAK);
  }
}

// #if WITH_LCD_BACKLIGHT_CONTROL
// // TIM_HandleTypeDef htim3;
// static void       MX_TIM3_Init(void);
// static void       MX_TIM4_Init(void);
// #endif

static uint32_t wm2812_timer;

static uint32_t wm2812_status_colors[2][3];

void wm2812_set_status_color(uint8_t idx, uint32_t rgb, uint32_t rgb_alt) {
  wm2812_status_colors[0][idx & 0x3] = rgb;

  if (rgb_alt == 0xffffffff)
    rgb_alt = rgb;

  wm2812_status_colors[1][idx & 0x3] = rgb_alt;
}




typedef struct
{
  __IO uint32_t ISR; /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR; /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

static void DMA_SetConfiguration(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) {
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs_dma = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  if (IS_DMA_DMAMUX_ALL_INSTANCE(hdma->Instance) != 0U) /* No DMAMUX available for BDMA1 */
  {
    /* Clear the DMAMUX synchro overrun flag */
    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

    if (hdma->DMAmuxRequestGen != 0U) {
      /* Clear the DMAMUX request generator overrun flag */
      hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }
  }

  /* Clear all interrupt flags at correct offset within the register */
  regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);

  /* Clear DBM bit */
  ((DMA_Stream_TypeDef *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  ((DMA_Stream_TypeDef *)hdma->Instance)->NDTR = DataLength;

  /* Peripheral to Memory */
  if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH) {
    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef *)hdma->Instance)->PAR  = DstAddress;

    /* Configure DMA Stream source address */
    ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = SrcAddress;
  }
  /* Memory to Peripheral */
  else {
    /* Configure DMA Stream source address */
    ((DMA_Stream_TypeDef *)hdma->Instance)->PAR  = SrcAddress;

    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = DstAddress;
  }
}

static void TransferComplete(DMA_HandleTypeDef *DmaHandle) {
  LOGE("XXX");
}

static void TransferError(DMA_HandleTypeDef *DmaHandle) {
  LOGE("XXX");
}


#if 0



void WS2812B_sendBuffer( void )
{
   // wait until last buffer transmission has been completed
  //  while( WS2812_State != WS2812B_READY );
  
   // transmission complete flag, indicate that transmission is taking place
  //  WS2812_State = WS2812B_BUSY;
   
   // set period to 1.25 us with the auto reload register
   TIM3->ARR = 29u;
   
   // set configuration
  //  DMA_SetConfiguration(&DMA_HandleStruct_UEV, (uint32_t)&WS2812_High, (uint32_t)&GPIOA->ODR, GPIO_BUFFERSIZE);
   DMA_SetConfiguration(&hdma_tim3_ch4, (uint32_t)&WS2812_Buffer[0], (uint32_t)&GPIOA->ODR, GPIO_BUFFERSIZE);
  //  DMA_SetConfiguration(&DMA_HandleStruct_CC2, (uint32_t)&WS2812_Low, (uint32_t)&GPIOA->ODR, GPIO_BUFFERSIZE);
   
   // clear all relevant DMA flags from the channels 0, 1 and 2
   __HAL_DMA_CLEAR_FLAG(&DMA_HandleStruct_UEV, DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TEIF0_4 );
   __HAL_DMA_CLEAR_FLAG(&DMA_HandleStruct_CC1, DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5 );
   __HAL_DMA_CLEAR_FLAG(&DMA_HandleStruct_CC2, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 );

   // Enable the selected DMA transfer interrupts
   __HAL_DMA_ENABLE_IT(&DMA_HandleStruct_CC2, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
   
   // enable dma channels
   __HAL_DMA_ENABLE(&DMA_HandleStruct_UEV);
   __HAL_DMA_ENABLE(&DMA_HandleStruct_CC1);
   __HAL_DMA_ENABLE(&DMA_HandleStruct_CC2);
   
   // clear all TIM2 flags
   TIM2->SR = 0;
   
   // IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
   __HAL_TIM_ENABLE_DMA(&TIM2_Handle, TIM_DMA_UPDATE);
   __HAL_TIM_ENABLE_DMA(&TIM2_Handle, TIM_DMA_CC1);
   __HAL_TIM_ENABLE_DMA(&TIM2_Handle, TIM_DMA_CC2);
   
   // Enable the Output compare channel
   TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_ENABLE);
   TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_ENABLE);
   
   // preload counter with 29 so TIM2 generates UEV directly to start DMA transfer
   __HAL_TIM_SET_COUNTER(&TIM2_Handle, 29);
   
   // start TIM2
   __HAL_TIM_ENABLE(&TIM2_Handle);
}
#endif

uint32_t fData[] = {
  // LED 1 - GREEN=255, REF = 0 , BLUE = 0
  // BIT7 ....... BIT0, BIT7......BIT0, BIT7.......BIT0
  ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // ZERO, ZERO, ZERO, ZERO, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // LED 2 - GREEN=0, RED = 255 , BLUE = 0
  // ZERO, ZERO, ZERO, ZERO, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // LED 3 - GREEN=0, RED = 0 , BLUE = 255
  // ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE,
  ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // LED 4 - GREEN=0, RED = 255 , BLUE = 255
  // ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
  // DUMMY LED FOR BREAK SIGNAL!
  BREAK, BREAK, BREAK, BREAK, BREAK, BREAK, BREAK, BREAK};

static uint8_t wm2812_bank = 0;
void           wm2812_update_buf(void) {
  static uint32_t counter = 0;
  if (!vo_gt_check_and_rearm(&wm2812_timer, 500)) {
    return;
  }
  // LOGI("TIMER FIRED*");
  wm2812_bank ^= 1;
  // uint32_t time_elapsed = vo_time_get_ms();
  send_rgb_led(wm2812_status_colors[wm2812_bank][0]);
  send_rgb_led(wm2812_status_colors[wm2812_bank][1]);
  send_rgb_led(wm2812_status_colors[wm2812_bank][2]);
  // send_rgb_led(COL_RED);
  // send_rgb_led(COL_RED);
  // send_rgb_led(COL_RED);
  send_end();
  // time_elapsed = vo_time_get_ms() - time_elapsed;
  // LOGI("time consumed: %d", time_elapsed);
  return;

  uint32_t i;
  // while(1)
  // send_led(50);
  // for (i = 0; i < VO_NUM_ITEMS(fData); i++) {
  // send_led(fData[i]);
  // }

  // __HAL_TIM_SET_PRESCALER(&htim3, PRESCALER);
  // __HAL_TIM_SET_AUTORELOAD(&htim3, AUTORELOAD);
  // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 30);

  // 4 LED WS2812 strip. Each LED has 3 values G,R,B , each 8 bit. So each LED has 24Bit. The last "Dummy" LED is the break signal (8bit).

  // Naive PWM signal set.

  // __HAL_DMA_CLEAR_FLAG(&hdma_tim3_ch4, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 );

  // Enable the selected DMA transfer interrupts
  // __HAL_DMA_ENABLE_IT(&hdma_tim3_ch4, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

  // enable dma channels
  // __HAL_DMA_ENABLE(&hdma_tim3_ch4);

  // clear all TIM2 flags
  // TIM2->SR = 0;
  // int i;
  for (i = 0; i < 3; i++) {
    fData[0] = fData[1];
  }
  // IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
  // __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
  // __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC1);

  // TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_4, TIM_CCx_ENABLE);
  // Send the fData out on GPIO PB1 (default TIM3->Channel4->PWM output!

  // while (1) {
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
  HAL_StatusTypeDef res;
  if ((res = HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, fData, sizeof(fData) / sizeof(uint32_t))) != HAL_OK) {
    LOGE("ERR %d", res);
  }
  HAL_Delay(10);
  // }
  // DMA_SetConfiguration(&hdma_tim3_ch4, (uint32_t)&fData[0], (uint32_t)&GPIOB->ODR, 24u * 4);

  // HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, fData, sizeof(fData) / sizeof(uint32_t));
#if 0
    /* Calculate fading efect */
    fade_value += fade_step;
    if (fade_value > 0xFF) {
      fade_value = 0xFF;
      fade_step  = -fade_step;
    } else if (fade_value < 0) {
      fade_value = 0;
      fade_step  = -fade_step;
    }

    /* Check if we need to change the color */
    if (fade_value == 0) {
      for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = color_counter & 0x01 ? 0xFF : 0x00;
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = color_counter & 0x02 ? 0xFF : 0x00;
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = color_counter & 0x04 ? 0xFF : 0x00;
      }
      color_counter++;
    }

    /* Calculate new brightness */
    brightness = (uint8_t)(quad_calc((float)fade_value / (float)0xFF) * (float)0x3F);

    /* Start data transfer in non-blocking mode */
    led_start_transfer();
#endif
}

extern "C" void DMA1_Stream4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_tim3_ch4);
  // __HAL_TIM_DISABLE(&htim3);
  // __HAL_TIM_SET_PRESCALER(&htim3, 0);
  // __HAL_TIM_SET_AUTORELOAD(&htim3, 0);
  LOGI("$");
}

void wm2812_process(void) {
  // LOGI("LOOP");
  wm2812_update_buf();
  //   extern TIM_HandleTypeDef htim4;
  //   // __HAL_TIM_SET_AUTORELOAD(&htim4, 100); // 27khz
  //   // __HAL_TIM_SET_PRESCALER(&htim4, 1);
  //   // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);

  // #if (WITH_LCD_BACKLIGHT_CONTROL)
  static uint16_t          last_timer = 0xffff;
  extern TIM_HandleTypeDef htim4;
  // timer = adc_pots_get_value_u16(0);
  // timer = 0x1f1f;
  // timer &= 0xfff0;

#if 0
    static int16_t           timer;
    timer += sw_enc_get_val(ENC_3);
    if (timer < 0)
      timer = 0;
    if (timer > 20)
      timer = 20;
#else

  // int16_t timer = sys_set_exp[SYS_SET_BACKLIGHT];
  int16_t timer = 5; // sys_set_exp[SYS_SET_BACKLIGHT];

#endif
  if (timer != last_timer) {
    last_timer = timer;

#if BLC_DEBUG
    LOGI("%04x", timer);
#endif
#if 1
    timer += 6;
    uint16_t new_timer = timer * timer;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, new_timer);
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, new_timer);
#else
    // CCRx, x is the channel we change
    // htim4.Instance->CCR4 = timer;
    // LOGI("%04x", timer);

    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = timer;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {

      Error_Handler();
    }
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
#endif
  }
}

#if 0

/**
* @brief TIM_PWM MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_pwm: TIM_PWM handle pointer
* @retval None
*/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(htim_pwm->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    // __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

  /* USER CODE BEGIN TIM3_MspInit 1 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PD14     ------> TIM4_CH3
    PD15     ------> TIM4_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE END TIM3_MspInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
    // __HAL_RCC_GPIOB_CLK_ENABLE();
    // /**TIM3 GPIO Configuration
    // PB5     ------> TIM3_CH2
    // */
    // GPIO_InitStruct.Pin = GPIO_PIN_5;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(htim->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    // __HAL_RCC_GPIOD_CLK_ENABLE();
    // /**TIM4 GPIO Configuration
    // PD14     ------> TIM4_CH3
    // PD15     ------> TIM4_CH4
    // */
    // GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }

}
/**
* @brief TIM_PWM MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_pwm: TIM_PWM handle pointer
* @retval None
*/
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }

}

#endif

// #if WITH_LCD_BACKLIGHT_CONTROL

void wm2812_init(void) {
  vo_gt_start(&wm2812_timer);
  // TIM_MasterConfigTypeDef sMasterConfig;
  // TIM_OC_InitTypeDef      sConfigOC;

  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin   = GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  return;
#if 0
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB1     ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin       = GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  // TIM4 GPIO Configuration
  // PD14     ------> TIM4_CH3
  GPIO_InitStruct.Pin       = GPIO_PIN_14;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  TIM_ClockConfigTypeDef         sClockSourceConfig;
  TIM_MasterConfigTypeDef        sMasterConfig;
  TIM_OC_InitTypeDef             sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  vo_memset(&sBreakDeadTimeConfig, 0, sizeof(sBreakDeadTimeConfig));
  vo_memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
  vo_memset(&sMasterConfig, 0, sizeof(sMasterConfig));
  vo_memset(&sConfigOC, 0, sizeof(sConfigOC));

#endif
/* USER CODE BEGIN TIM3_Init 1 */
#if 0
  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 3000;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 1500;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  // __HAL_TIM_SET_COUNTER(&htim3, 29);
  // if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
  //   /* PWM generation Error */
  //   Error_Handler();
  // }

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim3, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
#endif
#if 0
  __HAL_RCC_TIM3_CLK_ENABLE();

  // uint16_t uhPrescalerValue    = (uint32_t)(SystemCoreClock / (2 * 20000000)) - 1;

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 300;
  // htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // __HAL_TIM_SET_COUNTER(&htim3, 29);

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {

    Error_Handler();
  }

#endif
  // __HAL_TIM_SET_COUNTER(&htim3, 29);
  // if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
  //   /* PWM generation Error */
  //   Error_Handler();
  // }
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

#if 0

  hdma_tim3_ch4.Instance                 = DMA1_Stream4;
  hdma_tim3_ch4.Init.Request             = DMA_REQUEST_TIM3_CH4;
  hdma_tim3_ch4.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tim3_ch4.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tim3_ch4.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim3_ch4.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_tim3_ch4.Init.Mode                = DMA_CIRCULAR;
  // hdma_tim3_ch4.Init.Mode                = DMA_NORMAL;
  hdma_tim3_ch4.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tim3_ch4.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim3_ch4) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC4], hdma_tim3_ch4);
  // __HAL_DMA_ENABLE(&hdma_tim3_ch4);
  // HAL_DMA_Enable(hdma_tim3_ch4);

  // HAL_DMA_RegisterCallback(&hdma_tim3_ch4, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
  // HAL_DMA_RegisterCallback(&hdma_tim3_ch4, HAL_DMA_XFER_ERROR_CB_ID, TransferError);

  // htim3.State = HAL_TIM_STATE_BUSY;

  // /* Set the Time Base configuration */
  // TIM_Base_SetConfig(htim3.Instance, &htim3.Init);

  // /* Initialize the DMA burst operation state */
  // htim3.DMABurstState = HAL_DMA_BURST_STATE_READY;

  // /* Initialize the TIM channels state */
  // TIM_CHANNEL_STATE_SET_ALL(&htim3, HAL_TIM_CHANNEL_STATE_READY);
  // TIM_CHANNEL_N_STATE_SET_ALL(&htim3, HAL_TIM_CHANNEL_STATE_READY);

  // /* Initialize the TIM state*/
  // htim3.State = HAL_TIM_STATE_READY;

  // NVIC configuration for DMA transfer complete interrupt
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 1);

  // Enable interrupt
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

#endif
#if 0
  __HAL_RCC_TIM4_CLK_ENABLE();

  uint16_t uhPrescalerValue    = (uint32_t)(SystemCoreClock / (2 * 20000000)) - 1;

  htim4.Instance               = TIM4;
  htim4.Init.Prescaler         = 40;
  // htim4.Init.Prescaler         = 1;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.Period            = 400;
  // htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV4;
  htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  htim4.Init.RepetitionCounter = 65535;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 200;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {

    Error_Handler();
  }

  // if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) {
  //   /* PWM generation Error */
  //   Error_Handler();
  // }
#endif

  // HAL_TIM_MspPostInit(&htim3);
}
#if 0
/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  ZERO_STRUCT(sConfigOC);
  ZERO_STRUCT(sClockSourceConfig);
  ZERO_STRUCT(sMasterConfig);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = 0;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}
#endif

// #endif

// #endif

#if 0


// ****************************************************************************
/// \file      ws2812b.c
///
/// \brief     WS2812B C Source File
///
/// \details   Driver Module for WS2812B leds.
///
/// \author    Nico Korn
///
/// \version   1.0.0.0
///
/// \date      24032021
/// 
/// \copyright Copyright (c) 2021 Nico Korn
/// 
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///
/// \pre       
///
/// \bug       
///
/// \warning   
///
/// \todo      
///
// ****************************************************************************

// Include ********************************************************************
#include "ws2812b.h"
#include <string.h>

// Private define *************************************************************
/* WS2812 GPIO output buffer size */
#define GPIO_BUFFERSIZE ROW *COL * 24u // see ROW as LED stripe number and COL LED pixel number on the stripe

// Private types     **********************************************************
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

// Private variables **********************************************************
#pragma location = 0x24030000
static       uint32_t               WS2812_High  = 0xFFFFFFFF;
#pragma location = 0x24030060
static       uint32_t               WS2812_Low   = 0x00000000;
#pragma location = 0x24030200
static       uint16_t               WS2812_Buffer[GPIO_BUFFERSIZE];      // ROW * COL * 24 bits (R(8bit), G(8bit), B(8bit)) = y --- output array transferred to GPIO output --- 1 array entry contents 16 bits parallel to GPIO outp

static       WS2812B_StatusTypeDef  WS2812_State = WS2812B_RESET;
static       TIM_HandleTypeDef      TIM2_Handle;
static       DMA_HandleTypeDef      DMA_HandleStruct_UEV;
static       DMA_HandleTypeDef      DMA_HandleStruct_CC1;
static       DMA_HandleTypeDef      DMA_HandleStruct_CC2;

// Private function prototypes ************************************************
static WS2812B_StatusTypeDef        init_timer              ( void );
static WS2812B_StatusTypeDef        init_dma                ( void );
static WS2812B_StatusTypeDef        init_gpio               ( void );
static void                         DMA_SetConfiguration    ( DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength );
static void                         TransferComplete        ( DMA_HandleTypeDef *DmaHandle );
static void                         TransferError           ( DMA_HandleTypeDef *DmaHandle );
static void                         WS2812_TIM2_callback    ( void );

// Global variables ***********************************************************

// Functions ******************************************************************
// ----------------------------------------------------------------------------
/// \brief     Initialisation of the periphherals for using the ws2812b leds.
///
/// \param     none
///
/// \return    none
WS2812B_StatusTypeDef WS2812B_init( void )
{   
   memset( WS2812_Buffer, 0x00, GPIO_BUFFERSIZE*2 );
   
   // init peripherals
   if( init_gpio() != WS2812B_OK )
   {
     WS2812_State = WS2812B_ERROR;
     return WS2812_State;
   }
   
   if( init_dma() != WS2812B_OK )
   {
     WS2812_State = WS2812B_ERROR;
     return WS2812_State;
   }
   
   if( init_timer() != WS2812B_OK )
   {
     WS2812_State = WS2812B_ERROR;
     return WS2812_State;
   }
   
   // set the ws2812b state flag to ready for operation
   WS2812_State = WS2812B_READY;
   
   return WS2812_State;
}

// ----------------------------------------------------------------------------
/// \brief     Initialisation of the Timer.
///
/// \param     none
///
/// \return    none
static WS2812B_StatusTypeDef init_timer( void )
{
   TIM_OC_InitTypeDef           TIM_OC1Struct;          // cc1
   TIM_OC_InitTypeDef           TIM_OC2Struct;          // cc2
   uint16_t                     PrescalerValue;
   
   // TIM2 Periph clock enable
   __HAL_RCC_TIM2_CLK_ENABLE();
   
   // set prescaler to get a 24 MHz clock signal
   // SystemCoreClock divided by 2! Note the rcc configuration.
   PrescalerValue = (uint16_t) ( (SystemCoreClock/2) / 24000000) - 1;
   
   // Time base configuration
   TIM2_Handle.Instance                 = TIM2;
   TIM2_Handle.Init.Period              = 29;                // set the period to get 29 to get a 800kHz timer -> T=1250 ns, NOTE: the ARR will be set for data transmission and also set for the deadtime/reset timer, so the arr value changes 2 time per complete led write attempt
   TIM2_Handle.Init.Prescaler           = PrescalerValue;
   TIM2_Handle.Init.ClockDivision       = 0;
   TIM2_Handle.Init.CounterMode         = TIM_COUNTERMODE_UP;
   if( HAL_TIM_Base_Init(&TIM2_Handle) != HAL_OK )
   {
     return WS2812B_ERROR;
   }

   // Timing Mode configuration: Capture Compare 1
   TIM_OC1Struct.OCMode                 = TIM_OCMODE_TIMING;
   TIM_OC1Struct.OCPolarity             = TIM_OCPOLARITY_HIGH;
   TIM_OC1Struct.Pulse                  = 9;                    // 9 pulses => 9/30 => (9/30)*1250ns = 375 ns, ws2812b datasheet: 350 ns
   
   // Configure the channel
   if( HAL_TIM_OC_ConfigChannel(&TIM2_Handle, &TIM_OC1Struct, TIM_CHANNEL_1) != HAL_OK )
   {
     return WS2812B_ERROR;
   }
   
   // Timing Mode configuration: Capture Compare 2
   TIM_OC2Struct.OCMode                 = TIM_OCMODE_TIMING;
   TIM_OC2Struct.OCPolarity             = TIM_OCPOLARITY_HIGH;
   TIM_OC2Struct.Pulse                  = 22;                   // 22 pulses => 22/30 => (22/30)*1250ns = 916 ns, ws2812b datasheet: 900 ns

   // Configure the channel
   if( HAL_TIM_OC_ConfigChannel(&TIM2_Handle, &TIM_OC2Struct, TIM_CHANNEL_2) != HAL_OK )
   {
      return WS2812B_ERROR;
   }
   
   // configure TIM2 interrupt
   HAL_NVIC_SetPriority(TIM2_IRQn, 0, 2);
   HAL_NVIC_EnableIRQ(TIM2_IRQn);
   
   return WS2812B_OK;
}

// ----------------------------------------------------------------------------
/// \brief     Initialisation of the direct memory access DMA.
///
/// \param     none
///
/// \return    none
static WS2812B_StatusTypeDef init_dma( void )
{
   // activate bus on which dma1 is connected
   __HAL_RCC_DMA1_CLK_ENABLE();
   
   // TIM2 Update event, High Output
   // DMA1 Channel2 configuration ----------------------------------------------
   DMA_HandleStruct_UEV.Instance                   = DMA1_Stream0;
   DMA_HandleStruct_UEV.Init.Request               = DMA_REQUEST_TIM2_UP;  
   DMA_HandleStruct_UEV.Init.Direction             = DMA_MEMORY_TO_PERIPH;
   DMA_HandleStruct_UEV.Init.PeriphInc             = DMA_PINC_DISABLE;
   DMA_HandleStruct_UEV.Init.MemInc                = DMA_MINC_DISABLE;
   DMA_HandleStruct_UEV.Init.Mode                  = DMA_NORMAL;
   DMA_HandleStruct_UEV.Init.PeriphDataAlignment   = DMA_PDATAALIGN_HALFWORD;
   DMA_HandleStruct_UEV.Init.MemDataAlignment      = DMA_MDATAALIGN_HALFWORD;
   DMA_HandleStruct_UEV.Init.Priority              = DMA_PRIORITY_HIGH;
   if(HAL_DMA_Init(&DMA_HandleStruct_UEV) != HAL_OK)
   {
     return WS2812B_ERROR;
   }
  
   // TIM2 CC1 event, Dataframe Output, needs bit incrementation on memory
   // DMA1 Stream 1 configuration ----------------------------------------------
   DMA_HandleStruct_CC1.Instance                   = DMA1_Stream1;
   DMA_HandleStruct_CC1.Init.Request               = DMA_REQUEST_TIM2_CH1;  
   DMA_HandleStruct_CC1.Init.Direction             = DMA_MEMORY_TO_PERIPH;
   DMA_HandleStruct_CC1.Init.PeriphInc             = DMA_PINC_DISABLE;
   DMA_HandleStruct_CC1.Init.MemInc                = DMA_MINC_ENABLE;
   DMA_HandleStruct_CC1.Init.Mode                  = DMA_NORMAL;
   DMA_HandleStruct_CC1.Init.PeriphDataAlignment   = DMA_PDATAALIGN_HALFWORD;
   DMA_HandleStruct_CC1.Init.MemDataAlignment      = DMA_MDATAALIGN_HALFWORD;
   DMA_HandleStruct_CC1.Init.Priority              = DMA_PRIORITY_HIGH;
   if(HAL_DMA_Init(&DMA_HandleStruct_CC1) != HAL_OK)
   {
     return WS2812B_ERROR;
   }
   
   // TIM2 CC2 event, Low Output
   // DMA1 Stream 2 configuration ----------------------------------------------
   DMA_HandleStruct_CC2.Instance                   = DMA1_Stream2;
   DMA_HandleStruct_CC2.Init.Request               = DMA_REQUEST_TIM2_CH2;  
   DMA_HandleStruct_CC2.Init.Direction             = DMA_MEMORY_TO_PERIPH;
   DMA_HandleStruct_CC2.Init.PeriphInc             = DMA_PINC_DISABLE;
   DMA_HandleStruct_CC2.Init.MemInc                = DMA_MINC_DISABLE;
   DMA_HandleStruct_CC2.Init.Mode                  = DMA_NORMAL;
   DMA_HandleStruct_CC2.Init.PeriphDataAlignment   = DMA_PDATAALIGN_HALFWORD;
   DMA_HandleStruct_CC2.Init.MemDataAlignment      = DMA_MDATAALIGN_HALFWORD;
   DMA_HandleStruct_CC2.Init.Priority              = DMA_PRIORITY_HIGH;
   if(HAL_DMA_Init(&DMA_HandleStruct_CC2) != HAL_OK)
   {
     return WS2812B_ERROR;
   }
   
   // register callbacks
   HAL_DMA_RegisterCallback(&DMA_HandleStruct_CC2, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
   HAL_DMA_RegisterCallback(&DMA_HandleStruct_CC2, HAL_DMA_XFER_ERROR_CB_ID, TransferError);
   
   // NVIC configuration for DMA transfer complete interrupt 
   HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 1);
   
   // Enable interrupt
   HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
   
   return WS2812B_OK;
}

// ----------------------------------------------------------------------------
/// \brief     Initialisation of the GPIOS.
///
/// \param     none
///
/// \return    none
static WS2812B_StatusTypeDef init_gpio( void )
{
   __HAL_RCC_GPIOA_CLK_ENABLE();                        //enable clock on the bus
   GPIO_InitTypeDef GPIO_InitStruct;               
   GPIO_InitStruct.Pin          = GPIO_PIN_0;           // if you want also to use pin 1, then write GPIO_PIN_0 | GPIO_PIN_1 => GPIO_PIN_1 would be the second led row
   GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_PP;  // configure pins for pp output
   GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH; // 50 MHz rate
   GPIO_InitStruct.Pull         = GPIO_NOPULL;           // disable pull
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);              // setting GPIO registers
        
   return WS2812B_OK;
}

// ----------------------------------------------------------------------------
/// \brief     Send buffer to the ws2812b leds.
///
/// \param     none
///
/// \return    none
void WS2812B_sendBuffer( void )
{
   // wait until last buffer transmission has been completed
   while( WS2812_State != WS2812B_READY );
  
   // transmission complete flag, indicate that transmission is taking place
   WS2812_State = WS2812B_BUSY;
   
   // set period to 1.25 us with the auto reload register
   TIM2->ARR = 29u;
   
   // set configuration
   DMA_SetConfiguration(&DMA_HandleStruct_UEV, (uint32_t)&WS2812_High, (uint32_t)&GPIOA->ODR, GPIO_BUFFERSIZE);
   DMA_SetConfiguration(&DMA_HandleStruct_CC1, (uint32_t)&WS2812_Buffer[0], (uint32_t)&GPIOA->ODR, GPIO_BUFFERSIZE);
   DMA_SetConfiguration(&DMA_HandleStruct_CC2, (uint32_t)&WS2812_Low, (uint32_t)&GPIOA->ODR, GPIO_BUFFERSIZE);
   
   // clear all relevant DMA flags from the channels 0, 1 and 2
   __HAL_DMA_CLEAR_FLAG(&DMA_HandleStruct_UEV, DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TEIF0_4 );
   __HAL_DMA_CLEAR_FLAG(&DMA_HandleStruct_CC1, DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TEIF1_5 );
   __HAL_DMA_CLEAR_FLAG(&DMA_HandleStruct_CC2, DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TEIF2_6 );

   // Enable the selected DMA transfer interrupts
   __HAL_DMA_ENABLE_IT(&DMA_HandleStruct_CC2, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
   
   // enable dma channels
   __HAL_DMA_ENABLE(&DMA_HandleStruct_UEV);
   __HAL_DMA_ENABLE(&DMA_HandleStruct_CC1);
   __HAL_DMA_ENABLE(&DMA_HandleStruct_CC2);
   
   // clear all TIM2 flags
   TIM2->SR = 0;
   
   // IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
   __HAL_TIM_ENABLE_DMA(&TIM2_Handle, TIM_DMA_UPDATE);
   __HAL_TIM_ENABLE_DMA(&TIM2_Handle, TIM_DMA_CC1);
   __HAL_TIM_ENABLE_DMA(&TIM2_Handle, TIM_DMA_CC2);
   
   // Enable the Output compare channel
   TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_ENABLE);
   TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_ENABLE);
   
   // preload counter with 29 so TIM2 generates UEV directly to start DMA transfer
   __HAL_TIM_SET_COUNTER(&TIM2_Handle, 29);
   
   // start TIM2
   __HAL_TIM_ENABLE(&TIM2_Handle);
}

// ----------------------------------------------------------------------------
/// \brief      DMA1 Stream 2 Interrupt Handler gets executed once the complete 
///             frame buffer has been transmitted to the LEDs.
///
/// \param      none
///
/// \return     none
void DMA1_Stream2_IRQHandler( void )
{
   HAL_DMA_IRQHandler(&DMA_HandleStruct_CC2);
}

// ----------------------------------------------------------------------------
/// \brief      Timer 2 interrupt handler.
///
/// \param      none
///
/// \return     none
void TIM2_IRQHandler( void )
{
   WS2812_TIM2_callback();
}

// ----------------------------------------------------------------------------
/// \brief      Used to wait, until deadtime/reset period is finished, thus
///             the leds have accepted their values.
///
/// \param      none
///
/// \return     none
static void WS2812_TIM2_callback( void )
{
   // Clear TIM2 Interrupt Flag
   HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
   
   // stop TIM2 now because dead period has been reached
   __HAL_TIM_DISABLE(&TIM2_Handle);
   
   // disable the TIM2 Update interrupt again so it doesn't occur while transmitting data
   __HAL_TIM_DISABLE_IT(&TIM2_Handle, TIM_IT_UPDATE);
   
   // finally indicate that the data frame has been transmitted
   WS2812_State = WS2812B_READY;
}

// ----------------------------------------------------------------------------
/// \brief      Sets the DMA Transfer parameter.
///
/// \param      [in]    pointer to a DMA_HandleTypeDef structure that contains
///                     the configuration information for the specified DMA Stream.
/// \param      [in]    The source memory Buffer address
/// \param      [in]    The destination memory Buffer address
/// \param      [in]    The length of data to be transferred from source to destination
///
/// \return     none
static void DMA_SetConfiguration( DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength )
{
   /* calculate DMA base and stream number */
   DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdma->StreamBaseAddress;
   
   if(IS_DMA_DMAMUX_ALL_INSTANCE(hdma->Instance) != 0U) /* No DMAMUX available for BDMA1 */
   {
      /* Clear the DMAMUX synchro overrun flag */
      hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;
      
      if(hdma->DMAmuxRequestGen != 0U)
      {
         /* Clear the DMAMUX request generator overrun flag */
         hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
      }
   }

   /* Clear all interrupt flags at correct offset within the register */
   regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);
   
   /* Clear DBM bit */
   ((DMA_Stream_TypeDef *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);
   
   /* Configure DMA Stream data length */
   ((DMA_Stream_TypeDef *)hdma->Instance)->NDTR = DataLength;
   
   /* Peripheral to Memory */
   if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
   {
      /* Configure DMA Stream destination address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->PAR = DstAddress;
      
      /* Configure DMA Stream source address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = SrcAddress;
   }
   /* Memory to Peripheral */
   else
   {
      /* Configure DMA Stream source address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->PAR = SrcAddress;
      
      /* Configure DMA Stream destination address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = DstAddress;
   }
}

// ----------------------------------------------------------------------------
/// \brief      DMA conversion complete callback.
///
/// \param      [in]    pointer to a DMA_HandleTypeDef structure that contains
///                     the configuration information for the specified DMA Stream.
///
/// \return     none
static void TransferComplete( DMA_HandleTypeDef *DmaHandle )
{
   // clear DMA transfer complete interrupt flag
   HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
   
   // disable the DMA channels
   __HAL_DMA_DISABLE(&DMA_HandleStruct_UEV);
   __HAL_DMA_DISABLE(&DMA_HandleStruct_CC1);
   __HAL_DMA_DISABLE(&DMA_HandleStruct_CC2);
   
   // IMPORTANT: disable the DMA requests, too!
   __HAL_TIM_DISABLE_DMA(&TIM2_Handle, TIM_DMA_UPDATE);
   __HAL_TIM_DISABLE_DMA(&TIM2_Handle, TIM_DMA_CC1);
   __HAL_TIM_DISABLE_DMA(&TIM2_Handle, TIM_DMA_CC2);
   
   // disable the capture compare events
   TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);
   TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_DISABLE);
   
   // enable TIM2 Update interrupt to append min. 50us dead/reset period
   TIM2->ARR = 1500u; // 1 tick = 41.67ns => 1500 ticks = ~60us
   TIM2->CNT = 0u;
   __HAL_TIM_ENABLE_IT(&TIM2_Handle, TIM_IT_UPDATE);
}

// ----------------------------------------------------------------------------
/// \brief      DMA transfer error callback.
///
/// \param      [in]    pointer to a DMA_HandleTypeDef structure that contains
///                     the configuration information for the specified DMA Stream.
///
/// \return     none
static void TransferError( DMA_HandleTypeDef *DmaHandle )
{
   while(1)
   {
      // do nothing stay here
   }
}

// ----------------------------------------------------------------------------
/// \brief      Clear ws2812b buffer. All pixels are black after sending.
///
/// \param      none
///
/// \return     none
void WS2812B_clearBuffer( void )
{
   // wait until last buffer transmission has been completed
   while( WS2812_State != WS2812B_READY );
  
   // clear frame buffer
   for(uint8_t y=0; y<ROW;y++)
   {
      for(uint16_t x=0; x<COL; x++)
      {
         WS2812B_setPixel(y, x, 0x00, 0x00, 0x00);
      }
   }
}

// ----------------------------------------------------------------------------
/// \brief      This function sets the color of a single pixel.
///
/// \param      [in]    uint8_t row
/// \param      [in]    uint16_t col
/// \param      [in]    uint8_t red
/// \param      [in]    uint8_t green
/// \param      [in]    uint8_t blue
///
/// \return     none
void WS2812B_setPixel( uint8_t row, uint16_t col, uint8_t red, uint8_t green, uint8_t blue )
{
   // check if the col and row are valid
   if( row >= ROW || col >= COL )
   {
      return;
   }
   
   // wait until last buffer transmission has been completed
   while( WS2812_State != WS2812B_READY );
   
   // write pixel into the buffer
   for( uint8_t i = 0; i < 8; i++ )
   {
      /* clear the data for pixel */
      WS2812_Buffer[((col*24)+i)] &= ~(0x01<<row);
      WS2812_Buffer[((col*24)+8+i)] &= ~(0x01<<row);
      WS2812_Buffer[((col*24)+16+i)] &= ~(0x01<<row);
      
      /* write new data for pixel */
      WS2812_Buffer[((col*24)+i)] |= ((((green<<i) & 0x80)>>7)<<row);
      WS2812_Buffer[((col*24)+8+i)] |= ((((red<<i) & 0x80)>>7)<<row);
      WS2812_Buffer[((col*24)+16+i)] |= ((((blue<<i) & 0x80)>>7)<<row);
   }
}

#endif

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
CEXTERN void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base) {
  if (htim_base->Instance == TIM3) {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 DMA Init */
    /* TIM3_CH4 Init */
    hdma_tim3_ch4.Instance                 = DMA1_Stream4;
    hdma_tim3_ch4.Init.Request             = DMA_REQUEST_TIM3_CH4;
    hdma_tim3_ch4.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch4.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim3_ch4.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch4.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch4.Init.Mode                = DMA_CIRCULAR;
    hdma_tim3_ch4.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tim3_ch4.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim3_ch4) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC4], hdma_tim3_ch4);

    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
    // NVIC configuration for DMA transfer complete interrupt
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 1);

    // Enable interrupt
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  }
  LOGI("UHU");
}

#endif // WITH_WM2812_CONTROL