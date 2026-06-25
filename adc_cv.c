#include "adc_cv.h"
#include "adc_helper.h"
#include "cv_inputs.h"
#include "gpio.h"
#include "midi_port.h"
#include "nor_flash.h"
// #include "sto_common.h"
#include "sys_settings.h"
#include "vo_logger.h"
#include "vo_math.h"
// #include "chunk.h"
#include "cv_inputs.h"
#include "pots.h"

extern ADC_HandleTypeDef hadc1;

//#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  14)   /* Size of array aADCxConvertedData[] */

// DMA_BUFFER ALIGN_32BYTES (ADC_CV_VALUES);
volatile uint32_t adc_dma_done;

__attribute__((section(".dma_buffer"))) __attribute__((aligned(32))) ADC_CV_VALUES;

#define WITH_ADC1_DMA (1)

static const adc_channel_list_t adc_cv_ch_list[ADC_CV_CHANNEL_COUNT] = {
  {ADC_CHANNEL_18, ADC_REGULAR_RANK_1, GPIOA, GPIO_PIN_4},
  {ADC_CHANNEL_19, ADC_REGULAR_RANK_2, GPIOA, GPIO_PIN_5},
  {ADC_CHANNEL_3, ADC_REGULAR_RANK_3, GPIOA, GPIO_PIN_6},
  {ADC_CHANNEL_7, ADC_REGULAR_RANK_4, GPIOA, GPIO_PIN_7},
  {ADC_CHANNEL_4, ADC_REGULAR_RANK_5, GPIOC, GPIO_PIN_4},
  {ADC_CHANNEL_8, ADC_REGULAR_RANK_6, GPIOC, GPIO_PIN_5},
  {ADC_CHANNEL_9, ADC_REGULAR_RANK_7, GPIOB, GPIO_PIN_0},
  {ADC_CHANNEL_5, ADC_REGULAR_RANK_8, GPIOB, GPIO_PIN_1},
  {ADC_CHANNEL_16, ADC_REGULAR_RANK_9, GPIOA, GPIO_PIN_0},
  {ADC_CHANNEL_17, ADC_REGULAR_RANK_10, GPIOA, GPIO_PIN_1},
  {ADC_CHANNEL_14, ADC_REGULAR_RANK_11, GPIOA, GPIO_PIN_2},
  {ADC_CHANNEL_15, ADC_REGULAR_RANK_12, GPIOA, GPIO_PIN_3},
  // { ADC_CHANNEL_10,   ADC_REGULAR_RANK_13,  GPIOA,  GPIO_PIN_4 },
  // { ADC_CHANNEL_11,   ADC_REGULAR_RANK_14,  GPIOA,  GPIO_PIN_4 },
};

float cal_ch[4]; // = {0.f, 0.f, 0.f, 0.f };

void adc_cv_init(void) {
  //   int i;
  //   for(i = 0 ; i < 4 ; i++) {
  //     *((uint32_t *)(&cal_ch[i])) = 0xabcd8768 << (i * 2);
  //     LOGI("CH%d %f %08x", i, cal_ch[i], *(uint32_t *)(&cal_ch[i]));
  // }
  adc_init_channels(&hadc1, adc_cv_ch_list, ADC_CV_CHANNEL_COUNT);
  LOGI("ADC CV Init OK");

  // HAL_ADC_Start(&hadc1);
#if WITH_ADC1_DMA
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_cv_values, ADC_CV_CHANNEL_COUNT) != HAL_OK) {
    LOGE("Cannot start ADC1 DMA");
  }
#endif
  cv_normalization_probe(1);
  HAL_Delay(200);
  cv_normalization_probe(0);
}

uint16_t adc_cv_values_cached[16];

// static void adc_cv_loop(void) {
//   while (1) {
//     HAL_Delay(100);
//     uart_printf(".");
//     ADC1->CR |= (uint32_t)(ADC_CR_ADSTART);
//   }
// }

void cv_normalization_probe(uint8_t set) {
  // LOGI("NP: %d", set);
  HAL_GPIO_WritePin(NORMALIZATION_PROBE_GPIO_Port, NORMALIZATION_PROBE_Pin, set ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#define ONE_POLE(out, in, coefficient) out += (coefficient) * ((in)-out);
#define FLT_ONE_POLE(o, i, c)          o = i - c * o;

static float _note[4];

cv_cal_t cv_cal;



static const char *cv_cal_filename = "/.3318.cal";

#if 0
void cv_cal_voct(void) {
  int      i;
  uint32_t cal;
  uint32_t cal_min;
  uint32_t cal_max;
  uint32_t cal_read;

  float cal_f;
  //  LOGI("Insert 1V")
  for (i = 0; i < 4; i++) {
    int j;
    cal       = 0;
    cal_f     = 0.f;
    cal_ch[i] = 0.f;
    cv_div[i] = 6600.f;
    cv_cor[i] = 0.f;
    cal_max   = 0;
    cal_min   = 0xffffffff;
    for (j = 0; j < 1000; j++) {
      cal_read = adc_cv_get_value_u16(8 + i);
      cal += cal_read;
      if (cal_read > cal_max)
        cal_max = cal_read;
      if (cal_read < cal_min)
        cal_min = cal_read;
      cal_f += cv_get_note_f(i);
      HAL_Delay(1);
    }
    cal /= 1000;
    cal_f /= 1000.f;
    cal_ch[i] = 36.f - cal_f;
    LOGI("CH%d Average: %d %f Min: %d Max %d", i, cal, cal_f, cal_min, cal_max);
    // LOGI("CH%d %f %08x", i, cal_ch[i], *(uint32_t *)(&cal_ch[i]));
  }
  nor_erase_sector(0);
  nor_write_memory(0, &cal_ch[0], 16);
  // for(i = 0 ; i < 4 ; i++) {
  //   LOGI("CH%d %f %08x", i, cal_ch[i], *(uint32_t *)(&cal_ch[i]));
  // }
  uint32_t load_size = 4 * sizeof(float);
  sto_save(cv_cal_filename, &cal_ch[0], &load_size);
  cv_cal_load();
}
#endif

typedef struct {
  float    octave_divider;
  float    note_correct;
  uint32_t min;
  uint32_t max;
} cal_data_t;

cal_data_t cal_data[4];

void cv_cal_cvs(void) {
  uint16_t minv[8];
  uint16_t maxv[8];
  int      i;

  for (i = 0; i < 8; i++) {
    minv[i] = 0xffff;
    maxv[i] = 0x0;
  }
  uint16_t val;
  for (i = 0; i < 8; i++) {
    val = adc_cv_get_value_u16(i);
    if (val < minv[i])
      minv[i] = val;
    if (val > maxv[i])
      maxv[i] = val;
  }
  for (i = 0; i < 8; i++) {
    uint32_t v;
    v = minv[i];
    v += maxv[i];
    v >>= 1;
    cv_cal.base[i] = (uint16_t)v;
    LOGI("CV%d MIN: %d MAX: %d AVG: %d", minv[i], maxv[i], (uint16_t)v);
  }
}

void cv_cal_calibrate(cal_data_t *d) {
  int      i;
  uint32_t cal_val;
  uint32_t cal_min;
  uint32_t cal_max;
  uint32_t cal_read;

  float cal_f;

  for (i = 0; i < 4; i++) {
    int j;
    cal_val   = 0;
    cal_f     = 0.f;
    cal_ch[i] = 0.f;
    cal_max   = 0;
    cal_min   = 0xffffffff;
    for (j = 0; j < 100; j++) {
      cv_scan();
      cal_read = adc_cv_get_value_u16(8 + i);
      cal_val += cal_read;
      if (cal_read > cal_max)
        cal_max = cal_read;
      if (cal_read < cal_min)
        cal_min = cal_read;
      cal_f += cv_get_note_f(i);
      HAL_Delay(1);
    }
    cal_val /= 1000;
    cal_f /= 1000.f;
    cal_ch[i] = 36.f - cal_f;
    // LOGI("CH%d Average: %d %f Min: %d Max %d", i, cal, cal_f, cal_min, cal_max);
    // LOGI("CH%d %f %08x", i, cal_ch[i], *(uint32_t *)(&cal_ch[i]));
    d[i].max  = cal_max;
    d[i].min  = cal_min;
  }
}

typedef struct {
  uint16_t min;
  uint16_t max;
} cal_cv_data_t;

void cv_cal_cv_zero(cal_cv_data_t *d) {
  uint16_t cal_read;

  int j, i;
  for (i = 0; i < 8; i++) {
    d[i].max = 0;
    d[i].min = 0xffff;
  }
  for (j = 0; j < 100; j++) {
    cv_scan();
    for (i = 0; i < 8; i++) {
      cal_read = adc_cv_get_value_u16(i);
      if (cal_read > d[i].max)
        d[i].max = cal_read;
      if (cal_read < d[i].min)
        d[i].min = cal_read;
      HAL_Delay(1);
    }
    // LOGI("CH%d Average: %d %f Min: %d Max %d", i, cal, cal_f, cal_min, cal_max);
    // LOGI("CH%d %f %08x", i, cal_ch[i], *(uint32_t *)(&cal_ch[i]));
  }
  LOGI("CALIBRATE CV:");
  for (i = 0; i < 8; i++) {
    LOGI("%d: %4x %4x %4x", i, d[i].min, d[i].max, d[i].max - d[i].min);
  }
}

void cv_cal_voct_full(void) {
  uint8_t    buf[3];
  cal_data_t d[6][4];
  cal_data_t dm[6][4];
  cal_data_t dr[6][4];

  buf[0] = 0xb0;
  buf[1] = 121;
  buf[2] = 0;
  midi_port_send(1, buf, 3);
  buf[0] = 0xb0;
  buf[1] = 123;
  buf[2] = 0;
  midi_port_send(1, buf, 3);

  int i;
  for (i = 0; i < 6; i++) {
    buf[0] = 0x90;
    buf[1] = i * 12 + 0;
    buf[2] = 55;
    //    HAL_Delay(500);
    midi_port_send(1, buf, 3);
    HAL_Delay(250);

    // HAL_Delay(250);
    for (int j = 0; j < 200; j++) {
      cv_scan();
      HAL_Delay(5);
    }
    cv_cal_calibrate(&d[i][0]);
    LOGI("%d: 1 MIN %4d %4d %4d %4d", i, d[i][0].min, d[i][1].min, d[i][2].min, d[i][3].min);
    LOGI("%d: 1 MAX %4d %4d %4d %4d", i, d[i][0].max, d[i][1].max, d[i][2].max, d[i][3].max);

    for (int j = 0; j < 200; j++) {
      cv_scan();
      HAL_Delay(5);
    }
    cv_cal_calibrate(&dm[i][0]);
    LOGI("%d: 2 MIN %4d %4d %4d %4d", i, dm[i][0].min, dm[i][1].min, dm[i][2].min, dm[i][3].min);
    LOGI("%d: 2 MAX %4d %4d %4d %4d", i, dm[i][0].max, dm[i][1].max, dm[i][2].max, dm[i][3].max);
    // LOGI("%d: MAX %4d %4d %4d %4d", i, d[0].max, d[1].max, d[2].max, d[3].max);

    if (i > 0) {
      for (int k = 0; k < 4; k++) {
        dr[i][k].min = d[i - 1][k].min - d[i][k].min;
      }
      LOGI("%d: RES %4d %4d %4d %4d", i, dr[i][0].min, dr[i][1].min, dr[i][2].min, dr[i][3].min);
    }

    buf[0] = 0x80;
    buf[1] = i * 12 + 0;
    buf[2] = 64;
    midi_port_send(1, buf, 3);
  }
  uint32_t nor_addr = 4096;
  nor_addr += nor_write_memory(nor_addr, d, sizeof(d));

  buf[0] = 0x90;
  buf[1] = 36;
  buf[2] = 55;
  //    HAL_Delay(500);
  midi_port_send(1, buf, 3);
  HAL_Delay(250);

  cv_cal_calibrate(&dm[0][0]);

  // // HAL_Delay(250);
  // for (int j = 0; j < 200; j++) {
  //   cv_scan();
  //   HAL_Delay(5);
  // }

  for (int j = 0; j < 4; j++) {
    float v;
    v = 0.f;
    for (i = 2; i < 6; i++) {
      v += dr[i][j].min;
    }
    v /= 4.f;
    cv_cal.div[j] = 12.0f / v;
    cv_cal.cor[j] = 0.f;
    float note    = cv_get_note(j);
    cv_cal.cor[j] = 36 + 36 - note;
  }

  buf[0] = 0x80;
  buf[1] = 36;
  buf[2] = 0;
  midi_port_send(1, buf, 3);

  cal_cv_data_t cal_cv[8];
  cv_cal_cv_zero(cal_cv);
  LOGI("Writing cal_cv %d", sizeof(cal_cv));
  nor_addr += nor_write_memory(nor_addr, cal_cv, sizeof(cal_cv));
}

uint8_t adc_cv_plug[16];

int8_t norm_count;

void adc_cv_scan(void) {
#if 0
  norm_count++;
  if (norm_count == 99) {
    cv_normalization_probe(1);
  }
  if (norm_count == 102) {
    cv_normalization_probe(0);
  }
  int i;
  uint16_t *s1 = (uint16_t *)adc_cv_values;
  uint16_t *s2 = (uint16_t *)adc_cv_values_cached;
  uint16_t v;
  for(i = 0 ; i < ADC_CV_CHANNEL_COUNT ; i++) {
    v = *s1++;
    if ((v & 0xf000) != 0x8000) {
      adc_cv_plug[i] = 1;
      *s2++ = v;
    } else {
    if ((*s1++ >> 12) == 0x5 && (*s2++ >> 12) == 0x08) {
      adc_cv_plug[i] = 0;
    } else {
      // LOGI("Plug detected %d", i);
      
    }
  }



  if (norm_count == 105) {
    norm_count = 0;
  }
#endif
#if 0
  uint32_t *s = (uint32_t *)adc_cv_values;
  uint32_t *d = (uint32_t *)adc_cv_values_cached;
  *d++        = *s++;
  *d++        = *s++;
  *d++        = *s++;
  *d++        = *s++;
  *d++        = *s++;
  *d++        = *s++;
  *d++        = *s++;
  *d++        = *s++;
#else
  for (int i = 0; i < 12; i++) {
#if 0
    int32_t nv;
    int32_t ov;
    nv = adc_cv_values[i];
    ov = adc_cv_values_cached[i];
    nv = nv - ov;
    int32_t xv = __builtin_abs(nv);
    if (xv >= 0x400) nv >>= 0;
    else if (xv > 0x100) nv >>= 1;
    else nv >>= 5;
    // LOGI("%d: %d %d", i, ov, nv);
#else
    int32_t nnv = adc_cv_values[i] & 0xffc0;
#if (WITH_ADC_FILTER == 1)
    if (sys_set_exp[SYS_SET_EXPCVSMOOTH] == 0) {
      int32_t  ov   = adc_cv_values_cached[i];
      int32_t  nv   = nnv - ov;
      uint32_t anv  = __builtin_abs(nv);
      uint8_t  lz   = __builtin_clz(anv);
      uint8_t  bits = 24 + sys_set_exp[SYS_SET_EXPCVTHRESH];
      if (lz > bits)
        nv >>= (lz - bits);
      adc_cv_values_cached[i] = (ov + nv) & 0xffff;
    } else {
      adc_cv_values_cached[i] = nnv;
    }
#else
    adc_cv_values_cached[i] = nnv;
#endif
#endif
  }
#endif

#if WITH_ADC1_DMA
  ADC1->CR |= (uint32_t)(ADC_CR_ADSTART);
  // ADC3->CR |= (uint32_t)(ADC_CR_ADSTART);
#else
  int i;
  for (i = 0; i < 14; i++) {
    uint16_t value_adc;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_cv_values[i] = HAL_ADC_GetValue(&hadc1);
    // HAL_Delay(30);
  }
#endif
}




float cv_get_note_f(uint8_t ch) {
#if 1
  int32_t val32 = (0xffff - (uint16_t)adc_cv_get_value_u16(8 + ch));
  // val32 &= ~0x0ff;
  //  float valf = val32 * 12.f / 6610.f; // + cal_ch[ch]; //2.7f; //2.91f;// * -0.36f + 6600;
  float valf    = val32 * cv_cal.div[ch] + cv_cal.cor[ch]; // + cal_ch[ch]; //2.7f; //2.91f;// * -0.36f + 6600;
  // return (uint32_t)valf;
  return valf;
#else
  return 0.f;
#endif
}

float cv_get_note(uint8_t ch) {
  float valf = cv_get_note_f(ch);
  float valr;
  if (sys_set_exp[SYS_SET_VOCT_QUANTISE] == 0) {
    valr = vo_round(valf);
  } else {
    valr = valf;
  }
  // LOGI("%f %f", valf, valr);
  return valr;
#if 0
  valf = vo_round(valf * 4.f ) / 4.f;
  // return valf;
  _note[ch] = valf;
  //ONE_POLE(_note[ch], valf, 0.1f);
  return _note[ch];
#endif
}
