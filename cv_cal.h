#pragma once
#include "app_config.h"

typedef struct {
  uint32_t valid;
  float  base[4];
  float    mult[4];
} out_cal_t;

typedef struct {
  float     div[4];
  float     cor[4];
  uint16_t  base[8];
  out_cal_t out;
} cv_cal_t;

extern cv_cal_t cv_cal;

#define VALID_DATA        (0x1337F001)
