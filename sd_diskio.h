/**
 ******************************************************************************
 * @file    sd_diskio.h
 * @brief   Header for sd_diskio.c module
 ******************************************************************************
 */

#pragma once
#include "app_config.h"
/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"



#define STO_CARD_NOT_INSERTED (0)
#define STO_CARD_INSERTED     (1)
#define STO_CARD_MOUNTED      (2)


__CEXTERN_START

extern const Diskio_drvTypeDef SD_Driver;
uint8_t sd_check_card_state(void);
uint8_t sd_process(uint8_t old_status, TCHAR const *SDPath, uint8_t wide_op);


__CEXTERN_END