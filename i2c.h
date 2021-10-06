#pragma once
#include "app_config.h"

#if WITH_I2C
__CEXTERN_START
typedef enum {
    i2cSpeed_std,
    i2cSpeed_fast,
    i2cSpeed_fastPlus,
    i2cSpeed_count,
} i2cSpeed_t;


void i2c_init(void);
void i2c_init_int(I2C_TypeDef *i2c, i2cSpeed_t spd);
uint8_t i2c_detect(void);
HAL_StatusTypeDef i2c_eeprom_write(uint16_t DevAddress, uint16_t MemAddress, const uint8_t *pData, uint16_t len);
HAL_StatusTypeDef i2c_eeprom_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);

__CEXTERN_END

#endif