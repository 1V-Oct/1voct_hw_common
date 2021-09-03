#include "i2c.h"
#include "vo_logger.h"
#include "vo_memory.h"
#include "vo_string.h"

#if WITH_I2C

I2C_HandleTypeDef hi2c2;

void i2c_init(void) {
  hi2c2.Instance              = I2C2;
  hi2c2.Init.Timing           = 100000;
  hi2c2.Init.OwnAddress1      = 0;
  hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2      = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    // LOGE("init fail");
  }
  /** Configure Analogue filter
  */
  // if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
  //   LOGE("analog fail");
  // }

  I2C_HandleTypeDef *hi2c = &hi2c2;
  I2C_TypeDef *      i2c  = hi2c->Instance;

  __HAL_I2C_DISABLE(hi2c);
  /* Reset I2Cx ANOFF bit */
  hi2c->Instance->CR1 &= ~(I2C_CR1_ANFOFF);
  /* Set analog filter bit*/
  hi2c->Instance->CR1 |= I2C_ANALOGFILTER_ENABLE;

  // __HAL_I2C_ENABLE(hi2c);

  /** Configure Digital filter
  */
  // if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
  //   LOGE("digital fail");
  // }
  uint32_t DigitalFilter = 0;
  uint32_t tmpreg;
  // __HAL_I2C_DISABLE(hi2c);

  /* Get the old register value */
  tmpreg = hi2c->Instance->CR1;

  /* Reset I2Cx DNF bits [11:8] */
  tmpreg &= ~(I2C_CR1_DNF);

  /* Set I2Cx DNF coefficient */
  tmpreg |= DigitalFilter << 8U;

  /* Store the new register value */
  hi2c->Instance->CR1 = tmpreg;

  __HAL_I2C_ENABLE(hi2c);
}

void i2c_detect(void) {
  uint8_t devices = 0u;
  // LOGI("Searching for I2C devices on the bus...\n");
  /* Values outside 0x03 and 0x77 are invalid. */
  for (uint8_t i = 0x03u; i < 0x78u; i++) {
    uint8_t address = i << 1u;
    /* In case there is a positive feedback, print it out. */
    if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2, address, 3u, 10u)) {
      LOGI("Device found: 0x%x", address);
      devices++;
    }
    // LOGI(".");
  }
  /* Feedback of the total number of devices. */
  if (0u == devices) {
    LOGW("No dev");
  } else {
    LOGI("Devcount: %d", devices);
  }
}

#define I2C_7BIT_ADDR  (0 << 31)
#define I2C_10BIT_ADDR (1 << 31)

#define I2C_READ  0
#define I2C_WRITE 1


#if 1

// #include "i2c.h"


static uint32_t setupTiming(i2cSpeed_t spd, uint32_t clockFreq) {
    (void) spd;
    (void) clockFreq;
    uint32_t presc = 0;
    uint32_t sdadel = 2;
    uint32_t scldel = 2;
    uint32_t scll = 6;
    uint32_t sclh = 7;

    return  presc << 28 |
            scldel << 20 |
            sdadel << 16 |
            sclh << 8 |
            scll;
}

void i2c_init_int(I2C_TypeDef *i2c, i2cSpeed_t spd) {
    // assert(i2c == I2C1 || i2c == I2C2);
    // assert(spd < i2cSpeed_count);

    // if (i2c == I2C1) {
    //     rccEnableI2C1();
    // } else if (i2c == I2C2) {
    //     rccEnableI2C2();
    // }

    // Setup timing register
    i2c->TIMINGR = setupTiming(spd, SystemCoreClock);
    
    // Reset state
    i2c->CR1 &= ~I2C_CR1_PE;
}
#endif
#if 1
static uint32_t i2cSetup(uint32_t addr, uint8_t direction) {
  uint32_t ret = 0;
  if (addr & I2C_10BIT_ADDR) {
    ret = (addr & 0x000003FF) | I2C_CR2_ADD10;
  } else {
    // 7 Bit Address
    ret = (addr & 0x0000007F) << 1;
  }

  if (direction == I2C_READ) {
    ret |= I2C_CR2_RD_WRN;
    if (addr & I2C_10BIT_ADDR) {
      ret |= I2C_CR2_HEAD10R;
    }
  }

  return ret;
}
#endif
#if 1
// Will return the number of data bytes written to the device
uint32_t i2c_write(I2C_TypeDef *i2c, uint32_t addr, uint8_t *txBuffer,
                  uint32_t len) {
  // assert(i2c == I2C1 || i2c == I2C2);
  // assert(txBuffer != NULL);
  // assert(len > 0);

  uint32_t numTxBytes = 0;

  i2c->CR1 &= ~I2C_CR1_PE;
  i2c->CR2 = 0;

  i2c->CR2 = i2cSetup(addr, I2C_WRITE);

  if (len > 0xFF) {
    i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
  } else {
    i2c->CR2 |= ((len & 0xFF) << 16) | I2C_CR2_AUTOEND;
  }
  i2c->CR1 |= I2C_CR1_PE;
  i2c->CR2 |= I2C_CR2_START;
  while (i2c->CR2 & I2C_CR2_START)
    ;
  uint8_t  done = 0;
  uint32_t i    = 0;
  while (!done && i < 0x0000001F) {
    i++;
    if (i2c->ISR & I2C_ISR_NACKF) {
      // Was not acknowledged, disable device and exit
      done = 1;
    }

    if (i2c->ISR & I2C_ISR_TXIS) {
      // Device acknowledged and we must send the next byte
      if (numTxBytes < len) {
        i2c->TXDR = txBuffer[numTxBytes++];
      }

      // assert(numTxBytes <= len);
      i = 0;
    }

    if (i2c->ISR & I2C_ISR_TC) {
      done = 1;
    }

    if (i2c->ISR & I2C_ISR_TCR) {
      i = 0;
      if ((len - numTxBytes) > 0xFF) {
        i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
      } else {
        i2c->CR2 &= ~(0x00FF0000 | I2C_CR2_RELOAD);
        i2c->CR2 |= ((len - numTxBytes) & 0xFF) << 16 |
                    I2C_CR2_AUTOEND;
      }
    }
  }
  i2c->CR1 &= ~I2C_CR1_PE;
  return numTxBytes;
}
#endif
#if 1
uint32_t i2c_read(I2C_TypeDef *i2c, uint8_t addr, uint8_t *rxBuffer,
                 uint32_t numBytes) {
    // assert(i2c == I2C1 || i2c == I2C2);
    // assert(rxBuffer != NULL);
    // assert(numBytes > 0);

    uint32_t numRxBytes = 0;

    i2c->CR1 &= ~I2C_CR1_PE;
    i2c->CR2 = 0;

    i2c->CR2 = i2cSetup(addr, I2C_READ);

    if (numBytes > 0xFF) {
        i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
    } else {
        i2c->CR2 |= ((numBytes & 0xFF) << 16) | I2C_CR2_AUTOEND;
    }
    i2c->CR1 |= I2C_CR1_PE;
    i2c->CR2 |= I2C_CR2_START;

    while(i2c->CR2 & I2C_CR2_START);
    uint8_t done = 0;
    uint32_t i = 0;
    while (!done && i < 0x0000001F) {
        i++;

        if (i2c->ISR & I2C_ISR_RXNE) {
           // Device acknowledged and we must send the next byte
            if (numRxBytes < numBytes){
                rxBuffer[numRxBytes++] = i2c->RXDR;
            }

            // assert(numRxBytes <= numBytes);

            i = 0;
        }

        if (i2c->ISR & I2C_ISR_TC) {
            done = 1;
        }

        if (i2c->ISR & I2C_ISR_TCR) {
            i = 0;
            if ((numBytes - numRxBytes) > 0xFF) {
                i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
            } else {
                i2c->CR2 &= ~(0x00FF0000 | I2C_CR2_RELOAD);
                i2c->CR2 |= ((numBytes - numRxBytes) & 0xFF) << 16 |
                            I2C_CR2_AUTOEND;
            }
        }

    }
    i2c->CR1 &= ~I2C_CR1_PE;
    return numRxBytes;
}
#endif
#define ONEPAGE (16)
/*
 * brief: Used to write data array into specified EEPROM memory location
 * param: DevAddress: EEPROM device address
 * param: MemAddress: Memory address location of EEPROM that data needs to be written to.
 * param: pData: base address of data array that needs to be written in EEPROM
 * param: len: Total number of bytes need to write into EEPROM in bytes
 */

HAL_StatusTypeDef i2c_eeprom_write(uint16_t DevAddress, uint16_t MemAddress, const uint8_t *pData, uint16_t len) {
  HAL_StatusTypeDef returnValue = HAL_ERROR;
  uint8_t           data[18]    = {0};

  uint16_t completed = 0;
  while (len > 0) {
    /* We compute the MSB and LSB parts of the memory address */
    data[1] = (uint8_t)((MemAddress & 0xFF00) >> 8);
    data[0] = (uint8_t)(MemAddress & 0xFF);

    /* And copy the content of the pData array in the temporary buffer */
    uint32_t size = ONEPAGE;
    if (len < ONEPAGE) {
      size = len;
    }

    vo_memcpy(data + 2, pData + completed, size);
    completed += size;

    returnValue = HAL_I2C_Master_Transmit(&hi2c2, DevAddress, data, size + 2, HAL_MAX_DELAY);
    if (returnValue != HAL_OK)
      return returnValue;

    len -= size;
    MemAddress += size;

    /* We wait until the EEPROM effectively stores data in memory */
    while (HAL_I2C_Master_Transmit(&hi2c2, DevAddress, 0, 0, HAL_MAX_DELAY) != HAL_OK)
      ;
    returnValue = HAL_OK;
  }
  return returnValue;
}

/*
 * brief: Used to read data from given EEPROM memory location
 * param: DevAddress: EEPROM device address
 * param: MemAddress: Memory address location of EEPROM where the data needs to be read from.
 * param: pData: Base address of data array where the data read from the EEPROM is stored
 * param: len: Total number of bytes that need to be read from EEPROM in bytes
 */

HAL_StatusTypeDef i2c_eeprom_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len) {
  HAL_StatusTypeDef returnValue;
  uint8_t           addr[2];

  /* We compute the MSB and LSB parts of the memory address */
  addr[1] = (uint8_t)((MemAddress & 0xFF00) >> 8);
  addr[0] = (uint8_t)(MemAddress & 0xFF);

  /* First we send the memory location address where start reading data */
  returnValue = HAL_I2C_Master_Transmit(&hi2c2, DevAddress, addr, 2, HAL_MAX_DELAY);
  if (returnValue != HAL_OK)
    return returnValue;

  // i2c_write(I2C2, DevAddress, addr, 2);
  /* Next we can retrieve the data from EEPROM */
  while (HAL_I2C_Master_Receive(&hi2c2, DevAddress, pData, len, HAL_MAX_DELAY) != HAL_OK)
    ;

  // i2c_read(I2C2, DevAddress, pData, len);
  return HAL_OK;
  // return returnValue;
}


#endif