#include "quadspi.h"
#include "vo_chunk.h"
#include "vo_logger.h"

#define WITH_QSPI_DEBUG (0)

static uint8_t QSPI_WriteEnable(void);
uint8_t        QSPI_AutoPollingMemReady(void);
static uint8_t QSPI_Configuration(void);
static uint8_t QSPI_ResetChip(void);

QSPI_HandleTypeDef hqspi;

/* QUADSPI init function */
void MX_QUADSPI_Init(void) {

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  hqspi.Instance                = QUADSPI;
  hqspi.Init.ClockPrescaler     = 6;
  hqspi.Init.FifoThreshold      = 1;
  hqspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize          = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID            = QSPI_FLASH_ID_2;
  hqspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *qspiHandle) {

  GPIO_InitTypeDef         GPIO_InitStruct     = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (qspiHandle->Instance == QUADSPI) {
    /* USER CODE BEGIN QUADSPI_MspInit 0 */

    /* USER CODE END QUADSPI_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInitStruct.QspiClockSelection   = RCC_QSPICLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Error_Handler();
    }

    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PB2     ------> QUADSPI_CLK
    PE7     ------> QUADSPI_BK2_IO0
    PE8     ------> QUADSPI_BK2_IO1
    PE9     ------> QUADSPI_BK2_IO2
    PE10     ------> QUADSPI_BK2_IO3
    PC11     ------> QUADSPI_BK2_NCS
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN QUADSPI_MspInit 1 */

    /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *qspiHandle) {

  if (qspiHandle->Instance == QUADSPI) {
    /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

    /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PB2     ------> QUADSPI_CLK
    PE7     ------> QUADSPI_BK2_IO0
    PE8     ------> QUADSPI_BK2_IO1
    PE9     ------> QUADSPI_BK2_IO2
    PE10     ------> QUADSPI_BK2_IO3
    PC11     ------> QUADSPI_BK2_NCS
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);

    /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

    /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* USER CODE BEGIN 1 */
/* QUADSPI init function */
uint8_t CSP_QUADSPI_Init(void) {

  // prepare QSPI peripheral for ST-Link Utility operations
  hqspi.Instance = QUADSPI;
  if (HAL_QSPI_DeInit(&hqspi) != HAL_OK) {
    return HAL_ERROR;
  }

  MX_QUADSPI_Init();

  if (QSPI_ResetChip() != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(1);
  LOGI("Autopolling");
  if (QSPI_AutoPollingMemReady() != HAL_OK) {
    return HAL_ERROR;
  }

  LOGI("WE");
  if (QSPI_WriteEnable() != HAL_OK) {

    return HAL_ERROR;
  }
  LOGI("Config")
  if (QSPI_Configuration() != HAL_OK) {
    return HAL_ERROR;
  }
  LOGI("#Done!");
  HAL_Delay(100);

  return HAL_OK;
}

uint8_t CSP_QSPI_Erase_Chip(void) {

  QSPI_CommandTypeDef sCommand;

  if (QSPI_WriteEnable() != HAL_OK) {
    return HAL_ERROR;
  }

  /* Erasing Sequence --------------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = CHIP_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }

  if (QSPI_AutoPollingMemReady() != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

uint8_t QSPI_AutoPollingMemReady(void) {

  QSPI_CommandTypeDef     sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig  = {0};
  HAL_StatusTypeDef       ret;

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match              = 0x00;
  sConfig.Mask               = 0x01;
  sConfig.MatchMode          = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize    = 1;
  sConfig.Interval           = 0x10;
  sConfig.AutomaticStop      = QSPI_AUTOMATIC_STOP_ENABLE;
  if ((ret = HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig,
                                  HAL_MAX_DELAY)) != HAL_OK) {
    return ret;
  }
  return HAL_OK;
}

static uint8_t QSPI_WriteEnable(void) {
  QSPI_CommandTypeDef     sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig  = {0};
  HAL_StatusTypeDef       ret;

  /* Enable write operations ------------------------------------------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if ((ret = HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig,
                                  HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }
  return HAL_OK;
}
/*Enable quad mode*/
/* Las versiones acabadas en IQ lo llevan activado por defecto, como en mi caso. */

uint8_t QSPI_Configuration(void) {

  QSPI_CommandTypeDef sCommand = {0};
  uint8_t             reg;
  HAL_StatusTypeDef   ret;

#if 0

  /* Read Volatile Configuration register 1 --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  if ((ret = HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }
  LOGI("REG1: %02x", reg);

  /* Enable Volatile Write operations ---------------------------------------- */
  sCommand.DataMode    = QSPI_DATA_NONE;
  sCommand.Instruction = VOLATILE_SR_WRITE_ENABLE;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return ret;
  }

  // /* Write Volatile Configuration register 2 (QE = 1) -- */
  // sCommand.DataMode    = QSPI_DATA_1_LINE;
  // sCommand.Instruction = WRITE_STATUS_REG2_CMD;
  // LOGI("REG2: %02x", reg);
  // reg |= 2; // QE bit

  // if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
  //   return ret;
  // }

  // if (HAL_QSPI_Transmit(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
  //   return ret;
  // }

#endif

  /* Read Volatile Configuration register 2 --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG2_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  if ((ret = HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  /* Enable Volatile Write operations ---------------------------------------- */
  sCommand.DataMode    = QSPI_DATA_NONE;
  sCommand.Instruction = VOLATILE_SR_WRITE_ENABLE;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return ret;
  }

  /* Write Volatile Configuration register 2 (QE = 1) -- */
  sCommand.DataMode    = QSPI_DATA_1_LINE;
  sCommand.Instruction = WRITE_STATUS_REG2_CMD;
  LOGI("REG2: %02x", reg);
  reg &= ~0x40; // clear CMP bit
  reg |= 2;     // QE bit

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return ret;
  }

  if (HAL_QSPI_Transmit(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return ret;
  }

  /* Read Volatile Configuration register 3 --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG3_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  if ((ret = HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  /* Write Volatile Configuration register 2 (DRV1:2 = 00) -- */
  sCommand.Instruction = WRITE_STATUS_REG3_CMD;
  LOGI("REG3: %02x", reg);
  reg &= 0x9f; // DRV1:2 bit

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return ret;
  }

  if (HAL_QSPI_Transmit(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return ret;
  }

  return HAL_OK;
}

uint8_t CSP_QSPI_EraseBlock(uint32_t flash_address) {
  QSPI_CommandTypeDef sCommand = {0};
  HAL_StatusTypeDef   ret;

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations ------------------------------------------- */
  if ((ret = QSPI_WriteEnable()) != HAL_OK) {
    return ret;
  }

  /* Erasing Sequence -------------------------------------------------- */
  sCommand.Instruction = BLOCK_ERASE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.Address     = flash_address;
  sCommand.DataMode    = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  if (QSPI_AutoPollingMemReady() != HAL_OK) {
    LOGE("AP fail");
  }

  return HAL_OK;
}

uint8_t CSP_QSPI_EraseSector(uint32_t EraseStartAddress) {

  QSPI_CommandTypeDef sCommand;

  EraseStartAddress          = EraseStartAddress - EraseStartAddress % MEMORY_SECTOR_SIZE;

  /* Erasing Sequence -------------------------------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = SECTOR_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;

  sCommand.Address           = (EraseStartAddress & 0x0FFFFFFF);

  if (QSPI_WriteEnable() != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }
  EraseStartAddress += MEMORY_SECTOR_SIZE;

  if (QSPI_AutoPollingMemReady() != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

int32_t qspi_read_memory(uint32_t addr, void *data, uint32_t len) {
  uint32_t error = 0;
  if (CSP_QSPI_Read(data, addr, len) == HAL_OK)
    error = len;
#if WITH_QSPI_DEBUG
  else {
    LOGE("QSPI Read Error");
  }
#endif

  return error;
}
int32_t qspi_write_memory(uint32_t addr, void *data, uint32_t len) {
  uint32_t error = 0;
  if (CSP_QSPI_WriteMemory(data, addr, len) == HAL_OK)
    error = len;
#if WITH_QSPI_DEBUG
  else {
    LOGE("QSPI Write Error");
  }
#endif
  return len;
}

uint8_t CSP_QSPI_EraseSectors(uint32_t EraseStartAddress,
                              uint32_t EraseEndAddress) {

  QSPI_CommandTypeDef sCommand;

  EraseStartAddress          = EraseStartAddress - EraseStartAddress % MEMORY_SECTOR_SIZE;

  /* Erasing Sequence -------------------------------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = SECTOR_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;

  while (EraseEndAddress >= EraseStartAddress) {
    sCommand.Address = (EraseStartAddress & 0x0FFFFFFF);

    if (QSPI_WriteEnable() != HAL_OK) {
      return HAL_ERROR;
    }

    if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return HAL_ERROR;
    }
    EraseStartAddress += MEMORY_SECTOR_SIZE;

    if (QSPI_AutoPollingMemReady() != HAL_OK) {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

uint8_t CSP_QSPI_WriteMemory(const void *buffer, uint32_t address,
                             uint32_t buffer_size) {

  QSPI_CommandTypeDef sCommand;
  uint32_t            end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_addr = 0;

  //
  while (current_addr <= address) {
    current_addr += MEMORY_PAGE_SIZE;
  }
  current_size = current_addr - address;

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > buffer_size) {
    current_size = buffer_size;
  }

  /* Initialize the adress variables */
  current_addr               = address;
  end_addr                   = address + buffer_size;

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = QUAD_IN_FAST_PROG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.DataMode          = QSPI_DATA_4_LINES;
  sCommand.NbData            = buffer_size;
  sCommand.Address           = address;
  sCommand.DummyCycles       = 0;

  /* Perform the write page by page */
  do {
    sCommand.Address = current_addr;
    sCommand.NbData  = current_size;

    if (current_size == 0) {
      return HAL_OK;
    }

    /* Enable write operations */
    if (QSPI_WriteEnable() != HAL_OK) {
      return HAL_ERROR;
    }

    /* Configure the command */
    if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {

      return HAL_ERROR;
    }

    /* Transmission of the data */
    if (HAL_QSPI_Transmit(&hqspi, (void *)buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {

      return HAL_ERROR;
    }

    /* Configure automatic polling mode to wait for end of program */
    if (QSPI_AutoPollingMemReady() != HAL_OK) {
      return HAL_ERROR;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    buffer += current_size;
    current_size =
      ((current_addr + MEMORY_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MEMORY_PAGE_SIZE;
  } while (current_addr <= end_addr);

  return HAL_OK;
}

// uint8_t CSP_QSPI_EnableMemoryMappedMode(void) {
//
//	QSPI_CommandTypeDef sCommand;
//	QSPI_MemoryMappedTypeDef sMemMappedCfg;
//
//	/* Enable Memory-Mapped mode-------------------------------------------------- */
//
//	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
//	sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
//	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
//	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
//	sCommand.Address = 0;
//	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
//	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
//	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
//	sCommand.DataMode = QSPI_DATA_4_LINES;
//	sCommand.NbData = 0;
//	sCommand.DummyCycles = 4;
//	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	sCommand.AlternateBytes = 0;
//	sCommand.AlternateBytesSize = 0;
//
//	sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
//	sMemMappedCfg.TimeOutPeriod = 0;
//
//	if (HAL_QSPI_MemoryMapped(&hqspi, &sCommand, &sMemMappedCfg) != HAL_OK) {
//		return HAL_ERROR;
//	}
//	return HAL_OK;
// }

uint8_t CSP_QSPI_EnableMemoryMappedMode(void) {

  QSPI_CommandTypeDef      sCommand;
  QSPI_MemoryMappedTypeDef sMemMappedCfg;

  /* Enable Memory-Mapped mode-------------------------------------------------- */

  sCommand.InstructionMode        = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction            = QUAD_IN_OUT_FAST_READ_CMD;
  sCommand.AddressSize            = QSPI_ADDRESS_24_BITS;
  sCommand.AddressMode            = QSPI_ADDRESS_4_LINES;
  sCommand.Address                = 0;
  sCommand.AlternateByteMode      = QSPI_ALTERNATE_BYTES_4_LINES;
  sCommand.AlternateBytes         = 0xFF;
  sCommand.AlternateBytesSize     = 1;
  sCommand.DdrMode                = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle       = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode               = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.DataMode               = QSPI_DATA_4_LINES;
  sCommand.NbData                 = 0;
  sCommand.DummyCycles            = 4;

  sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  sMemMappedCfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(&hqspi, &sCommand, &sMemMappedCfg) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(100);
  return HAL_OK;
}

uint8_t QSPI_ResetChip(void) {
  QSPI_CommandTypeDef sCommand = {0};
  uint32_t            temp     = 0;
  HAL_StatusTypeDef   ret;

  /* Enable Reset --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  /* Reset Device --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_EXECUTE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if ((ret = HAL_QSPI_Command(&hqspi, &sCommand,
                              HAL_QPSI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
    return ret;
  }

  // for (temp = 0; temp < 500000; temp++) {
  // 	__NOP();
  // }

  return HAL_OK;
}

uint8_t CSP_QSPI_Read(void *pData, uint32_t ReadAddr, uint32_t Size) {
  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = QUAD_IN_OUT_FAST_READ_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 6U;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set S# timing for Read command */
  MODIFY_REG(hqspi.Instance->DCR, QUADSPI_DCR_CSHT,
             QSPI_CS_HIGH_TIME_5_CYCLE);

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Restore S# timing for nonRead commands */
  MODIFY_REG(hqspi.Instance->DCR, QUADSPI_DCR_CSHT,
             QSPI_CS_HIGH_TIME_6_CYCLE);

  return HAL_OK;
}

/**
 * @brief Initialises QSPI flash
 *
 * @return int  0 if FLASH already initialised
 *              1 if the routine erased and initialised flash
 *             -1 if initialisation failed
 */
int qspi_init_flash(void) {
  LOGI("QSPI Init");
  const uint32_t voct_chunk = MAKE_CHUNK_ID('V', 'O', 'C', 'T');
  uint32_t       cid        = 0;

  if (CSP_QUADSPI_Init() != HAL_OK)
    LOGE("QSPI Init fail");

  // LOGI("Erase");
  // if (CSP_QSPI_Erase_Chip() != HAL_OK)
  //   LOGE("QSPI Erase FAIL");
  // if (CSP_QSPI_EraseBlock(0) != HAL_OK)
  //   LOGE("QSPI Erase FAIL");
  // if (CSP_QSPI_EraseSector(0) != HAL_OK)
  //   LOGE("QSPI Erase FAIL");
  // LOGI("Erase Completed");

  LOGI("Read");
  if (CSP_QSPI_Read(&cid, 0, 4) != HAL_OK) {
    LOGE("QSPI Read Fail");
  } else {
    LOGI("CID: %C", cid);
  }

  if (CSP_QSPI_WriteMemory(&voct_chunk, 4, 4) != HAL_OK)
    LOGE("QSPI Write FAIL");

  if (cid == voct_chunk) {
    LOGI("FLASH INITIALISED");
    return 0;
  } else {
    LOGW("FLASH NOT INITIALISED");
    LOGI("Erase");
    if (CSP_QSPI_Erase_Chip() != HAL_OK) {
      LOGE("QSPI Erase FAIL");
      return -1;
    }
    // LOGI("Erase Completed");
    LOGI("Write");
    if (CSP_QSPI_WriteMemory(&voct_chunk, 0, 4) != HAL_OK) {
      LOGE("QSPI Write FAIL");
      return -1;
    }
  }
  return 1;
}