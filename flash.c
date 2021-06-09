#include "flash.h"

#define CABLE_NAME_FLASH_START_ADDRESS 0
/**
 * @brief Unprotects FLASH. Must be called before any write operation
 * 
 */

void flash_unlock_and_erase(void)
{
	uint32_t PAGEError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.PageAddress = CABLE_NAME_FLASH_START_ADDRESS;
	EraseInitStruct.NbPages = 1;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		//Erase error!
	}

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPTVERR | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
}

/**
 * @brief Turn on flash write protection
 * 
 */

void flash_lock(void) {
  HAL_FLASH_Lock();
}

/**
 * @brief FLASH read (16 bit)
 */
uint16_t flash_readh(uint32_t address)
{
	uint16_t val = 0;

  address = address * 2 + CABLE_NAME_FLASH_START_ADDRESS;

  val = *(__IO uint16_t*)address;

  return val;
}

/**
 * @brief Writes FLASH half word
 */
HAL_StatusTypeDef flash_writeh(uint32_t address, uint16_t data)
{
	HAL_StatusTypeDef status;

  address = address * 2 + CABLE_NAME_FLASH_START_ADDRESS;

  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data);

  while(!__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) == RESET);

  return status;
}
