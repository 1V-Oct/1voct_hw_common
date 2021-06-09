#pragma once

#include "app_config.h"

void flash_unlock_and_erase(void);
void flash_lock(void);
uint16_t flash_readh(uint32_t address);
HAL_StatusTypeDef flash_writeh(uint32_t address, uint16_t data);
