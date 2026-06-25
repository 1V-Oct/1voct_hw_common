#pragma once
#include "app_config.h"

__CEXTERN_START

void crx_crash_handler(uint32_t *sp);
void crx_log(char c);

uint8_t crx_has_crashed(void);
void crx_enable_log(void);
void crx_save_crash_dump(void);

__CEXTERN_END