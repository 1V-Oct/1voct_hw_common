#pragma once

#include "app_config.h"

__CEXTERN_START

void ili_init(void);
void ili_clear_screen(uint16_t col);
void ili_fill_rect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t col);
void ili_wait_scanline(uint16_t line);
void ili_blit_screen(uint16_t *ptr);
void ili_blit_wait(void);
void ili_blit_region(uint16_t *ptr, uint32_t y, uint32_t h);



__CEXTERN_END