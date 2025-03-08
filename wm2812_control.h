#pragma once
#include "app_config.h"

#if WITH_WM2812_CONTROL

void wm2812_init(void);
void wm2812_process(void);
void wm2812_set_status_color(uint8_t idx, uint32_t rgb, uint32_t rgb_alt = 0xffffffff);

#define COL_RED        0xff0000
#define COL_GREEN      0x00ff00
#define COL_BLUE       0x0000ff
#define COL_WHITE      0xffffff
#define COL_YELLOW     0xffff00
#define COL_ORANGE     0xffa500
#define COL_CYAN       0x00ffff
#define COL_MAGENTA    0xff00ff
#define COL_BLACK      0x000000
#define COL_GRAY       0x808080
#define COL_LIGHT_GRAY 0xd3d3d3
#define COL_DARK_GRAY  0xa9a9a9
#define COL_PINK       0xffc0cb
#define COL_PURPLE     0x800080
#define COL_BROWN      0xa52a2a
#define COL_OLIVE      0x808000
#define COL_MAROON     0x800000
#define COL_NAVY       0x000080
#define COL_TEAL       0x008080
#define COL_LIME       0x00ff00
#define COL_AQUA       0x00ffff
#define COL_FUCHSIA    0xff00ff
#define COL_SILVER     0xc0c0c0

#endif