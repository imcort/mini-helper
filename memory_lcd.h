#ifndef __MEMORY_LCD_H__
#define __MEMORY_LCD_H__

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"



bool disp_init(void);
void disp_clear(void);
void disp_refresh(uint8_t startLine, uint8_t* buffer);
void disp_show(void);
//void disp_draw(uint8_t x, uint8_t y, bool c);

#endif
