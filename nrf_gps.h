#ifndef __NRF_GPS_H__
#define __NRF_GPS_H__

#include "nrf_uart.h"
#include "app_uart.h"
#include "minmea.h"
#include "nrf_gpio.h"

void gps_init(void);
void gps_on(void);
void gps_off(void);

int32_t gps_gettime(void);
bool gps_is_valid(void);

#endif
