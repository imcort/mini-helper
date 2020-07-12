#ifndef __IIC_HANDLER_H_
#define __IIC_HANDLER_H_

#include <stdint.h>

#define BOARD_SDA_PIN 7
#define BOARD_SCL_PIN 8

void twi_init(void);
void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
void twi_readRegisters_stop(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
void twi_readRegisters_delay(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len, int delay);
void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);


#endif
