#ifndef __IIC_HANDLER_H_
#define __IIC_HANDLER_H_

#include <stdint.h>

#define BOARD_SDA_PIN 16
#define BOARD_SCL_PIN 15

void twi_init(void);
void twi_scanner(void);

void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
void twi_readRegisters_stop(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
void twi_readRegisters_delay(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len, int delay);
void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);

//VL53L1X

void twi_vl53l1x_write(uint8_t addr, uint16_t index, uint8_t data);
void twi_vl53l1x_read(uint8_t addr, uint16_t index, uint8_t *data);
void twi_vl53l1x_seq_write(uint8_t addr, uint16_t index, uint8_t *data, uint16_t length);
void twi_vl53l1x_seq_read(uint8_t addr, uint16_t index, uint8_t *data, uint16_t length);


#endif
