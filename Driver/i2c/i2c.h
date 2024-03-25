#ifndef _I2C_H_
#define _I2C_H_

#include "stdint.h"
#include "stdbool.h"

typedef enum{
	stand,
	high,
	super_high
}speed_of_i2c;

void i2c_init(uint32_t scl_pin, uint32_t sda_pin, speed_of_i2c speed);
void i2c_write(uint8_t address_slave, uint8_t address_reg, uint8_t *value, uint8_t numBytes);
void i2c_read(uint8_t address_slave, uint8_t address_reg, uint8_t *value, uint8_t numBytes);

#endif


