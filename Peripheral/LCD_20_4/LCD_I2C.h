#ifndef _LCD_I2C_H_
#define _LCD_I2C_H_

#include "i2c.h"
#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "nrf_delay.h"

#define ADDR_LCD_I2C 0x27
//#define ADDR_LCD_I2C 0x3F

void Init_LCD_I2C(uint32_t scl_pin, uint32_t sda_pin, speed_of_i2c speed);
void lcd_i2c_send_cmd(char cmd);
void lcd_i2c_send_data(char data);
void lcd_i2c_clear(void);
void lcd_i2c_put_cur(int row, int col);
void lcd_i2c_init(void);
void lcd_i2c_send_string(char *str);

#endif


