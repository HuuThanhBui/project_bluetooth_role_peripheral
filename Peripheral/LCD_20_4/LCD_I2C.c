#include "LCD_I2C.h"
#include "nrf_delay.h"

void Init_LCD_I2C(uint32_t scl_pin, uint32_t sda_pin, speed_of_i2c speed)
{
	i2c_init(scl_pin,sda_pin,speed);
}

void lcd_i2c_send_cmd(char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	i2c_write(ADDR_LCD_I2C, 0x00, data_t, 4);
}

void lcd_i2c_send_data(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	i2c_write(ADDR_LCD_I2C, 0x00, data_t, 4);
}

void lcd_i2c_clear(void)
{
	int i = 0;
	lcd_i2c_send_cmd(0x80);
	for(i=0; i<70; i++)
	{
		lcd_i2c_send_data(' ');
	}
}

void lcd_i2c_put_cur(int row, int col)
{
	switch (row)
	{
			case 0:
					col |= 0x80;
					break;
			case 1:
					col |= 0xC0;
					break;
			case 2:
					col |= 0x94;
					break;
			case 3:
					col |= 0xD4;
					break;
	}
	lcd_i2c_send_cmd(col);
}

void lcd_i2c_init(void)
{
	// 4 bit initialisation
	nrf_delay_ms(50);  // wait for >40ms
	lcd_i2c_send_cmd(0x30);
	nrf_delay_ms(5);  // wait for >4.1ms
	lcd_i2c_send_cmd(0x30);
	nrf_delay_ms(1);  // wait for >100us
	lcd_i2c_send_cmd(0x30);
	nrf_delay_ms(10);
	lcd_i2c_send_cmd(0x20);  // 4bit mode
	nrf_delay_ms(10);

  // dislay initialisation
	lcd_i2c_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	nrf_delay_ms(1);
	lcd_i2c_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	nrf_delay_ms(1);
	lcd_i2c_send_cmd(0x01);  // clear display
	nrf_delay_ms(1);
	nrf_delay_ms(1);
	lcd_i2c_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	nrf_delay_ms(1);
	lcd_i2c_send_cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_i2c_send_string(char *str)
{
	while(*str) 
	{
		lcd_i2c_send_data(*str++);
	}
}
















