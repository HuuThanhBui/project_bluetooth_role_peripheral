#include "dht11.h"
#include "onewire.h"
#include "nrf_delay.h"
#include "dht11.h"
#include "nrf_gpio.h"

#define DATA_PIN_DHT11 24

void DHT11_Init(void)
{
	nrf_gpio_pin_dir_set(DATA_PIN_DHT11,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(DATA_PIN_DHT11, 1);
}

void Start_DHT11(void)
{
	nrf_gpio_pin_dir_set(DATA_PIN_DHT11,NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(DATA_PIN_DHT11, 0);
	nrf_delay_ms(20);
	nrf_gpio_pin_write(DATA_PIN_DHT11, 1);
	nrf_delay_us(30);
}

uint8_t check_dht11(void)
{
	uint32_t time = 0;
	nrf_gpio_cfg_input(DATA_PIN_DHT11, NRF_GPIO_PIN_NOPULL);
	while(nrf_gpio_pin_read(DATA_PIN_DHT11) && time < 100)
	{
		time++;
		nrf_delay_us(1);
	}
	if(time >= 100)
	{
		return 1;
	}
	else{
		time = 0;
	}
	
	while(!nrf_gpio_pin_read(DATA_PIN_DHT11) && time < 100)
	{
		time++;
		nrf_delay_us(1);
	}
	if(time >= 100)
	{
		return 1;
	}
	else{
		return 0;
	}
}

uint8_t read_1_bit_data(void)
{
	uint32_t time = 0;
	while(nrf_gpio_pin_read(DATA_PIN_DHT11) && time < 100)
	{
		time++;
		nrf_delay_us(1);
	}
	while(!nrf_gpio_pin_read(DATA_PIN_DHT11) && time < 100)
	{
		time++;
		nrf_delay_us(1);
	}
	nrf_delay_us(40);
	if(nrf_gpio_pin_read(DATA_PIN_DHT11))
	{
		return 1;
	}
	else{
		return 0;
	}
}

uint8_t read_1_byte_data(void)
{
	uint8_t i = 0;
	uint8_t data = 0;
	
	for(i = 0; i < 8; i++)
	{
		data <<= 1;
		data |= read_1_bit_data();
	}
	return data;
}

uint8_t read_5_byte_data(uint8_t *temp, uint8_t *humi)
{
	uint8_t i = 0;
	uint8_t buf[5];
	
	
	Start_DHT11();
	
	if(check_dht11() == 0)
	{
		for(i = 0; i < 5; i++)
		{
			buf[i] = read_1_byte_data();
		}
		if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
		{
			*temp = buf[2];
			*humi = buf[0];
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

