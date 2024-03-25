#include "bh1750.h"
#include "i2c.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

double BH1750_read_lux(void)
{
	uint8_t raw_data[2] = {0};
	uint16_t data = 0;
	double light_value = 0.0;
	i2c_write(0x23, 0x10, raw_data, 0);
	nrf_delay_ms(200);
	i2c_read(0x23, 0x10, raw_data, 2);
	data = (uint16_t)(raw_data[0] << 8) | raw_data[1];
	light_value = (double)data/1.2;
	NRF_LOG_INFO("Data BH1750: %d",light_value);
	return light_value;
}
