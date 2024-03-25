#include "lm75.h"
#include "i2c.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

double LM75_read_temp(void)
{
	uint8_t raw_data[2] = {0};
	uint16_t data = 0;
	double temp_value = 0.0;
	i2c_write(0x48, 0x00, raw_data, 2);
	nrf_delay_ms(200);
	i2c_read(0x48, 0x00, raw_data, 2);
	data = (uint16_t)(raw_data[0] << 8) | raw_data[1];
	temp_value = (double)data/256.0;
	NRF_LOG_INFO("Data LM75: %d",temp_value);
	return temp_value;
}
