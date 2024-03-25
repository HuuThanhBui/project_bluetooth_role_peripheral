#include "i2c.h"
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"

#define	TWI_INSTANCE_ID			0

static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
        //If data transmission or receiving is finished
			case NRF_DRV_TWI_EVT_DONE:
        m_xfer_done = true;//Set the flag
        break;
        
        default:
        // do nothing
          break;
    }
}

void i2c_init(uint32_t scl_pin, uint32_t sda_pin, speed_of_i2c speed)
{
	static uint8_t entry_pass = 0;
	ret_code_t err_code;
	nrf_drv_twi_frequency_t temp_frequen;
	
	if(entry_pass == 1)
	{
		return;
	}
	entry_pass = 1;
	
	if(speed == stand)
	{
		temp_frequen = NRF_DRV_TWI_FREQ_100K;
	}
	else if(speed == high)
	{
		temp_frequen = NRF_DRV_TWI_FREQ_250K;
	}
	else if(speed == super_high)
	{
		temp_frequen = NRF_DRV_TWI_FREQ_400K;
	}

	const nrf_drv_twi_config_t twi_config = {
		 .scl                = scl_pin,  //SCL Pin
		 .sda                = sda_pin,  //SDA Pin
		 .frequency          = temp_frequen, //Communication Speed
		 .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
		 .clear_bus_init     = false //automatically clear bus
	};


	//A function to initialize the twi communication
	err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);
	if(err_code != NRF_SUCCESS)
	{
		return;
	}
	
	//Enable the TWI Communication
	nrf_drv_twi_enable(&m_twi);
}
uint8_t tx_buf[10];
void i2c_write(uint8_t address_slave, uint8_t address_reg, uint8_t *value, uint8_t numBytes)
{
	memset(tx_buf,0,sizeof(tx_buf));
	ret_code_t err_code;
	tx_buf[0] = address_reg;
	for(uint8_t i = 1; i <= numBytes; i++)
	{
		tx_buf[i] = value[i-1];
	}
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, address_slave, tx_buf, numBytes+1, false);
	while(m_xfer_done == false);
	if (NRF_SUCCESS != err_code)
	{
			return;
	}
	return;
}

void i2c_read(uint8_t address_slave, uint8_t address_reg, uint8_t *value, uint8_t numBytes)
{
	ret_code_t err_code;
	m_xfer_done = false;
	
	err_code = nrf_drv_twi_tx(&m_twi, address_slave, &address_reg, 1, true);
	while(m_xfer_done == false);
	if (NRF_SUCCESS != err_code)
	{
			return;
	}
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, address_slave, value, numBytes);
	while(m_xfer_done == false);
	if (NRF_SUCCESS != err_code)
	{
			return;
	}
}


