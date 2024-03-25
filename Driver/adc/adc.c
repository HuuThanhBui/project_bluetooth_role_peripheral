#include "adc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

void adc_ref_grou_init(channel_adc channel_adc_t,void(*func)(nrf_drv_saadc_evt_t const *p_event))
{
	ret_code_t err_code;
	if(channel_adc_t == AN0)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(0,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN1)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(1,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN2)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(2,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN3)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(3,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN4)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(4,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN5)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(5,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN6)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(6,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN7)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(7,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
}

static nrf_saadc_input_t ref_channel_handler(channel_adc channel_ref)
{
	if(channel_ref == AN0){return NRF_SAADC_INPUT_AIN0;}
	else if(channel_ref == AN1){return NRF_SAADC_INPUT_AIN1;}
	else if(channel_ref == AN2){return NRF_SAADC_INPUT_AIN2;}
	else if(channel_ref == AN3){return NRF_SAADC_INPUT_AIN3;}
	else if(channel_ref == AN4){return NRF_SAADC_INPUT_AIN4;}
	else if(channel_ref == AN5){return NRF_SAADC_INPUT_AIN5;}
	else if(channel_ref == AN6){return NRF_SAADC_INPUT_AIN6;}
	else if(channel_ref == AN7){return NRF_SAADC_INPUT_AIN7;}
	else if(channel_ref == VDD){return NRF_SAADC_INPUT_VDD;}
	return (nrf_saadc_input_t)0;
}

void adc_ref_input_init(channel_adc channel_adc_t,channel_adc channel_ref,void(*func)(nrf_drv_saadc_evt_t const *p_event))
{
	ret_code_t err_code;
	if(channel_adc_t == AN0)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN0);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(0,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN1)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN1);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(1,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN2)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN2);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(2,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN3)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN3);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(3,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN4)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN4);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(4,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN5)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN5);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(5,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN6)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN6);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(6,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
	else if(channel_adc_t == AN7)
	{
		nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(ref_channel_handler(channel_ref), NRF_SAADC_INPUT_AIN7);
		err_code = nrf_drv_saadc_init(NULL,func);
		APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_channel_init(7,&channel_config);
		APP_ERROR_CHECK(err_code);
	}
}

uint16_t read_adc(channel_adc channel_adc_t)
{
	nrf_saadc_value_t adc_val;
	
	if(channel_adc_t == AN0){nrfx_saadc_sample_convert(0,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN1){nrfx_saadc_sample_convert(1,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN2){nrfx_saadc_sample_convert(2,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN3){nrfx_saadc_sample_convert(3,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN4){nrfx_saadc_sample_convert(4,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN5){nrfx_saadc_sample_convert(5,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN6){nrfx_saadc_sample_convert(6,&adc_val);return (uint16_t)adc_val;}
	else if(channel_adc_t == AN7){nrfx_saadc_sample_convert(7,&adc_val);return (uint16_t)adc_val;}
	return 0;
}
