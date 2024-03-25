#include "exti.h"
#include "app_gpiote.h"

void init_exti(uint32_t pin_trig,trig_sence trig_sence_t,pull_res pull_res_t, void(*func)(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action))
{
	nrf_drv_gpiote_in_config_t in_config;
	ret_code_t err_code;
	
	if (!nrf_drv_gpiote_is_init())
	{
			int err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
	}
	
	if(trig_sence_t == hi_to_lo){in_config = (nrf_drv_gpiote_in_config_t)GPIOTE_CONFIG_IN_SENSE_HITOLO(true);}  //De false thi se tiet kiem nang luong hon
	else if(trig_sence_t == lo_to_hi){in_config = (nrf_drv_gpiote_in_config_t)GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);}
	else if(trig_sence_t == toggle){in_config = (nrf_drv_gpiote_in_config_t)GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);}
	
	if(pull_res_t == pull_up){in_config.pull = NRF_GPIO_PIN_PULLUP;}
	else if(pull_res_t == pull_down){in_config.pull = NRF_GPIO_PIN_PULLDOWN;}
	else if(pull_res_t == no_pull){in_config.pull = NRF_GPIO_PIN_NOPULL;}
	
	err_code = nrf_drv_gpiote_in_init(pin_trig,&in_config,func);
	APP_ERROR_CHECK(err_code);
}

void enable_exti(uint32_t pin_trig)
{
	nrf_drv_gpiote_in_event_enable(pin_trig,true);
	nrfx_gpiote_in_event_enable(pin_trig, true);
}

void disable_exti(uint32_t pin_trig)
{
	nrf_drv_gpiote_in_event_enable(pin_trig,false);
	nrfx_gpiote_in_event_disable(pin_trig);
}














