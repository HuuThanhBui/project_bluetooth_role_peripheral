#ifndef _ADC_H_
#define _ADC_H_

#include "stdint.h"
#include "nrf_drv_saadc.h"

typedef enum{
	AN0,
	AN1,
	AN2,
	AN3,
	AN4,
	AN5,
	AN6,
	AN7,
	VDD
}channel_adc;

void adc_ref_grou_init(channel_adc channel_adc_t,void(*func)(nrf_drv_saadc_evt_t const *p_event));
void adc_ref_input_init(channel_adc channel_adc_t,channel_adc channel_ref,void(*func)(nrf_drv_saadc_evt_t const *p_event));
uint16_t read_adc(channel_adc channel_adc_t);

#endif

