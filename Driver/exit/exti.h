#ifndef _EXTI_H_
#define _EXTI_H_

#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"

typedef enum{
	hi_to_lo,
	lo_to_hi,
	toggle
}trig_sence;

typedef enum{
	pull_up,
	pull_down,
	no_pull
}pull_res;

void init_exti(uint32_t pin_trig,trig_sence trig_sence_t,pull_res pull_res_t, void(*func)(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action));
void enable_exti(uint32_t pin_trig);
void disable_exti(uint32_t pin_trig);

#endif

















