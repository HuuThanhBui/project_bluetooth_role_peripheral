#ifndef _DHT11_H_
#define _DHT11_H_

#include "stdint.h"
#include "string.h"
#include "stdio.h"

void DHT11_Init(void);
void Start_DHT11(void);
uint8_t check_dht11(void);
uint8_t read_1_bit_data(void);
uint8_t read_1_byte_data(void);
uint8_t read_5_byte_data(uint8_t *temp, uint8_t *humi);

#endif
