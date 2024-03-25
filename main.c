/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "queue.h"
#include "Ringbuffer.h"
#include "nrf_drv_timer.h"
#include "nrf_timer.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "LCD_I2C.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Peripheral_Device"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                1024                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1024                                         /**< UART RX buffer size. */

#define MAX_MEM_HANDLER									50			//ThanhBH Add
#define SIZE_OF_QUEUE   								50			//ThanhBH Add

#define FRAME_SOF 			0xB1
#define FRAME_ACK 			0x06
#define FRAME_NACK 			0x15
#define CXOR_INIT_VAL 		0xFF

#define LENG_BUFF_SENSOR		7
#define LENG_BUFF_LED				6
#define LENG_PAYLOAD_SENSOR			9
#define LENG_PAYLOAD_LED				8

#define CMD_ID_LM75						0x00
#define CMD_ID_BH1750					0x01
#define CMD_ID_LED_1_ZONE_1		0x02
#define CMD_ID_LED_2_ZONE_1		0x03
#define CMD_ID_LED_3_ZONE_1		0x04
#define CMD_ID_LED_4_ZONE_1		0x05
#define CMD_ID_LED_5_ZONE_1		0x06
#define CMD_ID_LED_6_ZONE_1		0x07
#define CMD_ID_LED_7_ZONE_1		0x08
#define CMD_ID_LED_8_ZONE_1		0x09
#define CMD_ID_TEMP_DHT11			0x0A
#define CMD_ID_HUMI_DHT11			0x0B
#define CMD_ID_LM35						0x0C
#define CMD_ID_LED_1_ZONE_2		0x0D
#define CMD_ID_LED_2_ZONE_2		0x0E
#define CMD_ID_LED_3_ZONE_2		0x0F
#define CMD_ID_LED_4_ZONE_2		0x10
#define CMD_ID_LED_5_ZONE_2		0x11
#define CMD_ID_LED_6_ZONE_2		0x12
#define CMD_ID_LED_7_ZONE_2		0x13
#define CMD_ID_LED_8_ZONE_2		0x14
#define CMD_ID_PIR						0x15
#define CMD_ID_MQ8						0x16
#define CMD_ID_LED_1_ZONE_3		0x17
#define CMD_ID_LED_2_ZONE_3		0x18
#define CMD_ID_LED_3_ZONE_3		0x19
#define CMD_ID_LED_4_ZONE_3		0x1A
#define CMD_ID_LED_5_ZONE_3		0x1B
#define CMD_ID_LED_6_ZONE_3		0x1C
#define CMD_ID_LED_7_ZONE_3		0x1D
#define CMD_ID_LED_8_ZONE_3		0x1E

#define MAX_NUM_LED						9
#define MAX_NUM_ZONE					4
#define MAX_NUM_BIT_DATA_LED	2

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/***************************************** ThanhBH Timer ***********************************/
#include "nrf_drv_clock.h"

APP_TIMER_DEF(timer_send_form_peripheral_to_central_id);
APP_TIMER_DEF(timer_manage_display_lcd_id);
APP_TIMER_DEF(timer_update_status_BLE_device_id);
static void timer_send_form_peripheral_to_central_handler(void * p_context);
static void timer_manage_display_lcd_handler(void * p_context);
static void timer_update_status_BLE_device_handler(void * p_context);

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void create_timers(void)
{
    ret_code_t err_code;
	
		err_code = app_timer_create(&timer_send_form_peripheral_to_central_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_send_form_peripheral_to_central_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&timer_manage_display_lcd_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_manage_display_lcd_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&timer_update_status_BLE_device_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_update_status_BLE_device_handler);
    APP_ERROR_CHECK(err_code);
}

/*************************************************** ThanhBH Add ****************************************************/

static RingBuff ringbuff;
typedef struct mem_handler_connect mem_handler_connect_t;
uint8_t arr_mem_data[SIZE_OF_QUEUE] = {0};
uint8_t byRxBuffer2[SIZE_OF_QUEUE] = {0};
struct mem_handler_connect{
	uint16_t handler_connect;
	uint8_t status;
};

typedef struct {
	uint8_t cmdid;
	uint8_t type;
}frm_common_t;

mem_handler_connect_t mem_handler[MAX_MEM_HANDLER];

typedef enum{
	LM75,
	BH1750,
	TEMP_DHT11,
	HUMI_DHT11,
	LM35,
	LED,
	MQ7,
	PIR
}type_device_t;

typedef enum{
	GET,
	SET
}action_t;

typedef enum{
	LED_NUM_1 = 1,
	LED_NUM_2,
	LED_NUM_3,
	LED_NUM_4,
	LED_NUM_5,
	LED_NUM_6,
	LED_NUM_7,
	LED_NUM_8
}name_led_t;

typedef enum{
	ZONE_NUM_1 = 1,
	ZONE_NUM_2,
	ZONE_NUM_3
}name_zone_t;

typedef struct data_struct_zone_1 data_struct_zone_1_t;
struct data_struct_zone_1{
	uint16_t data_lm75;
	uint8_t byte_1_lm75;
	uint8_t byte_2_lm75;
	uint16_t data_bh1750;
	uint8_t byte_1_bh1750;
	uint8_t byte_2_bh1750;
	uint8_t data_led[8];
};

typedef struct data_struct_zone_2 data_struct_zone_2_t;
struct data_struct_zone_2{
	uint16_t data_temp_dht11;
	uint8_t byte_1_temp_dht11;
	uint8_t byte_2_temp_dht11;
	uint16_t data_humi_dht11;
	uint8_t byte_1_humi_dht11;
	uint8_t byte_2_humi_dht11;
	uint16_t data_lm35;
	uint8_t byte_1_lm35;
	uint8_t byte_2_lm35;
	uint8_t data_led[8];
};

typedef struct data_struct_zone_3 data_struct_zone_3_t;
struct data_struct_zone_3{
	uint16_t data_pir;
	uint8_t byte_1_pir;
	uint8_t byte_2_pir;
	uint16_t data_mq7;
	uint8_t byte_1_mq7;
	uint8_t byte_2_mq7;
	uint8_t data_led[8];
};
data_struct_zone_1_t data_zone_1;
data_struct_zone_2_t data_zone_2;
data_struct_zone_3_t data_zone_3;

void reset_struct_data_3_zone(data_struct_zone_1_t *data_zone_1_par, data_struct_zone_2_t *data_zone_2_par, data_struct_zone_3_t *data_zone_3_par)
{
	data_zone_1_par->data_bh1750 = 0;
	data_zone_1_par->data_lm75 = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		data_zone_1_par->data_led[i] = 0;
	}
	
	data_zone_2_par->data_humi_dht11 = 0;
	data_zone_2_par->data_lm35 = 0;
	data_zone_2_par->data_temp_dht11 = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		data_zone_2_par->data_led[i] = 0;
	}
	
	data_zone_3_par->data_mq7 = 0;
	data_zone_3_par->data_pir = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		data_zone_3_par->data_led[i] = 0;
	}
}

uint8_t table_convert_to_command_id(uint8_t number_led, uint8_t number_zone)
{
	name_led_t name_led;
	name_zone_t name_zone;
	if(number_led == 1){name_led = LED_NUM_1;}
	else if(number_led == 2){name_led = LED_NUM_2;}
	else if(number_led == 3){name_led = LED_NUM_3;}
	else if(number_led == 4){name_led = LED_NUM_4;}
	else if(number_led == 5){name_led = LED_NUM_5;}
	else if(number_led == 6){name_led = LED_NUM_6;}
	else if(number_led == 7){name_led = LED_NUM_7;}
	else if(number_led == 8){name_led = LED_NUM_8;}
	
	if(number_zone == 1){name_zone = ZONE_NUM_1;}
	else if(number_zone == 2){name_zone = ZONE_NUM_2;}
	else if(number_zone == 3){name_zone = ZONE_NUM_3;}
	
	switch(name_zone)
	{
		case ZONE_NUM_1:
		{
			if(name_led == LED_NUM_1) {return 0x02;}
			else if(name_led == LED_NUM_2) {return 0x03;}
			else if(name_led == LED_NUM_3) {return 0x04;}
			else if(name_led == LED_NUM_4) {return 0x05;}
			else if(name_led == LED_NUM_5) {return 0x06;}
			else if(name_led == LED_NUM_6) {return 0x07;}
			else if(name_led == LED_NUM_7) {return 0x08;}
			else if(name_led == LED_NUM_8) {return 0x09;}
			break;
		}
		case ZONE_NUM_2:
		{
			if(name_led == LED_NUM_1) {return 0x0D;}
			else if(name_led == LED_NUM_2) {return 0x0E;}
			else if(name_led == LED_NUM_3) {return 0x0F;}
			else if(name_led == LED_NUM_4) {return 0x10;}
			else if(name_led == LED_NUM_5) {return 0x11;}
			else if(name_led == LED_NUM_6) {return 0x12;}
			else if(name_led == LED_NUM_7) {return 0x13;}
			else if(name_led == LED_NUM_8) {return 0x14;}
			break;
		}
		case ZONE_NUM_3:
		{
			if(name_led == LED_NUM_1) {return 0x17;}
			else if(name_led == LED_NUM_2) {return 0x18;}
			else if(name_led == LED_NUM_3) {return 0x19;}
			else if(name_led == LED_NUM_4) {return 0x1A;}
			else if(name_led == LED_NUM_5) {return 0x1B;}
			else if(name_led == LED_NUM_6) {return 0x1C;}
			else if(name_led == LED_NUM_7) {return 0x1D;}
			else if(name_led == LED_NUM_8) {return 0x1E;}
			break;
		}
	}
	return 0xFF;
}

void calcu_data_to_send(type_device_t device, action_t action, uint8_t command_id, uint8_t option, uint8_t sequen,uint8_t data_to_send[], uint8_t leng_of_data, uint8_t data[])
{
	uint8_t result_xor = 0;
	if(device == LED)
	{
		if(action == SET)
		{
			result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
			for(uint32_t i = 0; i < leng_of_data; i++)
			{
				result_xor = result_xor ^ data_to_send[i];
			}
			data[0] = FRAME_SOF; data[1] = LENG_BUFF_LED; data[2] = option; data[3] = command_id; data[4] = 0x01;
			data[5] = data_to_send[0]; data[6] = sequen; data[7] = result_xor;
		}
		else if(action == GET)
		{
			result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x02 ^ sequen);
			for(uint32_t i = 0; i < leng_of_data; i++)
			{
				result_xor = result_xor ^ data_to_send[i];
			}
			data[0] = FRAME_SOF; data[1] = LENG_BUFF_LED; data[2] = option; data[3] = command_id; data[4] = 0x02;
			data[5] = data_to_send[0]; data[6] = sequen; data[7] = result_xor;
		}
	}
	else if(device == LM75)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == BH1750)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == TEMP_DHT11)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == HUMI_DHT11)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == LM35)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == MQ7)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == PIR)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
}

void add_handler(uint16_t handler_ble_connect)
{
	for(uint16_t i = 0; i < MAX_MEM_HANDLER; i++)
	{
		if(mem_handler[i].handler_connect != handler_ble_connect && mem_handler[i].status == 0)
		{
			mem_handler[i].handler_connect = handler_ble_connect;
			mem_handler[i].status = 1;
			break;
		}
	}
}

void remove_handler(uint16_t handler_ble_connect)
{
	for(uint16_t i = 0; i < MAX_MEM_HANDLER; i++)
	{
		if(mem_handler[i].handler_connect == handler_ble_connect && mem_handler[i].status == 1)
		{
			mem_handler[i].handler_connect = 0;
			mem_handler[i].status = 0;
			break;
		}
	}
}

void reset_handler(void)
{
	for(uint16_t i = 0; i < MAX_MEM_HANDLER; i++)
	{
		mem_handler[i].handler_connect = 0;
		mem_handler[i].status = 0;
	}
}

static void advertising_start(void);

/******************************************************************************************************/

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            ring_buff_push(&ringbuff, p_evt->params.rx_data.p_data[i]);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
						advertising_start();			//ThanhBH Add
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
					NRF_LOG_INFO("Connected with handler connect: %d",p_ble_evt->evt.gap_evt.conn_handle);
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
						add_handler(m_conn_handle);			//ThanhBH Add
						advertising_start();			//ThanhBH Add
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected with handler connect: %d",p_ble_evt->evt.gap_evt.conn_handle);
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						remove_handler(p_ble_evt->evt.gap_evt.conn_handle);			//ThanhBH Add
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
		uint16_t length = LENG_PAYLOAD_LED;			//LENG_PAYLOAD_SENSOR
		uint8_t data_send[10] = {0};
		static uint8_t data_led[] = {0x01, 0x00};
		int num_led = 0, num_zone = 0, led_data = 0;
		char arr_temp[10] = {0};
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
				{
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1 && index < 10)
                {
									strncpy(arr_temp,(char*)data_array, index);
									sscanf(arr_temp,"%d %d %d",&num_led,&num_zone,&led_data);
									if(num_led <= MAX_NUM_LED && num_zone <= MAX_NUM_ZONE & led_data <= MAX_NUM_BIT_DATA_LED)
									{
										NRF_LOG_INFO("Data Send To Central: %d  %d  %d\n",num_led,num_zone,led_data);
										data_led[0] = led_data;
										for(uint16_t i = 0; i < MAX_MEM_HANDLER; i++)
										{
											if(mem_handler[i].status == 1)
											{
													calcu_data_to_send(LED, SET, table_convert_to_command_id(num_led, num_zone), num_zone, 0, data_led, 1, data_send);
													err_code = ble_nus_data_send(&m_nus, data_send, &length, mem_handler[i].handler_connect);
													if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
													{
															APP_ERROR_CHECK(err_code);
													}
											}
										}
									}
                }

                index = 0;
            }
            break;
					}

        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/********************************************* ThanhBH Add **************************************/

typedef enum{
	UART_STATE_IDLE,
	RX_STATE_START_BYTE,
	RX_STATE_DATA_BYTES,
	UART_STATE_ACK_RECEIVED,
	UART_STATE_NACK_RECEIVED,
	UART_STATE_ERROR,
	RX_STATE_CXOR_BYTE,
	UART_STATE_DATA_RECEIVED,
	UART_STATE_RX_TIMEOUT
}status_revice_data;

static uint8_t byRxBufState = RX_STATE_START_BYTE;
static uint8_t byIndexRxBuf = 0;
static uint8_t byCheckXorRxBuf = 0;

uint8_t PollRxBuff(void)
{
	uint8_t byRxData;
	uint8_t byUartState = (uint8_t)UART_STATE_IDLE;
	while((ring_buff_available(&ringbuff) != 0) && (byUartState == UART_STATE_IDLE))
	{
		ring_buff_pop(&ringbuff, &byRxData);
//		NRF_LOG_INFO("Data recive PollRxBuff: %d",byRxData);
		switch(byRxBufState)
		{
			case RX_STATE_START_BYTE:
			{
				if(byRxData == FRAME_SOF)
				{
					byIndexRxBuf = 0;
					byCheckXorRxBuf = CXOR_INIT_VAL;
					byRxBufState = RX_STATE_DATA_BYTES;
				}
				else if(byRxData == FRAME_ACK)
				{
					byUartState = UART_STATE_ACK_RECEIVED;
				}
				else if(byRxData == FRAME_NACK)
				{
					byUartState = UART_STATE_NACK_RECEIVED;
				}
				else
				{
					byUartState = UART_STATE_ERROR;
				}
				break;
			}
			case RX_STATE_DATA_BYTES:
			{
				if(byIndexRxBuf < 254)
				{
					byRxBuffer2[byIndexRxBuf] = byRxData;
					if(byIndexRxBuf > 0)
					{
						byCheckXorRxBuf ^= byRxData;
					}
					if(++byIndexRxBuf == *byRxBuffer2)
					{
						byRxBufState = RX_STATE_CXOR_BYTE;
					}
				}
				else
				{
					byRxBufState = RX_STATE_START_BYTE;
					byUartState = UART_STATE_ERROR;
				}
				break;
			}
			case RX_STATE_CXOR_BYTE:
			{
				if(byRxData == byCheckXorRxBuf)
				{
					byUartState = UART_STATE_DATA_RECEIVED;
				}
				else
				{
					byUartState = UART_STATE_ERROR;
				}
			}
			default:
				byRxBufState = RX_STATE_START_BYTE;
				break;
		}
	}
	return byUartState;
}

void UartCommandProcess(void *arg)
{
	uint16_t real_data = 0;
	frm_common_t *pCmd = (frm_common_t *)arg;
	switch(pCmd->cmdid)
	{
		case CMD_ID_LM75:
		{
//			printf("\nLM75 Need handler !\n");
//			NRF_LOG_INFO("Raw Data LM75: %d  --  %d",byRxBuffer2[4], byRxBuffer2[5]);
			data_zone_1.byte_1_lm75 = byRxBuffer2[4];
			data_zone_1.byte_2_lm75 = byRxBuffer2[5];
			if(byRxBuffer2[5] != 0)
			{
				real_data = byRxBuffer2[4] * 10 + byRxBuffer2[5] * 1;
			}
			else
			{
				real_data = byRxBuffer2[4];
			}
			NRF_LOG_INFO("Data LM75: %d",real_data);
			data_zone_1.data_lm75 = real_data;
			break;
		}
		case CMD_ID_BH1750:
		{
//			printf("\nBH1750 Need handler !\n");
			NRF_LOG_INFO("Raw Data BH1750: %d  --  %d",byRxBuffer2[4], byRxBuffer2[5]);
			data_zone_1.byte_1_bh1750 = byRxBuffer2[4];
			data_zone_1.byte_2_bh1750 = byRxBuffer2[5];
			if(byRxBuffer2[5] != 0)
			{
				real_data = byRxBuffer2[4] * 10 + byRxBuffer2[5] * 1;
			}
			else
			{
				real_data = byRxBuffer2[4];
			}
			NRF_LOG_INFO("Data BH1750: %d",real_data);
			data_zone_1.data_bh1750 = real_data;
			break;
		}
		case CMD_ID_LED_1_ZONE_1:
		{
//			printf("\nLED_1_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_1_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[0] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_2_ZONE_1:
		{
//			printf("\nLED_2_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_2_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[1] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_3_ZONE_1:
		{
//			printf("\nLED_3_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_3_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[2] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_4_ZONE_1:
		{
//			printf("\nLED_4_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_4_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[3] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_5_ZONE_1:
		{
//			printf("\nLED_5_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_5_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[4] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_6_ZONE_1:
		{
//			printf("\nLED_6_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_6_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[5] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_7_ZONE_1:
		{
//			printf("\nLED_7_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_7_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[6] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_8_ZONE_1:
		{
//			printf("\nLED_8_ZONE_1 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_8_ZONE_1: %d",byRxBuffer2[4]);
			data_zone_1.data_led[7] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_TEMP_DHT11:
		{
//			printf("\nTEMP_DHT11 Need handler !\n");
			data_zone_2.byte_1_temp_dht11 = byRxBuffer2[4];
			data_zone_2.byte_2_temp_dht11 = byRxBuffer2[5];
			if(byRxBuffer2[5] != 0)
			{
				real_data = byRxBuffer2[4] * 10 + byRxBuffer2[5] * 1;
			}
			else
			{
				real_data = byRxBuffer2[4];
			}
			NRF_LOG_INFO("Data TEMP_DHT11: %d",real_data);
			data_zone_2.data_temp_dht11 = real_data;
			break;
		}
		case CMD_ID_HUMI_DHT11:
		{
//			printf("\nHUMI_DHT11 Need handler !\n");
			data_zone_2.byte_1_humi_dht11 = byRxBuffer2[4];
			data_zone_2.byte_1_humi_dht11 = byRxBuffer2[5];
			if(byRxBuffer2[5] != 0)
			{
				real_data = byRxBuffer2[4] * 10 + byRxBuffer2[5] * 1;
			}
			else
			{
				real_data = byRxBuffer2[4];
			}
			NRF_LOG_INFO("Data HUMI_DHT11: %d",real_data);
			data_zone_2.data_humi_dht11 = real_data;
			break;
		}
		case CMD_ID_LM35:
		{
//			printf("\nLM35 Need handler !\n");
			data_zone_2.byte_1_lm35 = byRxBuffer2[4];
			data_zone_2.byte_2_lm35 = byRxBuffer2[5];
			if(byRxBuffer2[5] != 0)
			{
				real_data = byRxBuffer2[4] * 10 + byRxBuffer2[5] * 1;
			}
			else
			{
				real_data = byRxBuffer2[4];
			}
			NRF_LOG_INFO("Data LM35: %d",real_data);
			data_zone_2.data_lm35 = real_data;
			break;
		}
		case CMD_ID_LED_1_ZONE_2:
		{
//			printf("\nLED_1_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_1_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[0] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_2_ZONE_2:
		{
//			printf("\nLED_2_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_2_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[1] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_3_ZONE_2:
		{
//			printf("\nLED_3_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_3_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[2] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_4_ZONE_2:
		{
//			printf("\nLED_4_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_4_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[3] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_5_ZONE_2:
		{
//			printf("\nLED_5_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_5_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[4] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_6_ZONE_2:
		{
//			printf("\nLED_6_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_6_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[5] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_7_ZONE_2:
		{
//			printf("\nLED_7_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_7_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[6] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_8_ZONE_2:
		{
//			printf("\nLED_8_ZONE_2 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_8_ZONE_2: %d",byRxBuffer2[4]);
			data_zone_2.data_led[7] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_PIR:
		{
//			printf("\nPIR Need handler !\n");
			data_zone_3.byte_1_pir = byRxBuffer2[4];
			data_zone_3.byte_2_pir = byRxBuffer2[5];
			NRF_LOG_INFO("Raw Data PIR: %d  --  %d",byRxBuffer2[4], byRxBuffer2[5]);
			data_zone_3.data_pir = byRxBuffer2[4];
			break;
		}
		case CMD_ID_MQ8:
		{
//			printf("\nMQ8 Need handler !\n");
			data_zone_3.byte_1_mq7 = byRxBuffer2[4];
			data_zone_3.byte_2_mq7 = byRxBuffer2[5];
			NRF_LOG_INFO("Raw Data MQ8: %d  --  %d",byRxBuffer2[4], byRxBuffer2[5]);
			data_zone_3.data_mq7 = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_1_ZONE_3:
		{
//			printf("\nLED_1_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_1_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[0] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_2_ZONE_3:
		{
//			printf("\nLED_2_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_2_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[1] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_3_ZONE_3:
		{
//			printf("\nLED_3_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_3_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[2] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_4_ZONE_3:
		{
//			printf("\nLED_4_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_4_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[3] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_5_ZONE_3:
		{
//			printf("\nLED_5_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_5_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[4] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_6_ZONE_3:
		{
//			printf("\nLED_6_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_6_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[5] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_7_ZONE_3:
		{
//			printf("\nLED_7_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_7_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[6] = byRxBuffer2[4];
			break;
		}
		case CMD_ID_LED_8_ZONE_3:
		{
//			printf("\nLED_8_ZONE_3 Need handler !\n");
			NRF_LOG_INFO("Raw Data LED_8_ZONE_3: %d",byRxBuffer2[4]);
			data_zone_3.data_led[7] = byRxBuffer2[4];
			break;
		}
		default:
//			printf("\nData Error !!!\n");
			NRF_LOG_INFO("Data Error !!!");
			break;
	}
}

void processDataReceiver(void)
{
	uint8_t stateRx;
	stateRx = PollRxBuff();
	if(stateRx != UART_STATE_IDLE)
	{
		switch(stateRx)
		{
			case UART_STATE_ACK_RECEIVED:
			{
//				printf("UART_STATE_ACK_RECEIVED\r\n");
				break;
			}
			case UART_STATE_NACK_RECEIVED:
			{
//				printf("UART_STATE_NACK_RECEIVED\r\n");
				break;
			}
			case UART_STATE_DATA_RECEIVED:
			{
//				printf("UART_STATE_DATA_RECEIVED\r\n");
				UartCommandProcess(&byRxBuffer2[2]);
				break;
			}
			case UART_STATE_ERROR:
			case UART_STATE_RX_TIMEOUT:
			{
//				printf("UART_STATE_RX_TIMEOUT\r\n");
				break;
			}
			default:
				break;
		}
	}
}



void init_peripheral(void)
{
	
}

#define ZONE 2

#if (ZONE == 1)

static void timer_send_form_peripheral_to_central_handler(void * p_context)
{
//	ret_code_t err_code;
//	uint16_t length = LENG_PAYLOAD_LED;			//LENG_PAYLOAD_SENSOR
//	uint8_t data_send[10] = {0};
//	static uint8_t data_led[] = {0x01, 0x00};
//	
//	for(uint16_t i = 0; i < MAX_MEM_HANDLER; i++)
//	{
//		if(mem_handler[i].status == 1)
//		{
//			calcu_data_to_send(LED, SET, 0x02, 1, 0, data_led, 1, data_send);
//			err_code = ble_nus_data_send(&m_nus, data_send, &length, mem_handler[i].handler_connect);
//			if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
//			{
//					APP_ERROR_CHECK(err_code);
//			}
//		}
//	}
}

#elif  (ZONE == 2)

static void timer_send_form_peripheral_to_central_handler(void * p_context)
{
	
}

#elif  (ZONE == 3)

static void timer_send_form_peripheral_to_central_handler(void * p_context)
{
	
}

#endif

static void timer_manage_display_lcd_handler(void * p_context)
{
	char arr_send[20] = {0};
	typedef enum{
		ZONE_1,
		ZONE_2,
		ZONE_3
	}state_machine_zone;
	static state_machine_zone state_zone = ZONE_1;
	switch(state_zone)
	{
		case ZONE_1:
		{
			lcd_i2c_clear();
			lcd_i2c_put_cur(0,0);
			sprintf(arr_send,"%s:%d %s:%d TEMP:","LED1",data_zone_1.data_led[0],"LED2",data_zone_1.data_led[1]);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(1,0);
			sprintf(arr_send,"%s:%d %s:%d %d","LED3",data_zone_1.data_led[2],"LED4",data_zone_1.data_led[3],data_zone_1.data_lm75);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(2,0);
			sprintf(arr_send,"%s:%d %s:%d LUX:","LED5",data_zone_1.data_led[4],"LED6",data_zone_1.data_led[5]);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(3,0);
			sprintf(arr_send,"%s:%d %s:%d %d","LED7",data_zone_1.data_led[6],"LED8",data_zone_1.data_led[7],data_zone_1.data_bh1750);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			state_zone = ZONE_2;
			break;
		}
		case ZONE_2:
		{
			lcd_i2c_clear();
			lcd_i2c_put_cur(0,0);
			sprintf(arr_send,"%s:%d %s:%d TEMP:","LED1",data_zone_2.data_led[0],"LED2",data_zone_2.data_led[1]);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(1,0);
			sprintf(arr_send,"%s:%d %s:%d %d ","LED3",data_zone_2.data_led[2],"LED4",data_zone_2.data_led[3],data_zone_2.data_lm35);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(2,0);
			sprintf(arr_send,"%s:%d %s:%d HUMI:","LED5",data_zone_2.data_led[4],"LED6",data_zone_2.data_led[5]);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(3,0);
			sprintf(arr_send,"%s:%d %s:%d %d ","LED7",data_zone_2.data_led[6],"LED8",data_zone_2.data_led[7],data_zone_2.data_humi_dht11);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			state_zone = ZONE_3;
			break;
		}
		case ZONE_3:
		{
			lcd_i2c_clear();
			lcd_i2c_put_cur(0,0);
			sprintf(arr_send,"%s:%d %s:%d PIR:","LED1",data_zone_3.data_led[0],"LED2",data_zone_3.data_led[1]);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(1,0);
			sprintf(arr_send,"%s:%d %s:%d %d","LED3",data_zone_3.data_led[2],"LED4",data_zone_3.data_led[3],data_zone_3.data_pir);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(2,0);
			sprintf(arr_send,"%s:%d %s:%d MQ7:","LED5",data_zone_3.data_led[4],"LED6",data_zone_3.data_led[5]);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			
			lcd_i2c_put_cur(3,0);
			sprintf(arr_send,"%s:%d %s:%d %d   ","LED7",data_zone_3.data_led[6],"LED8",data_zone_3.data_led[7],data_zone_3.data_mq7);
			lcd_i2c_send_string(arr_send);
			memset(arr_send,0,sizeof(arr_send));
			state_zone = ZONE_1;
			break;
		}
	}
}

static void timer_update_status_BLE_device_handler(void * p_context)
{
	uint8_t data_send[13] = {0};
	uint8_t data_send_temp[500] = {0};
	static uint8_t data_led[] = {0x00, 0x00};
	static uint8_t data_sensor[] = {0x00, 0x00};
	uint16_t j = 0;

	typedef enum{
		ZONE_1,
		ZONE_2,
		ZONE_3
	}state_machine_zone;
	static state_machine_zone state_zone = ZONE_1;
	
//	NRF_LOG_INFO("\r\nUpdate data BLE !!!\r\n");
	
	data_send_temp[j] = 0x02;				//Type device 0x01: Zigbee, 0x02: BLE
	j++;
	
	/******************************* Zone 1 ***************************************/

	switch(state_zone)
	{
		case ZONE_1:
		{
			data_sensor[0] = data_zone_1.byte_1_lm75; data_sensor[1] = data_zone_1.byte_2_lm75;
			calcu_data_to_send(LM75, SET, CMD_ID_LM75, 1, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_sensor[0] = data_zone_1.byte_1_bh1750; data_sensor[1] = data_zone_1.byte_2_bh1750;
			calcu_data_to_send(BH1750, SET, CMD_ID_BH1750, 1, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');

			data_led[0] = data_zone_1.data_led[0];
			calcu_data_to_send(LED, SET, CMD_ID_LED_1_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[1];
			calcu_data_to_send(LED, SET, CMD_ID_LED_2_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[2];
			calcu_data_to_send(LED, SET, CMD_ID_LED_3_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[3];
			calcu_data_to_send(LED, SET, CMD_ID_LED_4_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[4];
			calcu_data_to_send(LED, SET, CMD_ID_LED_5_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[5];
			calcu_data_to_send(LED, SET, CMD_ID_LED_6_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[6];
			calcu_data_to_send(LED, SET, CMD_ID_LED_7_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_1.data_led[7];
			calcu_data_to_send(LED, SET, CMD_ID_LED_8_ZONE_1, 1, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');

			for(uint16_t i = 0; i < j; i++)
			{
				printf("%x ",data_send_temp[i]);
			}
			printf("\n");
			state_zone = ZONE_2;
			break;
		}
	
	/******************************* Zone 2 ***************************************/
		case ZONE_2:
		{
			data_sensor[0] = data_zone_2.byte_1_temp_dht11; data_sensor[1] = data_zone_2.byte_2_temp_dht11;
			calcu_data_to_send(TEMP_DHT11, SET, CMD_ID_TEMP_DHT11, 2, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_sensor[0] = data_zone_2.byte_1_humi_dht11; data_sensor[1] = data_zone_2.byte_2_humi_dht11;
			calcu_data_to_send(HUMI_DHT11, SET, CMD_ID_HUMI_DHT11, 2, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_sensor[0] = data_zone_2.byte_1_lm35; data_sensor[1] = data_zone_2.byte_2_lm35;
			calcu_data_to_send(LM35, SET, CMD_ID_LM35, 2, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[0];
			calcu_data_to_send(LED, SET, CMD_ID_LED_1_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[1];
			calcu_data_to_send(LED, SET, CMD_ID_LED_2_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[2];
			calcu_data_to_send(LED, SET, CMD_ID_LED_3_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[3];
			calcu_data_to_send(LED, SET, CMD_ID_LED_4_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[4];
			calcu_data_to_send(LED, SET, CMD_ID_LED_5_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[5];
			calcu_data_to_send(LED, SET, CMD_ID_LED_6_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[6];
			calcu_data_to_send(LED, SET, CMD_ID_LED_7_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_2.data_led[7];
			calcu_data_to_send(LED, SET, CMD_ID_LED_8_ZONE_2, 2, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			for(uint16_t i = 0; i < j; i++)
			{
				printf("%x ",data_send_temp[i]);
			}
			printf("\n");
			state_zone = ZONE_3;
			break;
		}
	
	/******************************* Zone 3 ***************************************/
		case ZONE_3:
		{
			data_sensor[0] = data_zone_3.byte_1_mq7; data_sensor[1] = data_zone_3.byte_2_mq7;
			calcu_data_to_send(MQ7, SET, CMD_ID_MQ8, 3, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_sensor[0] = data_zone_3.byte_1_pir; data_sensor[1] = data_zone_3.byte_2_pir;
			calcu_data_to_send(PIR, SET, CMD_ID_PIR, 3, 0, data_sensor, 2, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_SENSOR; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[0];
			calcu_data_to_send(LED, SET, CMD_ID_LED_1_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[1];
			calcu_data_to_send(LED, SET, CMD_ID_LED_2_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[2];
			calcu_data_to_send(LED, SET, CMD_ID_LED_3_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[3];
			calcu_data_to_send(LED, SET, CMD_ID_LED_4_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[4];
			calcu_data_to_send(LED, SET, CMD_ID_LED_5_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[5];
			calcu_data_to_send(LED, SET, CMD_ID_LED_6_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[6];
			calcu_data_to_send(LED, SET, CMD_ID_LED_7_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			// app_uart_put('\n');
			
			data_led[0] = data_zone_3.data_led[7];
			calcu_data_to_send(LED, SET, CMD_ID_LED_8_ZONE_3, 3, 0, data_led, 1, data_send);
			for(uint8_t i = 0; i < LENG_PAYLOAD_LED; i++)
			{
				// app_uart_put(data_send[i]);
				data_send_temp[j] = data_send[i];
				j++;
			}
			for(uint16_t i = 0; i < j; i++)
			{
				printf("%x ",data_send_temp[i]);
			}
			printf("\n");
			// app_uart_put('\n');
			state_zone = ZONE_1;
			break;
		}
	}
}

/************************************************************************************************/

/**@brief Application main function.
 */
int main(void)
{
		ret_code_t err_code;
		//Xet NRF_SDH_BLE_CENTRAL_LINK_COUNT = 10 trong sdk_config.h
    bool erase_bonds;

    // Initialize.
		reset_struct_data_3_zone(&data_zone_1, &data_zone_2, &data_zone_3);
		ring_buff_init(&ringbuff, arr_mem_data, SIZE_OF_QUEUE);
		lfclk_request();			//ThanhBH Add
		init_peripheral();
		reset_handler();
    uart_init();
    log_init();
    timers_init();
		create_timers();	//ThanhBH Add
		err_code = app_timer_start(timer_send_form_peripheral_to_central_id, APP_TIMER_TICKS(1000), NULL);	//ThanhBH Add timer 1s
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
		err_code = app_timer_start(timer_manage_display_lcd_id, APP_TIMER_TICKS(5000), NULL);	//ThanhBH Add timer 1s
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
	
		err_code = app_timer_start(timer_update_status_BLE_device_id, APP_TIMER_TICKS(3000), NULL);	//ThanhBH Add timer 3s
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
//    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();
		Init_LCD_I2C(2, 3, super_high);
		lcd_i2c_init();
		lcd_i2c_clear();

    // Enter main loop.
    for (;;)
    {
				processDataReceiver();		//ThanhBH Add
        idle_state_handle();
    }
}


/**
 * @}
 */
