/*
 * Copyright(c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BATTERY_MP2667_H_
#define ZEPHYR_DRIVERS_SENSOR_BATTERY_MP2667_H_

#include <logging/log.h>
#include <drivers/gpio.h>

#define MP2667_DELAY 1000


/*** General Constant ***/
#define MP2667_DEVICE_ID 0x09/* Default device ID */

/*** ADDRESS CONFIGURATION ***/
#define MP2667_INPUT_SOURCE_CONTROL 0x00
#define MP2667_POWER-ON_CONFIGURATION 0x01
#define MP2667_CHARGE_CURRENT_CONTROL 0x02
#define MP2667_DISCHARGE_CURRENT 0x03
#define MP2667_CHARGE_VOLTAGE_CONTROL 0x04
#define MP2667_TIMER_CONROL 0x05
#define MP2667_MISCELLANOUS_CONTROL 0x06
#define MP2667_SYSTEM_STATUS 0x07
#define MP2667_FAULT_REG 0x08

uint8_t REG0_VIN_MIN = MP2667_INPUT_SOURCE_CONTROL &(10<<3);
uint8_t REG0_IIN_LIM = MP2667_INPUT_SOURCE_CONTROL & 3;

uint8_t REG1_VBATT_UVLO = MP2667_POWER-ON_CONFIGURATION & (1<<2);

uint8_t REG2_ICC = MP2667_CHARGE_CURRENT_CONTROL & 7;
uint8_t REG03_IDSCHG = MP2667_INPUT_SOURCE_CONTROL & 3;

uint8_t REG03_IDSCHG = MP2667_INPUT_SOURCE_CONTROL & 3;





/***REGISTER CONFIGURATION***/
/*
// input source contrl reg_0
#define REG0_VIN_MIN BIT[3] //4.60v
#define REG0_VIN_MIN BIT[6]

#define REG0_IIN_LIM BIT[0]// 470mA
#define REG0_IIN_LIM BIT[1]

//power on configuration - register_1
#define REG1_VBATT_UVLO BIT[2]

//fast charge current setting - register_2
#define REG2_ICC BIT[0] //257mA 
#define REG2_ICC BIT[1] 
#define REG2_ICC BIT[2] 

//discharge termination current - register_3
#define REG3_ITERM[1] //56mA
#define REG3_IDSCHG[3]
#define REG3_IDSCHG[6]

//battery regulation volt - register_4
#define REG4_VBATT_REG BIT[5] // 4.2v
#define REG4_VBATT_REG BIT[7]

//pre-charge to fast charge - register_4
#define REG4_VBATT_PRE BIT[1] //3.0v
#define REG4_VRECH BIT[0] //300mv

//termination setting - register_5
#define REG5_EN_TERM BIT[6]
#define REG5_EN_TIMER BIT[3]
#define REG5_CHG_TMR BIT[2] //5hrs

//miscellaneous operation control - register_6
#define EN_NTC BIT[3]
#define TJ_REG BIT[1] // default:120c 11
#define TJ_REG BIT[0]
*/



struct bq274xx_data {
	uint16_t input_minimum_voltage;
	uint16_t input_current_limit;
	uint16_t fast_charge_current;
	uint16_t battery_UVLO_thershold;
	uint16_t terminal_current;
	uint16_t battery_regulation_voltage;
	uint16_t pre_charge_to_fast_charge;
	uint16_t battery_recharge_threshold;
	uint16_t discharge_current_limit;
	uint16_t thermal_regulation_threshold;
	uint16_t fast_charge_timer;

};

struct mp2667_config {
        char *bus_name;
        uint16_t input_current_limit;
        uint16_t input_minimum_voltage_reg;
        uint16_t charging_current;
        uint16_t battery_reg_volt;
	uint16_t safety_timer;
	uint16_t battery_reg_volt;


#ifdef CONFIG_PM_DEVICE
        struct gpio_dt_spec int_gpios;
#endif
};

#endif




