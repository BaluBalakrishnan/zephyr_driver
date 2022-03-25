/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mps_mp2667

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <pm/device.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>

#include "mp2667.h"

#define MP2667_DELAY 5 
#define MP_I2C_ADDR  0x09




static int mp2667_gauge_configure(const struct device *dev);

static int mp2667_command_reg_read(struct mp2667_data *mp2667, uint8_t reg_addr,
				    int16_t *val)
{
	uint8_t i2c_data[2];
	int status;

	status = i2c_burst_read(mp2667->i2c, MP_I2C_ADDR, reg_addr,
				i2c_data, 2);
	if (status < 0) {
		LOG_ERR("Unable to read register");
		return -EIO;
	}

	*val = (i2c_data[1] << 8) | i2c_data[0];
	return 0;
}



static int termination__mode (struct mp2667_data , int16_t *val)
{
	uint8_t REG_BIT[1] , EN_TERM;
	int status;
	EN_TERM = MP2667_TIMER_CONROL & (1<<3);

	status = i2c_burst_read(mp2667->i2c, MP_I2C_ADDR, MP2667_TIMER_CONROL, EN_TERM, 1);
	if(status == 0)
	{
	LOG_ERR("Charge Status To Charge Done");
	}
	else
	{
	LOG_ERR("Charge Current Continues Tapering Off");
	}
	return 0;
}


static int safety_timer(const struct device *dev)
{
	int status;
	uint8_t CEB ,EN_TIMER ,SOFT_RST ,i2c_data;
	struct mp2667_data *mp2667 = dev->data;

	
	CEB = MP2667_POWER-ON_CONFIGURATION << BIT[3];
	EN_TIMER = MP2667_TIMER_CONROL << BIT[3];
	SOFT_RST = MP2667_POWER-ON_CONFIGURATION << BIT[7];
		
	status = i2c_reg_read_byte(mp2667, MP_I2C_ADDR, CEB, i2c_data);
	if(status)
        {
	LOG_ERR("CHARGE IS ENABLE");
	}
	else
	{
	LOG_ERR("CHARGE IS DISABLE");
	}
	
	status = i2c_reg_read_byte(mp2667, MP_I2C_ADDR, EN_TIMER, i2c_data);
	if(status)
        {
	LOG_ERR("SAFETY TIMER ENABLE");
	}
	else
	{
	LOG_ERR("SAFETY TIMER DISABLE");
	}
	status = i2c_reg_read_byte(mp2667, MP_I2C_ADDR, SOFT_RST, i2c_data);
        if(status)
        {
	LOG_ERR("SOFTWARE RESET ENABLE");
	}
	else
	{
	LOG_ERR("SOFTWARE RESET DISABLE");
        }
	
	return 0;
}	



static int interrupt_fault_mode (const struct device *dev)
{
	int status;
	uint16_t interrupt_mode, watchdog_timer, input_fault, safety_timer_fault, battery_OVP_fault, NTC_fault ;
	
     	struct interrupt_mode = {
		watchdog_timer,
	       	input_fault,
	       	safety_timer_fault, 
		battery_OVP_fault, 
		NTC_fault
		};
	



}

static int mp2667_NTC_mode(const struct device *dev, uint16_t *val)//temperature sense input
{
	int status;
	uint16_t I_NTC=&val ,V_COLD=&val ,V_HOT=&val ;
	

	if(I_NTC >= -100 && I_NTC <=100)
	{
	LOG_ERR("THERMAL OUTPUT CURRENT WITH IN A LIMIT ");
	}
	else if(V_COLD >=64 && V_COLD <= 68)
	{
	LOG_ERR("COLD TEMPERTURE RISING THRESHOLD WITH IN A LIMIT");
	}
	else if(V_HOT >= 33 && V_HOT <=37)
	{
	LOG_ERR("HOT TEMPERATURE FALLING THRESHOLD WITH IN A LIMIT");
	}
	else
	{
	LOG_ERR("NO THERMISTOR FAULT AT NTC MODE");
	}

}

static int battery_charge_mode (const device *dev)
{
	int status;
	uint8_t pre_charge, fast_charge, constant_volt , i2c_data[3];

}	





static int mp2667_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mp2667_data *mp2667 = dev->data;
	float int_temp;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = ((mp2667->input_minimum_voltage / 1000));
		val->val2 = ((mp2667->input_minimum_voltage 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
                val->val1 = ((mp2667->input_current_limit / 1000));
                val->val2 = ((mp2667->input_current_limit % 1000) * 1000U);
                break;
	
	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                val->val1 = ((mp2667->battery_UVLO_thershold / 1000));
                val->val2 = ((mp2667->battery_UVLO_thershold 1000) * 1000U);
                break;
	
	case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
                val->val1 = ((mp2667->fast_charge_current / 1000));
                val->val2 = ((mp2667->fast_charge_current % 1000) * 1000U);
                break;

	case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
                val->val1 = ((mp2667->discharge_current_limit / 1000));
                val->val2 = ((mp2667->discharge_current_limit % 1000) * 1000U);
                break;
	
       	case SENSOR_CHAN_CURRENT:
                val->val1 = ((mp2667->terminal_current / 1000));
                val->val2 = ((mp2667->terminal_current % 1000) * 1000U);
                break;
	
	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                val->val1 = ((mp2667->battery_regulation_voltage / 1000));
                val->val2 = ((mp2667->battery_regulation_voltage% 1000) * 1000U);
                break;
	
	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                val->val1 = ((mp2667->pre_charge_to_fast_charge / 1000));
                val->val2 = ((mp2667->pre_charge_to_fast_charge% 1000) * 1000U);
                break;
	
	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                val->val1 = ((mp2667->battery_recharge_threshold / 1000));
                val->val2 = ((mp2667->battery_recharge_threshold% 1000) * 1000U);
                break;

	case SENSOR_CHAN_GAUGE_TEMP:
		int_temp = (mp2667->thermal_regulation_threshold* 0.1f);
		int_temp = int_temp - 273.15f;
		val->val1 = (int32_t)int_temp;
		val->val2 = (int_temp - (int32_t)int_temp) * 1000000;
		break;
	
	default:
		
		return -ENOTSUP;
	}

	return 0;
}

static int mp2667_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mp2667_data mp2667 = dev->data;
	int status = 0;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
	status = mp2667_command_reg_read(mp2667, MP2667_INPUT_SOURCE_CONTROL, 
				&mp2667->input_minimum_voltage);
		if (status < 0) {
			LOG_ERR("Failed to read input_minimum_voltage");
			return -EIO;
		}
  		break;

	case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
		status = mp2667_command_reg_read(mp2667,
						  MP2667_INPUT_SOURCE_CONTROL,
						  &mp2667->input_current_limit);
		if (status < 0) {
			LOG_ERR("Failed to read input current limit");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
		status = mp2667_command_reg_read(
			mp2667, MP2667_POWER-ON_CONFIGURATION,
			&mp2667->battery_UVLO_thershold);
		if (status < 0) {
			LOG_ERR("Failed to read battery_UVLO_thershold");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
		status = mp2667_command_reg_read(mp2667,
						  &MP2667_CHARGE_CURRENT_CONTROL,
						  mp2667->fast_charge_current);
		if (status < 0) {
			LOG_ERR("Failed to read fast_charge_current");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
		status = mp2667_command_reg_read(mp2667,
						  MP2667_DISCHARGE_CURRENT,
						  &mp2667->discharge_current_limit);
		if (status < 0) {
			LOG_ERR("Failed to read discharge_current_limit");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_CURRENT:
		status = mp2667_command_reg_read(mp2667, MP2667_DISCHARGE_CURRENT,
			       	&mp2667->terminal_current);
		if (status < 0) {
			LOG_ERR("Failed to read terminal_current");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                status = mp2667_command_reg_read(mp2667, MP2667_CHARGE_VOLTAGE_CONTROL,
                                                  &mp2667->battery_regulation_voltage);
                if (status < 0) {
                        LOG_ERR("Failed to read battery_regulation_voltage");
                        return -EIO;
                }
                break;

	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                status = mp2667_command_reg_read(mp2667, MP2667_CHARGE_VOLTAGE_CONTROL,
                                                  &mp2667->pre_charge_to_fast_charge);
                if (status < 0) {
                        LOG_ERR("Failed to read battery pre_charge_to_fast_charge");
                        return -EIO;
                }
                break;

	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
                status = mp2667_command_reg_read(mp2667, MP2667_CHARGE_VOLTAGE_CONTROL,
                                                  &mp2667->battery_recharge_threshold);
                if (status < 0) {
                        LOG_ERR("Failed to read battery_recharge_threshold");
                        return -EIO;
                }
                break;

	case SENSOR_CHAN_GAUGE_TEMP:
                status = mp2667_command_reg_read(mp2667, MP2667_MISCELLANOUS_CONTROL,
                                                  &mp2667->thermal_regulation_threshold);
                if (status < 0) {
                        LOG_ERR("Failed to read thermal_regulation_threshold");
                        return -EIO;
                }
                break;

	default:
		{
		return -ENOTSUP;
	}
 
	return 0;
}

static int mp2667_gauge_init(const struct device *dev)
{
	struct mp2667_data *mp2667 = dev->data;
	const struct mp2667_config *const config = dev->config;
	int status = 0;
	uint16_t id;

#ifdef CONFIG_PM_DEVICE
	if (!device_is_ready(config->int_gpios.port)) {
		LOG_ERR("GPIO device pointer is not ready to be used");
		return -ENODEV;
	}
#endif

	mp2667->i2c = device_get_binding(config->bus_name);
	if (mp2667->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			config->bus_name);
		return -EINVAL;
	}

	status = mp2667_get_device_type(mp2667, &id);
	if (status < 0) {
 		LOG_ERR("Unable to get device ID");
		return -EIO;
	}

	if (id != MP2667_DEVICE_ID) {
		LOG_ERR("Invalid Device");
		return -EINVAL;
	}
}

#define MP2667_INT_CFG(index)                                                                    \
        .int_gpios = GPIO_DT_SPEC_INST_GET(index, int_gpios),
#else
#define MP2667_INT_CFG(index)
#endif


#define MP2667_INIT(index)                                                                       \
        static struct  mp2667_data MP2667_driver_##index;                                        \
                                                                                                 \
        static const struct mp2667_config mp2667_config_##index= {                               \
                MP2667_INT_CFG(index)                                                            \
                .bus_name = DT_INST_BUS_LABEL(index),                                            \
                .input_current_limit = DT_INST_PROP(index, input_current_limit),                 \
                .input_minimum_voltage_reg = DT_INST_PROP(index, input_minimum_voltage_reg),     \



static const struct sensor_driver_api mp2667_battery_driver_api = {
	.sample_fetch = mp2667_sample_fetch,
	.channel_get = mp2667_channel_get,
};

#ifdef CONFIG_PM_DEVICE
#define MP2667_INT_CFG(index)						     			 \
	.int_gpios = GPIO_DT_SPEC_INST_GET(index, int_gpios),
#else
#define MP2667_INT_CFG(index)
#endif


#define MP2667_INIT(index)                                                  	 		 \
	static struct  mp2667_data MP2667_driver_##index;                    	 		 \
									      			 \
	static const struct mp2667_config mp2667_config_##index= {          			 \
		MP2667_INT_CFG(index)                                			         \
		.bus_name = DT_INST_BUS_LABEL(index),                         			 \
		.input_current_limit = DT_INST_PROP(index, input_current_limit),   		 \
		.input_minimum_voltage_reg = DT_INST_PROP(index, input_minimum_voltage_reg), 	 \
		.charging_current = DT_INST_PROP(index, charging_current),           		 \
		.battery_reg_volt = DT_INST_PROP(index, battery_reg_volt),
	     	.safety_timer = DT_INST_PROP(index, safety_timer),				 \
		.battery_UVLO = DT_INST_PROP(index, battery_UVLO),				 \
									      			 \
	};                                                                    			 \
									       			 \
	PM_DEVICE_DT_INST_DEFINE(index, mp2667_pm_action);		      			 \
									      			 \
	DEVICE_DT_INST_DEFINE(index, &mp2667_gauge_init,		      			 \
			    PM_DEVICE_DT_INST_GET(index),		      			 \
			    &mp2667_driver_##index,                          			 \
			    &mp2667_config_##index, POST_KERNEL,             			 \
			    CONFIG_SENSOR_INIT_PRIORITY,                      			 \
			    &mp2667_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MP2667_INIT)
