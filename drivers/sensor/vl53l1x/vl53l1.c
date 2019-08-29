//
// Created by Christoph Schramm on 2019-08-28.
//

/* vl53l0x.c - Driver for ST VL53L0X time of flight sensor */

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/__assert.h>
#include <zephyr/types.h>
#include <device.h>
#include <logging/log.h>

#include "vl53l1_api.h"
#include "vl53l1_platform.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(VL53L1X);

struct vl53l1x_data {
	struct device *i2c;
	VL53L1_Dev_t vl53l1x;
	VL53L1_RangingMeasurementData_t RangingMeasurementData;
};

static int vl53l1x_init(struct device *dev)
{
	struct vl53l1x_data *drv_data = dev->driver_data;
	VL53L1_Error ret;
	u16_t vl53l1x_id = 0U;
	VL53L1_DeviceInfo_t vl53l1x_dev_info;

	LOG_DBG("enter in %s", __func__);

#ifdef CONFIG_VL53L0X_XSHUT_CONTROL_ENABLE
	struct device *gpio;

	/* configure and set VL53L0X_XSHUT_Pin */
	gpio = device_get_binding(CONFIG_VL53L0X_XSHUT_GPIO_DEV_NAME);
	if (gpio == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			CONFIG_VL53L0X_XSHUT_GPIO_DEV_NAME);
		return -EINVAL;
	}

	if (gpio_pin_configure(gpio,
			       CONFIG_VL53L0X_XSHUT_GPIO_PIN_NUM,
			       GPIO_DIR_OUT | GPIO_PUD_PULL_UP) < 0) {
		LOG_ERR("Could not configure GPIO %s %d).",
			CONFIG_VL53L0X_XSHUT_GPIO_DEV_NAME,
			CONFIG_VL53L0X_XSHUT_GPIO_PIN_NUM);
		return -EINVAL;
	}

	gpio_pin_write(gpio, CONFIG_VL53L0X_XSHUT_GPIO_PIN_NUM, 1);
	k_sleep(100);
#endif

	drv_data->i2c = device_get_binding(DT_INST_0_ST_VL53L1X_BUS_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			DT_INST_0_ST_VL53L1X_BUS_NAME);
		return -EINVAL;
	}


	drv_data->vl53l1x.I2cHandle = drv_data->i2c;
	drv_data->vl53l1x.I2cDevAddr = DT_INST_0_ST_VL53L1X_BASE_ADDRESS;

	ret = VL53L1_WaitDeviceBooted(&drv_data->vl53l1x);
	if (ret) {
		return -ETIMEDOUT;
	}

	/* sensor init */
	ret = VL53L1_DataInit(&drv_data->vl53l1x);
	if (ret < 0) {
		LOG_ERR("VL53L1X_DataInit return error (%d)", ret);
		return -ENOTSUP;
	}


	return 0;
}

static struct vl53l1x_data vl53l1x_driver;

DEVICE_AND_API_INIT(vl53l1x, DT_INST_0_ST_VL53L1X_LABEL, vl53l1x_init, &vl53l1x_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);
//		&vl53l0x_api_funcs);

