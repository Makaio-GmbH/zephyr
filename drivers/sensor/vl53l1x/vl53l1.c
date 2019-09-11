/* vl53l1.c - Driver for ST VL53L1X time of flight sensor */
/*
 * Copyright (c) 2017 STMicroelectronics
 * Copyright (c) 2019 Makaio GmbH
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



#define LOG_LEVEL 4//CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(VL53L1X);

static struct gpio_callback vl53l1x_cb;
K_SEM_DEFINE(vl53l1x_sem, 0, 1);

void vl53l1x_interrupt_cb(struct device *gpiob, struct gpio_callback *cb,
						   u32_t pins)
{
	ARG_UNUSED(gpiob);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);


	k_sem_give(&vl53l1x_sem);
}


struct vl53l1x_pin_config {
	bool use;
	u32_t gpios_pin;
	char *gpios_ctrl;
};

struct vl53l1x_data {
	char *i2c_dev_name;
	u16_t i2c_addr;
	VL53L1_Dev_t vl53l1x;
	VL53L1_RangingMeasurementData_t RangingMeasurementData;
	int inter_measurement_period;
	int timing_budget;
	struct vl53l1x_pin_config irq_config;
	struct vl53l1x_pin_config xshut_config;
	u8_t instance_id;
};

static void vl53l1x_print_device_settings(struct device *dev)
{
	struct vl53l1x_data *drv_data = dev->driver_data;

	struct vl53l1x_pin_config *irq_cfg = &drv_data->irq_config;

	LOG_DBG("Name:\t\t%s", log_strdup(dev->config->name));
	LOG_DBG("Instance ID:\t%u", drv_data->instance_id);
	LOG_DBG("I2C Address:\t0x%02x", drv_data->i2c_addr);

	if (irq_cfg && irq_cfg->use) {
		LOG_DBG("IRQ:\t\tenabled, ctrl %s, pin %u",
				irq_cfg->gpios_ctrl, irq_cfg->gpios_pin);
	} else {
		LOG_DBG("IRQ:\t\tdisabled");
	}

	struct vl53l1x_pin_config *xshut_cfg = &drv_data->xshut_config;

	if (xshut_cfg && xshut_cfg->use) {
		LOG_DBG("XSHUT:\t\tenabled, ctrl %s, pin %u",
				xshut_cfg->gpios_ctrl, xshut_cfg->gpios_pin);
	} else {
		LOG_DBG("XSHUT:\t\tdisabled");
	}


}

static void vl53l1x_print_ranging_data(VL53L1_Dev_t *Dev)
{
	static VL53L1_RangingMeasurementData_t RangingData;

	VL53L1_Error status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
	if(!status)
	{
		LOG_DBG("Status: %d", RangingData.RangeStatus);
		LOG_DBG("RangeMilliMeter: %d", RangingData.RangeMilliMeter);

	} else {
		LOG_ERR("Error: %d", status);
	}
}

#define I2C_TEST
#ifdef I2C_TEST
#define REG 0x3A
int rd_write_verification( VL53L1_Dev_t *dev, u16_t addr, u32_t expected_value)
{
	VL53L1_Error Status  = VL53L1_ERROR_NONE;
	u8_t bytes[4],mbytes[4];
	u16_t words[2];
	u32_t dword;
	int i;
	VL53L1_ReadMulti(dev, addr, mbytes, 4);
	for (i=0; i<4; i++){ VL53L1_RdByte(dev, addr+i, &bytes[i]); }
	for (i=0; i<2; i++){ VL53L1_RdWord(dev, addr+i*2, &words[i]); }
	Status = VL53L1_RdDWord(dev, addr, &dword);

	LOG_DBG("expected   = %8x,\n",expected_value);
	LOG_DBG("read_multi = %2x, %2x, %2x, %2x\n", mbytes[0],mbytes[1],mbytes[2],mbytes[3]);
	LOG_DBG("read_bytes = %2x, %2x, %2x, %2x\n", bytes[0],bytes[1],bytes[2],bytes[3]);
	LOG_DBG("read words = %4x, %4x\n",words[0],words[1]);
	LOG_DBG("read dword = %8x\n",dword);

	if((mbytes[0]<<24 | mbytes[1]<<16 | mbytes[2]<<8 | mbytes[3]) != expected_value) return (-1);
	if((bytes[0]<<24 | bytes[1]<<16 | bytes[2]<<8 | bytes[3]) != expected_value) return (-1);
	if((words[0]<<16 | words[1]) != expected_value) return (-1);
	if(dword != expected_value) return(-1);
	return Status;

}

void i2c_test(VL53L1_Dev_t *dev)
{
	VL53L1_Error Status  = VL53L1_ERROR_NONE;
	int err_count = 0;
	uint8_t buff[4] = {0x11,0x22,0x33,0x44};
	uint8_t long_out[135] ={0x29, 0x02, 0x10, 0x00, 0x22, 0xBC, 0xCC, 0x81, 0x80, 0x07, 0x16, 0x00, 0xFF, 0xFD,
							0xF7, 0xDE, 0xFF, 0x0F, 0x00, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x44, 0x00, 0x2C, 0x00, 0x11, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF,
							0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00, 0x00,
							0x02, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00,
							0x9D, 0x07, 0x00, 0xD2, 0x05, 0x01, 0x68, 0x00, 0xC0, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x03,
							0xB6, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x01, 0x00, 0x02,
							0xC7, 0xFF, 0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40};
	uint8_t long_in[135]= {0xff};
	int i=0;

	Status = rd_write_verification(dev, 0x10f, 0xeacc10ff);			// verify the Chip ID works

	Status += VL53L1_WriteMulti(dev, 0x01,  long_out, 135);			// check if WriteMulti can write 135 bytes
	Status += VL53L1_ReadMulti(dev, 0x01,  long_in, 135);			// check if WriteMulti can read 135 bytes

	for (i=0; i<135; i++) if(long_in[i] != long_out[i])err_count++;
	if (err_count > 10) Status++;

	Status += VL53L1_WriteMulti(dev, REG,  buff, 4);				// check WriteMulti
	if (rd_write_verification(dev, REG, 0x11223344) <0) err_count++;

	Status += VL53L1_WrDWord(dev, REG, 0xffeeddcc);				// check WrDWord
	if (rd_write_verification(dev, REG, 0xffeeddcc) <0) err_count++;

	Status += VL53L1_WrWord(dev, REG, 0x5566);					// check WrWord
	Status += VL53L1_WrWord(dev, REG+2, 0x7788);
	if (rd_write_verification(dev, REG, 0x55667788) <0) err_count++;

	for (i=0; i<4; i++){ VL53L1_WrByte (dev, REG+i, buff[i]); }
	if (rd_write_verification(dev, REG,0x11223344) <0) Status++;
	if (Status != 0)
	{
		for(u8_t i = 0; i < 10; i++)
		{
			LOG_DBG("i2c test failed - please check it. Status = %d\n", Status);
			k_sleep(1000);
		}

	} else {
		for(u8_t i = 0; i < 10; i++)
		{
			LOG_DBG("i2c test succeeded");
			k_sleep(1000);
		}
	}
}

#endif

static int vl53l1x_init(struct device *dev)
{

	struct vl53l1x_data *drv_data = dev->driver_data;
	VL53L1_Error ret;
	u16_t vl53l1x_id = 0U;
	VL53L1_DeviceInfo_t vl53l1x_dev_info;

	u8_t data_ready;
	u8_t raw_ready;

	LOG_DBG("enter in %s", __func__);

	vl53l1x_print_device_settings(dev);

	k_sleep(10000);

	VL53L1_Dev_t vl53l1dev = {
			.I2cDevAddr = drv_data->i2c_addr,
			.I2cHandle = device_get_binding(drv_data->i2c_dev_name)
	};


	drv_data->vl53l1x.I2cDevAddr = drv_data->i2c_addr;
	drv_data->vl53l1x.I2cHandle = device_get_binding(drv_data->i2c_dev_name);


	if (vl53l1dev.I2cHandle == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
				drv_data->i2c_dev_name);
		return -EINVAL;
	}



	struct device *xshut_gpios = device_get_binding(DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_CONTROLLER);
	gpio_pin_configure(xshut_gpios, DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_PIN, GPIO_DIR_OUT);
	gpio_pin_write(xshut_gpios, DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_PIN, 0);
	k_sleep(100);
	gpio_pin_write(xshut_gpios, DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_PIN, 1);
	k_sleep(100);


#ifdef I2C_TEST
	i2c_test(&drv_data->vl53l1x);
#endif



	VL53L1_software_reset(&drv_data->vl53l1x);

	ret = VL53L1_WaitDeviceBooted(&drv_data->vl53l1x);

	if (ret) {
		return -ETIMEDOUT;
	}

	LOG_DBG("VL53L1_WaitDeviceBooted succeeded");

	/* sensor init */
	ret = VL53L1_DataInit(&drv_data->vl53l1x);
	if (ret < 0) {
		LOG_ERR("VL53L1X_DataInit return error (%d)", ret);
		return -ENOTSUP;
	}

	LOG_DBG("VL53L1_DataInit succeeded");

	/* static init */
	ret = VL53L1_StaticInit(&drv_data->vl53l1x);
	if (ret < 0) {
		LOG_ERR("VL53L1_StaticInit return error (%d)", ret);
		return -ENOTSUP;
	}


	LOG_DBG("VL53L1_StaticInit succeeded");
/*
	VL53L1_UserRoi_t Roi;
	Roi.TopLeftX = 0;
	Roi.TopLeftY = 15;
	Roi.BotRightX = 15;
	Roi.BotRightY = 0;
	VL53L1_SetUserROI( &drv_data->vl53l1x, &Roi);*/
	VL53L1_SetPresetMode(&drv_data->vl53l1x, VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS);

	u16_t rangedata = 0;

	ret = VL53L1_SetDistanceMode(&drv_data->vl53l1x,
								 VL53L1_DISTANCEMODE_LONG);
	if (ret < 0) {
		LOG_ERR("VL53L1_SetDistanceMode return error (%d)", ret);
		return -ENOTSUP;
	}




	ret = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&drv_data->vl53l1x, 66000);
	if (ret < 0) {
		LOG_ERR("VL53L1_SetMeasurementTimingBudgetMicroSeconds return error (%d)", ret);
		return -ENOTSUP;
	}

	ret = VL53L1_SetInterMeasurementPeriodMilliSeconds(&drv_data->vl53l1x, 1000);
	if (ret < 0) {
		LOG_ERR("VL53L1_SetInterMeasurementPeriodMilliSeconds return error (%d)", ret);
		return -ENOTSUP;
	}
	LOG_DBG("Start Measurement");
	ret = VL53L1_StartMeasurement(&drv_data->vl53l1x);
	k_sleep(5000);
	VL53L1_StopMeasurement(&drv_data->vl53l1x);
	k_sleep(5000);
	LOG_DBG("Start measurement result: %d", ret);
	struct device *irq_gpios = device_get_binding(DT_INST_0_ST_VL53L1X_IRQ_GPIOS_CONTROLLER);
	gpio_pin_configure(irq_gpios, DT_INST_0_ST_VL53L1X_IRQ_GPIOS_PIN, GPIO_DIR_IN | GPIO_INT_ACTIVE_LOW | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE | GPIO_PUD_NORMAL);

	gpio_init_callback(&vl53l1x_cb, vl53l1x_interrupt_cb, BIT(DT_INST_0_ST_VL53L1X_IRQ_GPIOS_PIN));

	gpio_add_callback(irq_gpios, &vl53l1x_cb);
	gpio_pin_enable_callback(irq_gpios, DT_INST_0_ST_VL53L1X_IRQ_GPIOS_PIN);

	int irq_val = 0;
	/*
	device_set_power_state(drv_data->vl53l1x.I2cHandle, DEVICE_PM_LOW_POWER_STATE, NULL, NULL);
	device_set_power_state(device_get_binding("I2C_0"), DEVICE_PM_SUSPEND_STATE, NULL, NULL);
	device_set_power_state(device_get_binding("I2C_1"), DEVICE_PM_SUSPEND_STATE, NULL, NULL);
	struct device *irq_gpios = device_get_binding(DT_INST_0_ST_VL53L1X_IRQ_GPIOS_CONTROLLER);
	gpio_pin_configure(irq_gpios, DT_INST_0_ST_VL53L1X_IRQ_GPIOS_PIN, GPIO_DIR_IN);
*/
	VL53L1_RangingMeasurementData_t measurementData = {0};


	while(true)
	{

		ret = VL53L1_WaitMeasurementDataReady(&drv_data->vl53l1x);
		LOG_DBG("VL53L1_WaitMeasurementDataReady: %d", ret);
		k_sem_take(&vl53l1x_sem, K_FOREVER);
		VL53L1_ReadMulti(&drv_data->vl53l1x, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,&rangedata,2);

		//LOG_DBG("rangedata: %u", rangedata);

		VL53L1_GetRangingMeasurementData(&drv_data->vl53l1x, &measurementData);
		//LOG_DBG("StreamCount: %u", measurementData.StreamCount);
		LOG_DBG("ret: %d", ret);
		LOG_DBG("MM: %d", measurementData.RangeMilliMeter);
		LOG_DBG("Status: %u", measurementData.RangeStatus);

			VL53L1_StopMeasurement(&drv_data->vl53l1x);
		gpio_pin_write(xshut_gpios, DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_PIN, 0);
		k_sleep(30000);
		gpio_pin_write(xshut_gpios, DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_PIN, 1);
		k_sleep(3000);
		ret =  VL53L1_ClearInterruptAndStartMeasurement(&drv_data->vl53l1x);
		LOG_DBG("OK");
	}


	while(true)
	{
		gpio_pin_read(irq_gpios, DT_INST_0_ST_VL53L1X_IRQ_GPIOS_PIN, &irq_val);
		if(irq_val)
		{

				LOG_DBG("PIN HIGH");
				k_sleep(1000);
			VL53L1_GetRangingMeasurementData(&drv_data->vl53l1x, &measurementData);
			LOG_DBG("MM: %d", measurementData.RangeMilliMeter);
			VL53L1_StartMeasurement(&drv_data->vl53l1x);
		} else {
			LOG_DBG("Pin low");
			k_sleep(1000);
		}


	}

	VL53L1_WaitMeasurementDataReady(&drv_data->vl53l1x);


	while(true)
	{
		ret =  VL53L1_ClearInterruptAndStartMeasurement(&drv_data->vl53l1x);
		VL53L1_WaitMeasurementDataReady(&drv_data->vl53l1x);
		k_sleep(10000);
		LOG_DBG("OK");
	}
	LOG_DBG("VL53L1_SetDistanceMode succeeded");
	k_sleep(1000);
	VL53L1_SetPresetMode (&drv_data->vl53l1x,
						   VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS);

	k_sleep(1000);

	//ret = VL53L1_ClearInterruptAndStartMeasurement(&drv_data->vl53l1x);
	//LOG_DBG("VL53L1_ClearInterruptAndStartMeasurement: %d", ret);
	//ret = VL53L1_StartMeasurement(&drv_data->vl53l1x);




	while(true)
	{
		VL53L1_RdByte(
				&drv_data->vl53l1x,
				VL53L1_GPIO__TIO_HV_STATUS,
				&raw_ready);
		LOG_DBG("dready: %u", raw_ready);
		k_sleep(1000);

		VL53L1_GetMeasurementDataReady(&drv_data->vl53l1x, &data_ready);
		LOG_DBG("Cool: %u", data_ready);
		vl53l1x_print_ranging_data(&drv_data->vl53l1x);
		ret = VL53L1_StartMeasurement(&drv_data->vl53l1x);
		/*
		LOG_DBG("VL53L1_ClearInterruptAndStartMeasurement: %d", ret);
		VL53L1_GetMeasurementDataReady(&drv_data->vl53l1x, &data_ready);
		LOG_DBG("Cool: %u", data_ready);
		k_sleep(1000);
		 */
	}

	return 0;
}

#define VL53L1X_PIN_CFG(id, pintype)					\
static const struct vl53l1x_pin_config\
	vl53l1x_##pintype##_##id##_cfg = {	\
		.use = true,	\
		.gpios_pin =	\
		DT_INST_##id##_ST_VL53L1X_##pintype##_GPIOS_PIN,	\
		.gpios_ctrl =	\
		DT_INST_##id##_ST_VL53L1X_##pintype##_GPIOS_CONTROLLER	\
}

#define VL53L1X_NOPIN_CFG(id, pintype)					\
static const struct vl53l1x_pin_config	\
	vl53l1x_##pintype##_##id##_cfg = {	\
/* this is unnecessary, but let's keep it for clarity */	\
		.use = false,	\
	}




#define VL53L1X_INST_INIT(id)	\
static  struct vl53l1x_data vl53l1x_driver_##id##_data = {	\
		.instance_id = id,	\
		.i2c_dev_name = DT_INST_##id##_ST_VL53L1X_BUS_NAME,	\
		.i2c_addr = DT_INST_##id##_ST_VL53L1X_BASE_ADDRESS,	\
		.irq_config = vl53l1x_IRQ_##id##_cfg,	\
		.xshut_config = vl53l1x_XSHUT_##id##_cfg,	\
		.inter_measurement_period =	\
		DT_INST_##id##_ST_VL53L1X_INTER_MEASUREMENT_PERIOD,	\
		.timing_budget =	\
		DT_INST_##id##_ST_VL53L1X_MEASUREMENT_TIMING_BUDGET	\
};	\
DEVICE_AND_API_INIT(vl53l1x,	\
		DT_INST_##id##_ST_VL53L1X_LABEL,	\
		vl53l1x_init,	\
		&vl53l1x_driver_##id##_data,	\
NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL)

#ifdef DT_INST_0_ST_VL53L1X_BASE_ADDRESS
#ifndef DT_INST_0_ST_VL53L1X_INTER_MEASUREMENT_PERIOD
#define DT_INST_0_ST_VL53L1X_INTER_MEASUREMENT_PERIOD
			CONFIG_VL53L1X_INTERMEASUREMENT_PERIOD
#endif
#ifndef DT_INST_0_ST_VL53L1X_MEASUREMENT_TIMING_BUDGET
#define DT_INST_0_ST_VL53L1X_MEASUREMENT_TIMING_BUDGET
			CONFIG_VL53L1X_MEAS_TIMING_BUDGET
#endif
#ifdef DT_INST_0_ST_VL53L1X_IRQ_GPIOS_PIN
VL53L1X_PIN_CFG(0, IRQ);
#else
VL53L1X_NOPIN_CFG(0, IRQ);
#endif
#ifdef DT_INST_0_ST_VL53L1X_XSHUT_GPIOS_PIN
VL53L1X_PIN_CFG(0, XSHUT);
#else
VL53L1X_NOPIN_CFG(0, XSHUT);
#endif
VL53L1X_INST_INIT(0);
#endif

