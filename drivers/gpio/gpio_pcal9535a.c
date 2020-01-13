/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Driver for PCAL9535A I2C-based GPIO driver.
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include "gpio_pcal9535a.h"
#include "gpio_utils.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
#include <irq.h>

LOG_MODULE_REGISTER(gpio_pcal9535a);

/* Register definitions */
#define REG_INPUT_PORT0			0x00
#define REG_INPUT_PORT1			0x01
#define REG_OUTPUT_PORT0		0x02
#define REG_OUTPUT_PORT1		0x03
#define REG_POL_INV_PORT0		0x04
#define REG_POL_INV_PORT1		0x05
#define REG_CONF_PORT0			0x06
#define REG_CONG_PORT1			0x07
#define REG_OUT_DRV_STRENGTH_PORT0_L	0x40
#define REG_OUT_DRV_STRENGTH_PORT0_H	0x41
#define REG_OUT_DRV_STRENGTH_PORT1_L	0x42
#define REG_OUT_DRV_STRENGTH_PORT1_H	0x43
#define REG_INPUT_LATCH_PORT0		0x44
#define REG_INPUT_LATCH_PORT1		0x45
#define REG_PUD_EN_PORT0		0x46
#define REG_PUD_EN_PORT1		0x47
#define REG_PUD_SEL_PORT0		0x48
#define REG_PUD_SEL_PORT1		0x49
#define REG_INT_MASK_PORT0		0x4A
#define REG_INT_MASK_PORT1		0x4B
#define REG_INT_STATUS_PORT0		0x4C
#define REG_INT_STATUS_PORT1		0x4D
#define REG_OUTPUT_PORT_CONF		0x4F

static struct gpio_callback pcal9535a_gpio_cb;


typedef struct pcal9535a_int_event {
	struct device *device;
	u32_t pins;
} pcal9535a_int_event;

K_MSGQ_DEFINE(pcal9535a_int_msgq, sizeof(pcal9535a_int_event), 10, 4);

/**
 * @brief Check to see if a I2C master is identified for communication.
 *
 * @param dev Device struct.
 * @return 1 if I2C master is identified, 0 if not.
 */
static inline int has_i2c_master(struct device *dev)
{
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	struct device * const i2c_master = drv_data->i2c_master;

	if (i2c_master) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * @brief Read both port 0 and port 1 registers of certain register function.
 *
 * Given the register in reg, read the pair of port 0 and port 1.
 *
 * @param dev Device struct of the PCAL9535A.
 * @param reg Register to read (the PORT0 of the pair of registers).
 * @param buf Buffer to read data into.
 *
 * @return 0 if successful, failed otherwise.
 */
static int read_port_regs(struct device *dev, u8_t reg,
			   union gpio_pcal9535a_port_data *buf)
{
	const struct gpio_pcal9535a_config * const config =
		dev->config->config_info;
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	struct device * const i2c_master = drv_data->i2c_master;
	u16_t i2c_addr = config->i2c_slave_addr;
	int ret;

	ret = i2c_burst_read(i2c_master, i2c_addr, reg, buf->byte, 2);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error reading register 0x%X (%d)",
			i2c_addr, reg, ret);
		goto error;
	}

	LOG_DBG("PCAL9535A[0x%X]: Read: REG[0x%X] = 0x%X, REG[0x%X] = 0x%X",
		i2c_addr, reg, buf->byte[0], (reg + 1), buf->byte[1]);

error:
	return ret;
}

/**
 * @brief Write both port 0 and port 1 registers of certain register function.
 *
 * Given the register in reg, write the pair of port 0 and port 1.
 *
 * @param dev Device struct of the PCAL9535A.
 * @param reg Register to write into (the PORT0 of the pair of registers).
 * @param buf Buffer to write data from.
 *
 * @return 0 if successful, failed otherwise.
 */
static int write_port_regs(struct device *dev, u8_t reg,
			    union gpio_pcal9535a_port_data *buf)
{
	const struct gpio_pcal9535a_config * const config =
		dev->config->config_info;
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	struct device * const i2c_master = drv_data->i2c_master;
	u16_t i2c_addr = config->i2c_slave_addr;
	int ret;

	LOG_DBG("PCAL9535A[0x%X]: Write: REG[0x%X] = 0x%X, REG[0x%X] = "
		"0x%X", i2c_addr, reg, buf->byte[0], (reg + 1),
		buf->byte[1]);

	ret = i2c_reg_write_byte(i2c_master, i2c_addr, reg, buf->byte[0]);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error writing from register 0x%X "
			"(%d)", i2c_addr, reg, ret);
	}
	ret = i2c_reg_write_byte(i2c_master, i2c_addr, reg+1, buf->byte[1]);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error writing from register 0x%X "
				"(%d)", i2c_addr, reg, ret);
	}
	return ret;
}

/**
 * @brief Setup the pin direction (input or output)
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_dir(struct device *dev, int access_op,
			  u32_t pin, int flags)
{
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	union gpio_pcal9535a_port_data *port = &drv_data->reg_cache.dir;
	u16_t bit_mask;
	u16_t new_value = 0U;
	int ret;

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		bit_mask = 1 << pin;

		/* Config 0 == output, 1 == input */
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			new_value = 1 << pin;
		}

		port->all &= ~bit_mask;
		port->all |= new_value;

		break;
	case GPIO_ACCESS_BY_PORT:
		/* Config 0 == output, 1 == input */
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			port->all = 0xFFFF;
		} else {
			port->all = 0x0;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto done;
	}

	ret = write_port_regs(dev, REG_CONF_PORT0, port);

done:
	return ret;
}

/**
 * @brief Setup the pin pull up/pull down status
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_pullupdown(struct device *dev, int access_op,
				 u32_t pin, int flags)
{
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	union gpio_pcal9535a_port_data *port;
	u16_t bit_mask;
	u16_t new_value = 0U;
	int ret;

	/* If disabling pull up/down, there is no need to set the selection
	 * register. Just go straight to disabling.
	 */
	if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
		goto en_dis;
	}

	/* Setup pin pull up or pull down */
	port = &drv_data->reg_cache.pud_sel;
	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		bit_mask = 1 << pin;

		/* pull down == 0, pull up == 1*/
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			new_value = 1 << pin;
		}

		port->all &= ~bit_mask;
		port->all |= new_value;

		break;
	case GPIO_ACCESS_BY_PORT:
		/* pull down == 0, pull up == 1*/
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			port->all = 0xFFFF;
		} else {
			port->all = 0x0;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto done;
	}

	ret = write_port_regs(dev, REG_PUD_SEL_PORT0, port);
	if (ret) {
		goto done;
	}

en_dis:
	/* enable/disable pull up/down */
	port = &drv_data->reg_cache.pud_en;
	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		bit_mask = 1 << pin;

		if ((flags & GPIO_PUD_MASK) != GPIO_PUD_NORMAL) {
			new_value = 1 << pin;
		}

		port->all &= ~bit_mask;
		port->all |= new_value;

		break;
	case GPIO_ACCESS_BY_PORT:
		if ((flags & GPIO_PUD_MASK) != GPIO_PUD_NORMAL) {
			port->all = 0xFFFF;
		} else {
			port->all = 0x0;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto done;
	}

	ret = write_port_regs(dev, REG_PUD_EN_PORT0, port);

done:
	return ret;
}

/**
 * @brief Setup the polarity of pin or port
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_polarity(struct device *dev, int access_op,
			       u32_t pin, int flags)
{
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	union gpio_pcal9535a_port_data *port = &drv_data->reg_cache.pol_inv;
	u16_t bit_mask;
	u16_t new_value = 0U;
	int ret;

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		bit_mask = BIT(pin);

		/* normal == 0, invert == 1 */
		if ((flags & GPIO_POL_MASK) == GPIO_POL_INV) {
			new_value = BIT(pin);
		}

		port->all &= ~bit_mask;
		port->all |= new_value;

		break;
	case GPIO_ACCESS_BY_PORT:
		/* normal == 0, invert == 1 */
		if ((flags & GPIO_POL_MASK) == GPIO_POL_INV) {
			port->all = 0xFFFF;
		} else {
			port->all = 0x0;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto done;
	}

	ret = write_port_regs(dev, REG_POL_INV_PORT0, port);
	if (!ret) {
		drv_data->out_pol_inv = port->all;
	}

done:
	return ret;
}

/**
 * @brief Enable or disable pin interrupts
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int setup_pin_interrupts(struct device *dev, int access_op,
						 u32_t pin, int flags)
{
	struct gpio_pcal9535a_drv_data * const drv_data =
			(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	union gpio_pcal9535a_port_data *port = &drv_data->reg_cache.int_en;
	u16_t bit_mask;
	u16_t new_value = 0U;
	int ret;

	if (access_op == GPIO_ACCESS_BY_PIN) {

		bit_mask = BIT(pin);

		/* 0 == INT enabled, 1 == INT disabled */
		if (!(flags & GPIO_INT)) {
			new_value = BIT(pin);
		}

		port->all &= ~bit_mask;
		port->all |= new_value;
	} else {
		ret = -ENOTSUP;
		goto done;
	}

	ret = write_port_regs(dev, REG_INT_MASK_PORT0, port);

	done:
	return ret;
}

/**
 * @brief Configure pin or port
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_pcal9535a_config(struct device *dev, int access_op,
				 u32_t pin, int flags)
{
	int ret;

	struct gpio_pcal9535a_drv_data * const drv_data =
			(struct gpio_pcal9535a_drv_data * const)dev->driver_data;

#if (CONFIG_GPIO_LOG_LEVEL >= LOG_LEVEL_DEBUG)
	const struct gpio_pcal9535a_config * const config =
		dev->config->config_info;
	u16_t i2c_addr = config->i2c_slave_addr;
#endif

	if ((flags & GPIO_INT) && !drv_data->supports_interrupts) {
		LOG_ERR("Trying to add interrupt but no GPIO available");
		return -ENOTSUP;
	}

	if (!has_i2c_master(dev)) {
		return -EINVAL;
	}

	ret = setup_pin_dir(dev, access_op, pin, flags);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error setting pin direction (%d)",
			i2c_addr, ret);
		goto done;
	}

	ret = setup_pin_polarity(dev, access_op, pin, flags);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error setting pin polarity (%d)",
			i2c_addr, ret);
		goto done;
	}

	ret = setup_pin_pullupdown(dev, access_op, pin, flags);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error setting pin pull up/down "
			"(%d)", i2c_addr, ret);
		goto done;
	}

	ret = setup_pin_interrupts(dev, access_op, pin, flags);
	if (ret) {
		LOG_ERR("PCAL9535A[0x%X]: error setting pin pull up/down "
				"(%d)", i2c_addr, ret);
		goto done;
	}

done:
	return ret;
}

/**
 * @brief Set the pin or port output
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_pcal9535a_write(struct device *dev, int access_op,
				u32_t pin, u32_t value)
{
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	union gpio_pcal9535a_port_data *port = &drv_data->reg_cache.output;
	u16_t bit_mask;
	u16_t new_value;
	int ret;

	if (!has_i2c_master(dev)) {
		return -EINVAL;
	}

	/* Invert input value for pins configurated as active low. */
	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		bit_mask = BIT(pin);

		new_value = (value << pin) & bit_mask;
		new_value ^= (drv_data->out_pol_inv & bit_mask);
		new_value &= bit_mask;

		port->all &= ~bit_mask;
		port->all |= new_value;

		break;
	case GPIO_ACCESS_BY_PORT:
		port->all = value;
		bit_mask = drv_data->out_pol_inv;

		new_value = value & bit_mask;
		new_value ^= drv_data->out_pol_inv;
		new_value &= bit_mask;

		port->all &= ~bit_mask;
		port->all |= new_value;

		break;
	default:
		ret = -ENOTSUP;
		goto done;
	}

	ret = write_port_regs(dev, REG_OUTPUT_PORT0, port);

done:
	return ret;
}

/**
 * @brief Read the pin or port status
 *
 * @param dev Device struct of the PCAL9535A
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_pcal9535a_read(struct device *dev, int access_op,
			       u32_t pin, u32_t *value)
{
	union gpio_pcal9535a_port_data buf;
	int ret;

	if (!has_i2c_master(dev)) {
		return -EINVAL;
	}

	ret = read_port_regs(dev, REG_INPUT_PORT0, &buf);
	if (ret != 0) {
		goto done;
	}

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		*value = (buf.all >> pin) & 0x01;
		break;
	case GPIO_ACCESS_BY_PORT:
		*value = buf.all;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

done:
	return ret;
}


static int gpio_pcal9535a_manage_callback(struct device *dev,
									 struct gpio_callback *callback, bool set)
{
	struct gpio_pcal9535a_drv_data *data = dev->driver_data;
LOG_ERR("manage callback");
	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int gpio_pcal9535a_enable_callback(struct device *dev,
									 int access_op, u32_t pin)
{
	struct gpio_pcal9535a_drv_data *data = dev->driver_data;
	LOG_ERR("enable callback");
	if (access_op == GPIO_ACCESS_BY_PIN) {
		data->pin_callback_enables |= BIT(pin);
		LOG_ERR("access by pin: %u", pin);

		LOG_ERR("pin_callback_enables: %u", data->pin_callback_enables);
	} else {
		data->pin_callback_enables = 0xFFFFFFFF;
		LOG_ERR("access by port");

	}

	return 0;
}

static int gpio_pcal9535a_disable_callback(struct device *dev,
									  int access_op, u32_t pin)
{
	struct gpio_pcal9535a_drv_data *data = dev->driver_data;
	LOG_ERR("disable callback");
	if (access_op == GPIO_ACCESS_BY_PIN) {
		data->pin_callback_enables &= ~BIT(pin);
	} else {
		data->pin_callback_enables = 0U;
	}

	return 0;
}


static const struct gpio_driver_api gpio_pcal9535a_drv_api_funcs = {
	.config = gpio_pcal9535a_config,
	.write = gpio_pcal9535a_write,
	.read = gpio_pcal9535a_read,

	.manage_callback = gpio_pcal9535a_manage_callback,
	.enable_callback = gpio_pcal9535a_enable_callback,
	.disable_callback = gpio_pcal9535a_disable_callback,
};


static void gpio_pcal9535a_interrupt_cb(struct device *gpiob, struct gpio_callback *cb,
						   u32_t pins)
{
	ARG_UNUSED(gpiob);
	ARG_UNUSED(cb);

	pcal9535a_int_event event_msg = {
			.device = gpiob,
			.pins = pins
	};

	while (k_msgq_put(&pcal9535a_int_msgq, &event_msg, K_NO_WAIT) != 0) {
		k_msgq_purge(&pcal9535a_int_msgq);
	}
}

/**
 * @brief Initialization function of PCAL9535A
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_pcal9535a_init(struct device *dev)
{
	const struct gpio_pcal9535a_config * const config =
		dev->config->config_info;
	struct gpio_pcal9535a_drv_data * const drv_data =
		(struct gpio_pcal9535a_drv_data * const)dev->driver_data;
	struct device *i2c_master;

	/* Find out the device struct of the I2C master */
	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	drv_data->i2c_master = i2c_master;

	/* check if there is a GPIO controller for interrupt generation */
	if(config->int_gpio_dev_name)
	{
		struct device *gpio_ctrl = device_get_binding(config->int_gpio_dev_name);
		if(!gpio_ctrl)
		{
			return -ENODEV;
		}

		LOG_ERR("Interrupt at pin %u of %s", config->int_gpio_pin, log_strdup(config->int_gpio_dev_name));
		gpio_pin_configure(gpio_ctrl, config->int_gpio_pin, GPIO_DIR_IN | GPIO_INT | GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP | GPIO_INT_EDGE);

		gpio_init_callback(&pcal9535a_gpio_cb, gpio_pcal9535a_interrupt_cb,
						   BIT(config->int_gpio_pin));

		gpio_add_callback(gpio_ctrl, &pcal9535a_gpio_cb);
		gpio_pin_enable_callback(gpio_ctrl, config->int_gpio_pin);

/*		IRQ_CONNECT(DT_INST_0_NXP_PCAL9535A_IRQ_0, DT_INST_0_NXP_PCAL9535A_IRQ_0_PRIORITY,
					gpio_pcal9535a_interrupt_cb, dev, 0);

		irq_enable(DT_INST_0_NXP_PCAL9535A_IRQ_0);
		*/
		drv_data->supports_interrupts = true;
	}


	return 0;
}

void pcal9535a_thread(int unused1, int unused2, int unused3)
{
	struct pcal9535a_int_event event_msg;

	while (1) {
		k_msgq_get(&pcal9535a_int_msgq, &event_msg, K_FOREVER);

		/* find out which actual pin triggered the interrupt */
		/* FIXME */
		struct device *src_device = device_get_binding(DT_INST_0_NXP_PCAL9535A_LABEL);
		struct gpio_pcal9535a_drv_data *data = src_device->driver_data;

		if(!src_device)
		{
			LOG_ERR("Device not found");
		} else {
			union gpio_pcal9535a_port_data buf;
			read_port_regs(src_device, REG_INT_STATUS_PORT0, &buf);

			if(buf.all != 0)
			{
				gpio_fire_callbacks(&data->callbacks, src_device, buf.all);
			}
		}

	}
}

/* Initialization for PCAL9535A_0 */
#ifdef DT_INST_0_NXP_PCAL9535A
static const struct gpio_pcal9535a_config gpio_pcal9535a_0_cfg = {
	.i2c_master_dev_name = DT_INST_0_NXP_PCAL9535A_BUS_NAME,
	.i2c_slave_addr = DT_INST_0_NXP_PCAL9535A_BASE_ADDRESS,
#ifdef DT_INST_0_NXP_PCAL9535A_INT_GPIOS_CONTROLLER
	.int_gpio_dev_name = DT_INST_0_NXP_PCAL9535A_INT_GPIOS_CONTROLLER,
	.int_gpio_pin = DT_INST_0_NXP_PCAL9535A_INT_GPIOS_PIN,
#endif
};

static struct gpio_pcal9535a_drv_data gpio_pcal9535a_0_drvdata = {
	/* Default for registers according to datasheet */
	.reg_cache.output = { .all = 0xFFFF },
	.reg_cache.pol_inv = { .all = 0x0 },
	.reg_cache.dir = { .all = 0xFFFF },
	.reg_cache.pud_en = { .all = 0x0 },
	.reg_cache.pud_sel = { .all = 0xFFFF },
	.reg_cache.int_en = { .all = 0xFFFF }
};

/* This has to init after I2C master */
DEVICE_AND_API_INIT(gpio_pcal9535a_0, DT_INST_0_NXP_PCAL9535A_LABEL,
	    gpio_pcal9535a_init,
	    &gpio_pcal9535a_0_drvdata, &gpio_pcal9535a_0_cfg,
	    POST_KERNEL, CONFIG_GPIO_PCAL9535A_INIT_PRIORITY,
	    &gpio_pcal9535a_drv_api_funcs);

#endif /* CONFIG_GPIO_PCAL9535A_0 */

/* Initialization for PCAL9535A_1 */
#ifdef CONFIG_GPIO_PCAL9535A_1
static const struct gpio_pcal9535a_config gpio_pcal9535a_1_cfg = {
	.i2c_master_dev_name = CONFIG_GPIO_PCAL9535A_1_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_GPIO_PCAL9535A_1_I2C_ADDR,
};

static struct gpio_pcal9535a_drv_data gpio_pcal9535a_1_drvdata = {
	/* Default for registers according to datasheet */
	.reg_cache.output = { .all = 0xFFFF },
	.reg_cache.pol_inv = { .all = 0x0 },
	.reg_cache.dir = { .all = 0xFFFF },
	.reg_cache.pud_en = { .all = 0x0 },
	.reg_cache.pud_sel = { .all = 0xFFFF },
	.reg_cache.int_en = { .all = 0xFFFF }
};

/* This has to init after I2C master */
DEVICE_AND_API_INIT(gpio_pcal9535a_1, CONFIG_GPIO_PCAL9535A_1_DEV_NAME,
	    gpio_pcal9535a_init,
	    &gpio_pcal9535a_1_drvdata, &gpio_pcal9535a_1_cfg,
	    POST_KERNEL, CONFIG_GPIO_PCAL9535A_INIT_PRIORITY,
	    &gpio_pcal9535a_drv_api_funcs);

#endif /* CONFIG_GPIO_PCAL9535A_1 */

/* Initialization for PCAL9535A_2 */
#ifdef CONFIG_GPIO_PCAL9535A_2
static const struct gpio_pcal9535a_config gpio_pcal9535a_2_cfg = {
	.i2c_master_dev_name = CONFIG_GPIO_PCAL9535A_2_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_GPIO_PCAL9535A_2_I2C_ADDR,
};

static struct gpio_pcal9535a_drv_data gpio_pcal9535a_2_drvdata = {
	/* Default for registers according to datasheet */
	.reg_cache.output = { .all = 0xFFFF },
	.reg_cache.pol_inv = { .all = 0x0 },
	.reg_cache.dir = { .all = 0xFFFF },
	.reg_cache.pud_en = { .all = 0x0 },
	.reg_cache.pud_sel = { .all = 0xFFFF },
	.reg_cache.int_en = { .all = 0xFFFF }
};

/* This has to init after I2C master */
DEVICE_AND_API_INIT(gpio_pcal9535a_2, CONFIG_GPIO_PCAL9535A_2_DEV_NAME,
	    gpio_pcal9535a_init,
	    &gpio_pcal9535a_2_drvdata, &gpio_pcal9535a_2_cfg,
	    POST_KERNEL, CONFIG_GPIO_PCAL9535A_INIT_PRIORITY,
	    &gpio_pcal9535a_drv_api_funcs);

#endif /* CONFIG_GPIO_PCAL9535A_2 */

/* Initialization for PCAL9535A_3 */
#ifdef CONFIG_GPIO_PCAL9535A_3
static const struct gpio_pcal9535a_config gpio_pcal9535a_3_cfg = {
	.i2c_master_dev_name = CONFIG_GPIO_PCAL9535A_3_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_GPIO_PCAL9535A_3_I2C_ADDR,
};

static struct gpio_pcal9535a_drv_data gpio_pcal9535a_3_drvdata = {
	/* Default for registers according to datasheet */
	.reg_cache.output = { .all = 0xFFFF },
	.reg_cache.pol_inv = { .all = 0x0 },
	.reg_cache.dir = { .all = 0xFFFF },
	.reg_cache.pud_en = { .all = 0x0 },
	.reg_cache.pud_sel = { .all = 0xFFFF },
	.reg_cache.int_en = { .all = 0xFFFF }
};

/* This has to init after I2C master */
DEVICE_AND_API_INIT(gpio_pcal9535a_3, CONFIG_GPIO_PCAL9535A_3_DEV_NAME,
	    gpio_pcal9535a_init,
	    &gpio_pcal9535a_3_drvdata, &gpio_pcal9535a_3_cfg,
	    POST_KERNEL, CONFIG_GPIO_PCAL9535A_INIT_PRIORITY,
	    &gpio_pcal9535a_drv_api_funcs);

#endif /* CONFIG_GPIO_PCAL9535A_3 */

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5



K_THREAD_DEFINE(pcal9535a_tid, MY_STACK_SIZE,
				pcal9535a_thread, NULL, NULL, NULL,
				MY_PRIORITY, 0, K_NO_WAIT);