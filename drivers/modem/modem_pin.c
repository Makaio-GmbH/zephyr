/** @file
 * @brief Modem pin setup for modem context driver
 *
 * GPIO-based pin handling for the modem context driver
 */

/*
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>

#include "modem_context.h"

int modem_pin_read(struct modem_pin* mpin)
{
	if (mpin == NULL) {
		return -ENODEV;
	}

	return gpio_pin_get(mpin->gpio_port_dev,
						mpin->pin);
}

int modem_pin_write(struct modem_pin* mpin, uint32_t value)
{
	if (mpin == NULL) {
		return -ENODEV;
	}

	return gpio_pin_set(mpin->gpio_port_dev,
						mpin->pin, value);
}

int modem_pin_config(struct modem_pin* mpin, bool enable)
{


	if (mpin == NULL) {
		return -ENODEV;
	}

	return gpio_pin_configure(mpin->gpio_port_dev,
							  mpin->pin,
				  enable ? mpin->init_flags :
					   GPIO_INPUT);
}

int modem_pins_init(struct modem_context *ctx)
{
	struct modem_pins *pins = ctx->pins;
	int ret;

	MODEM_PIN_INIT(pins, power);
	MODEM_PIN_INIT(pins, reset);
	MODEM_PIN_INIT(pins, vint);
	MODEM_PIN_INIT(pins, wake);

	return 0;
}
