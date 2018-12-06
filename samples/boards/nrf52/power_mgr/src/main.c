/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(app, 4);
#include "test.h"

#define BUSY_WAIT_DELAY_US		(10 * 1000000)

#define LPS_STATE_ENTER_TO		(30 * 1000)
#define LPS1_STATE_ENTER_TO		(90 * 1000)
#define DEEP_SLEEP_STATE_ENTER_TO	(120 * 1000)

#define DEMO_DESCRIPTION	\
	"Demo Description\n"	\
	"Application creates Idleness, Due to which System Idle Thread is\n"\
	"scheduled and it enters into various Low Power States.\n"\

void gpio_setup(void);
void create_device_list(void);

void sys_pm_notify_lps_entry(enum power_states state)
{
	LOG_DBG("Entering Low Power state (%d)\n", state);
}

void sys_pm_notify_lps_exit(enum power_states state)
{
	LOG_DBG("Exiting Low Power state (%d)\n", state);
}

/* Application main Thread */
void main(void)
{
	struct device *gpio_in;
	u32_t level = 0U;

	LOG_DBG("\n\n***Power Management Demo on %s****\n", CONFIG_ARCH);
	LOG_DBG(DEMO_DESCRIPTION);

	//gpio_setup();
#ifdef CONFIG_PM_CONTROL_APP
create_device_list();
#endif
	//gpio_in = device_get_binding(PORT);

//#ifdef CONFIG_GPIO
	struct device *gpioa = device_get_binding(DT_GPIO_P0_DEV_NAME);
	gpio_pin_configure(gpioa, 17, GPIO_DIR_OUT);
	gpio_pin_write(gpioa, 17, 1);
//#endif
	//struct device *gpioa = device_get_binding(CONFIG_GPIO_P0_DEV_NAME);

	// disable modem
	/*
	u32_t statValue = 0;
	gpio_pin_configure(gpioa, 2, GPIO_DIR_IN);
	gpio_pin_read(gpioa, 2, &statValue);
	if(statValue > 0)
	{
		gpio_pin_write(gpioa, 16, 0);
		k_busy_wait(50);
		gpio_pin_write(gpioa, 16, 1);
	}*/

	// disable spi flash
	//gpio_pin_configure(gpioa, 17, GPIO_DIR_OUT);
	//gpio_pin_write(gpioa, 17, 1);


	for (int i = 1; i <= 4; i++) {
		LOG_DBG("\n<-- App doing busy wait for 10 Sec -->\n");
		k_busy_wait(BUSY_WAIT_DELAY_US);

		/* Create Idleness to make Idle thread run */
		if ((i % 2) == 0) {
			LOG_DBG("\n<-- App going to sleep for 90 Sec -->\n");
			k_sleep(LPS1_STATE_ENTER_TO);
		} else {
			LOG_DBG("\n<-- App going to sleep for 30 Sec -->\n");
			k_sleep(LPS_STATE_ENTER_TO);
		}

	}

	LOG_DBG("\nPress BUTTON1 to enter into Deep Dleep state..."
		   "Press BUTTON2 to exit Deep Sleep state\n");
	while (1) {
		gpio_pin_read(gpio_in, BUTTON_1, &level);
		if (level == LOW) {
			k_sleep(DEEP_SLEEP_STATE_ENTER_TO);
		}
		k_busy_wait(1000);
	}
}