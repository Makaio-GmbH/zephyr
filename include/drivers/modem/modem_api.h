/** @file
 * @brief Modem API header file.
 *
 * A modem driver allowing application to control
 * modem devices.
 */

/*
 * Copyright (c) 2019 Makaio GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_API_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_API_H_
#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief Modem sleep modes
*/
enum modem_sleep_mode {
	/** No sleep, stay awake */
		MODEM_SLEEP_NONE,
	/** Low power, stay connected, uart/interrupts enabled */
		MODEM_SLEEP_CONNECTED,
	/** Lower power, stay connected, uart/interrupts disabled */
		MODEM_SLEEP_DEEP,
	/** Usually AT+CFUN=0 */
		MODEM_SLEEP_OFF,
};

typedef void (*modem_api_resp_cb)(struct device *dev, u8_t *data, u16_t len);

typedef int (*modem_api_wake)(struct device *dev);
typedef int (*modem_api_sleep)(struct device *dev, enum modem_sleep_mode sleep_mode);
typedef int (*modem_api_register_resp_cb)(struct device *dev, modem_api_resp_cb cb);

struct modem_driver_api {
	void (*init)(struct net_if *iface);
	modem_api_sleep sleep;
	modem_api_wake wake;
	modem_api_register_resp_cb register_resp_cb;
};

#ifdef __cplusplus
}
#endif
#endif //ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_API_H_
