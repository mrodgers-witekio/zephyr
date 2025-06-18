/** @file
 *  @brief HoG Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/logging/log_core.h"
#include "zephyr/sys/util.h"
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/usb/class/hid.h>

LOG_MODULE_REGISTER(keyboard, LOG_LEVEL_DBG);

static K_SEM_DEFINE(gpio_sem, 0, 1);

enum {
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
	uint16_t version; /* version number of base USB HID Specification */
	uint8_t code; /* country HID Device hardware is localized for. */
	uint8_t flags;
} __packed;

struct hids_report {
	uint8_t id; /* report id */
	uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
	.version = 0x0000,
	.code = 0x00,
	.flags = HIDS_NORMALLY_CONNECTABLE,
};

enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
	.id = 0x01,
	.type = HIDS_INPUT,
};

static uint8_t subscribed;
static uint8_t ctrl_point;
static uint8_t report_map[] = HID_KEYBOARD_REPORT_DESC();


static ssize_t read_info(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, void *buf,
			  uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,
				 sizeof(report_map));
}

static ssize_t read_report(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_report));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	subscribed = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_input_report(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset,
				uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(ctrl_point)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

/* Require encryption. */
#define SAMPLE_BT_PERM_READ BT_GATT_PERM_READ_ENCRYPT
#define SAMPLE_BT_PERM_WRITE BT_GATT_PERM_WRITE_ENCRYPT

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(hog_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_info, NULL, &info),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_report_map, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       SAMPLE_BT_PERM_READ,
			       read_input_report, NULL, NULL),
	BT_GATT_CCC(input_ccc_changed,
		    SAMPLE_BT_PERM_READ | SAMPLE_BT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ,
			   read_report, NULL, &input),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, write_ctrl_point, &ctrl_point),
);

#define SWITCH_COUNT 4
BUILD_ASSERT(SWITCH_COUNT <= 6, "Simple implementation can't support more than 6 keys");

static const struct gpio_dt_spec switches[SWITCH_COUNT] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios),
};

static const uint8_t keycodes[SWITCH_COUNT] = {
	0x04, /* A */
	0x05, /* B */
	0x06, /* C */
	0x07, /* D */
};

static struct gpio_callback gpio_cb_data;

void on_gpio_edge(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_sem_give(&gpio_sem);
}


int hog_init(void)
{
	/* For simplicity we make (and verify) the assumption that all pins are on the same port */
	const struct device *port = switches[0].port;
	uint32_t pins = 0;

	for (uint32_t i = 0; i < SWITCH_COUNT; i++) {
		const struct gpio_dt_spec *spec = &switches[i];

		if (!gpio_is_ready_dt(spec)) {
			LOG_ERR("GPIO is not ready");
			return -ENODEV;
		}

		if (spec->port != port) {
			LOG_ERR("All GPIOs must be on the same port");
			return -EINVAL;
		}

		if (0 != gpio_pin_configure_dt(&switches[i], GPIO_INPUT)) {
			LOG_ERR("Failed to configure GPIO pin");
			return -EINVAL;
		}

		if (0 != gpio_pin_interrupt_configure_dt(&switches[i], GPIO_INT_EDGE_BOTH)) {
			LOG_ERR("Failed to configure GPIO pin interrupt");
			return -EINVAL;
		}

		pins |= BIT(spec->pin);
	}

	gpio_init_callback(&gpio_cb_data, on_gpio_edge, pins);

	if (0 != gpio_add_callback(port, &gpio_cb_data)) {
		LOG_ERR("Failed to add GPIO callback");
		return -EINVAL;
	}

	return 0;
}

void hog_button_loop(void)
{
	while (1) {
		k_sem_take(&gpio_sem, K_FOREVER);

		/* Ignore any further GPIO events in the next xx ms as a crude debounce strategy */
		k_sem_take(&gpio_sem, K_MSEC(30));
		LOG_INF("Button event");

		if (subscribed) {
			/* HID Report:
			 *   See report layout here: https://wiki.osdev.org/USB_Human_Interface_Devices#USB_keyboard
			 *   See key codes here: https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf
			 * Byte 0: Modifier keys (we don't have any, so zero)
			 * Byte 1: Reserved (zero)
			 * Byte 2: Keypress #1
			 * Byte 3: Keypress #2
			 * Byte 4: Keypress #3
			 * Byte 5: Keypress #4
			 * Byte 6: Keypress #5
			 * Byte 7: Keypress #6
			 */
			uint8_t report[8] = {0};
			size_t index = 2;

			for (uint32_t i = 0; i < SWITCH_COUNT; i++) {
				if (gpio_pin_get_dt(&switches[i])) {
					report[index] = keycodes[i];
					index++;
				}
			}

			bt_gatt_notify(NULL, &hog_svc.attrs[5],
				       report, sizeof(report));
		}
	}
}
