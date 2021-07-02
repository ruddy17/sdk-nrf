/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include <stdio.h>
#include <string.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>

#include <usb/usb_device.h>
#include <logging/log.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN		DT_GPIO_PIN(LED0_NODE,   gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);
#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];
struct ring_buf ringbuf;

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				printk("Drop %u bytes\n", recv_len - rb_len);
			}

			//printk("tty fifo -> ringbuf %d bytes\n", rb_len);

			uart_irq_tx_enable(dev);
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				printk("Ring buffer empty, disable TX IRQ\n");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				printk("Drop %d bytes\n", rb_len - send_len);
			}

			//printk("ringbuf -> tty fifo %d bytes\n", send_len);
		}
	}
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16,
		      0xaa, 0xfe, /* Eddystone UUID */
		      0x10, /* Eddystone-URL frame type */
		      0x00, /* Calibrated Tx power at 0m */
		      0x00, /* URL Scheme Prefix http://www. */
		      'f', 'a', 'c', 'e', 'l', 
		      0x00) /* .org */
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	/* For connectable advertising you would use
	 * bt_le_oob_get_local().  For non-connectable non-identity
	 * advertising an non-resolvable private address is used;
	 * there is no API to retrieve that.
	 */

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	printk("Beacon started, advertising as %s\n", addr_s);
}

char buf[50];
int n=0;
const struct device *dev;

static void fetch_and_display(const struct device *sensor)
{
	static unsigned int count;
	struct sensor_value accel[3];
	
	int rc = sensor_sample_fetch(sensor);
	++count;
	if (rc == -EBADMSG) {
		printk("OVERRUN\n");
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printk("ERROR: Update failed\n");
	} else {
		n = sprintf(buf,"x:%f y:%f z:%f\n",
			sensor_value_to_double(&accel[0]),
			sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));
		ring_buf_put(&ringbuf, buf, n);
		uart_irq_tx_enable(dev);
	}
}

static void trigger_handler(const struct device *dev, struct sensor_trigger *trig)
{
	fetch_and_display(dev);
}


void main(void)
{
      // ---------------------------------------------------------- BT beacon
      int err;
      printk("Starting Beacon Demo\n");
      /* Initialize the Bluetooth Subsystem */
      err = bt_enable(bt_ready);
      if (err) {
              printk("Bluetooth init failed (err %d)\n", err);
      }
      // ---------------------------------------------------------- USB serial
      int ret;
      dev = device_get_binding("CDC_ACM_0");
      if (!dev) {
              printk("CDC ACM device not found\n");
              return;
      }
      ret = usb_enable(NULL);
      if (ret != 0) {
              printk("Failed to enable USB\n");
              return;
      }
      ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
      uart_irq_callback_set(dev, interrupt_handler);
      /* Enable rx interrupts */
      uart_irq_rx_enable(dev);

      // ---------------------------------------------------------- Blink
      const struct device *dev_led;
      dev_led = device_get_binding(LED0);
      if (dev_led == NULL) {
              return;
      }

      ret = gpio_pin_configure(dev_led, 14, GPIO_OUTPUT_ACTIVE);//led
      if (ret < 0) {
              return;
      }

      gpio_pin_set(dev_led, 14, true);

      // ---------------------------------------------------------- Accel
      const struct device *sensor = device_get_binding(DT_LABEL(DT_N_S_soc_S_spi_40003000_S_lis2dw12_0));
      if (sensor == NULL) {
              printk("No device found\n");
              return;
      }
      if (!device_is_ready(sensor)) {
              printk("Device %s is not ready\n", sensor->name);
              return;
      }

      struct sensor_trigger trig;
      int rc;

      trig.type = SENSOR_TRIG_DATA_READY;
      trig.chan = SENSOR_CHAN_ACCEL_XYZ;

      if (IS_ENABLED(CONFIG_LIS2DW12_ODR_RUNTIME)) {
              struct sensor_value odr = {
                      .val1 = 2,
              };

              rc = sensor_attr_set(sensor, trig.chan,
                                              SENSOR_ATTR_SAMPLING_FREQUENCY,
                                              &odr);
              if (rc != 0) {
                      printk("Failed to set odr: %d\n", rc);
                      return;
              }
              printk("Sampling at %u Hz\n", odr.val1);
      }

      rc = sensor_trigger_set(sensor, &trig, trigger_handler);
      if (rc != 0) {
              printk("Failed to set trigger: %d\n", rc);
      }

      printk("Waiting for triggers\n");
      while (true) {
              gpio_pin_toggle(dev_led, 14);
              k_sleep(K_MSEC(1000));
      }
}
