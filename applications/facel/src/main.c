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
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <settings/settings.h>

#include <stdio.h>
#include <string.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>

#include <usb/usb_device.h>
#include <logging/log.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN		DT_GPIO_PIN(LED0_NODE,   gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)

LOG_MODULE_REGISTER(facel, LOG_LEVEL_INF);
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

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", log_strdup(addr));

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

static struct bt_conn_auth_cb conn_auth_callbacks;

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", log_strdup(addr));
        for(int i=0; i<len; i++)
        {
          printk("0x%02X ",data[i]);
        }
        printk("\n");

        uint8_t buf[30]={0};
        memcpy(buf,data,len);

        printk("%s\n", buf);

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};



char buf[50];
int n=0;
const struct device *cdc;
const struct device *dev_led;

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
		uart_irq_tx_enable(cdc);
	}
}

static void accel_handler(const struct device *dev, struct sensor_trigger *trig)
{
	fetch_and_display(dev);
}

static void afe_handler(const struct device *dev, struct sensor_trigger *trig)
{
        gpio_pin_set(dev_led, 14, true);
	struct sensor_value v[4];
	
	int rc = sensor_sample_fetch(dev);
	if (rc == -EBADMSG) {
		printk("OVERRUN\n");
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev,
					SENSOR_CHAN_VOLTAGE,
					v);
	}
	if (rc < 0) {
		printk("ERROR: Update failed\n");
	} else {
		n=sprintf(buf,"1:%i 2:%i ",
			v[0].val1,
			v[1].val1);
		//ring_buf_put(&ringbuf, buf, n);
		//uart_irq_tx_enable(cdc);
                bt_nus_send(NULL, buf, n);
                n=sprintf(buf,"3:%i 4:%i\n",
			v[2].val1,
			v[3].val1);
                bt_nus_send(NULL, buf, n);
	}
        gpio_pin_set(dev_led, 14, false);
}


void main(void)
{
      int rc, err;
      // ---------------------------------------------------------- BT Gatt Uart

      bt_conn_cb_register(&conn_callbacks);

      if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
              bt_conn_auth_cb_register(&conn_auth_callbacks);
      }

      err = bt_enable(NULL);
      if (err) {
              printk("Failed to enable BT\n");
              return;
      }

      LOG_INF("Bluetooth initialized");

      k_sem_give(&ble_init_ok);

      if (IS_ENABLED(CONFIG_SETTINGS)) {
              settings_load();
      }

      err = bt_nus_init(&nus_cb);
      if (err) {
              LOG_ERR("Failed to initialize UART service (err: %d)", err);
              return;
      }

      err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
                            ARRAY_SIZE(sd));
      if (err) {
              LOG_ERR("Advertising failed to start (err %d)", err);
      }

      printk("Starting Nordic UART service example\n");

      // ---------------------------------------------------------- USB serial
      int ret;
      cdc = device_get_binding("CDC_ACM_0");
      if (!cdc) {
              printk("CDC ACM device not found\n");
              return;
      }
      ret = usb_enable(NULL);
      if (ret != 0) {
              printk("Failed to enable USB\n");
              return;
      }
      ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
      uart_irq_callback_set(cdc, interrupt_handler);
      /* Enable rx interrupts */
      uart_irq_rx_enable(cdc);

      // ---------------------------------------------------------- Led
      dev_led = device_get_binding(LED0);
      if (dev_led == NULL) {
              return;
      }

      ret = gpio_pin_configure(dev_led, 14, GPIO_OUTPUT_ACTIVE);//led
      if (ret < 0) {
              return;
      }

      gpio_pin_set(dev_led, 14, false);

      // ---------------------------------------------------------- Accel
      //const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(accel));
      //if (accel == NULL) {
      //        printk("No device found\n");
      //        return;
      //}
      //if (!device_is_ready(accel)) {
      //        printk("Device %s is not ready\n", accel->name);
      //        return;
      //}

      //struct sensor_trigger accel_trig;

      //accel_trig.type = SENSOR_TRIG_DATA_READY;
      //accel_trig.chan = SENSOR_CHAN_ACCEL_XYZ;

      //if (IS_ENABLED(CONFIG_LIS2DW12_ODR_RUNTIME)) {
      //        struct sensor_value odr = {
      //                .val1 = 2,
      //        };

      //        rc = sensor_attr_set(accel, accel_trig.chan,
      //                                        SENSOR_ATTR_SAMPLING_FREQUENCY,
      //                                        &odr);
      //        if (rc != 0) {
      //                printk("Failed to set odr: %d\n", rc);
      //                return;
      //        }
      //        printk("Sampling at %u Hz\n", odr.val1);
      //}

      //rc = sensor_trigger_set(accel, &accel_trig, accel_handler);
      //if (rc != 0) {
      //        printk("Failed to set trigger: %d\n", rc);
      //}

      // ---------------------------------------------------------- Analog frontend
      const struct device *afe = DEVICE_DT_GET(DT_NODELABEL(afe));
      if (afe == NULL) {
              printk("No afe device found\n");
              return;
      }
      if (!device_is_ready(afe)) {
              printk("Device %s is not ready\n", afe->name);
              return;
      }

      struct sensor_trigger afe_trig;

      afe_trig.type = SENSOR_TRIG_DATA_READY;
      afe_trig.chan = SENSOR_CHAN_VOLTAGE;

      rc = sensor_trigger_set(afe, &afe_trig, afe_handler);
      if (rc != 0) {
              printk("Failed to set afe trigger: %d\n", rc);
      }

      printk("Waiting for triggers\n");

      // ---------------------------------------------------------- Loop forever
      while (true) {
              k_sleep(K_MSEC(1000));
      }
}

//void ble_write_thread(void)
//{
//	/* Don't go any further until BLE is initialized */
//	k_sem_take(&ble_init_ok, K_FOREVER);

//	for (;;) {
//		/* Wait indefinitely for data to be sent over bluetooth */
//		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
//						     K_FOREVER);

//		if (bt_nus_send(NULL, buf->data, buf->len)) {
//			LOG_WRN("Failed to send data over BLE connection");
//		}

//		k_free(buf);
//	}
//}

//K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
//		NULL, PRIORITY, 0, 0);