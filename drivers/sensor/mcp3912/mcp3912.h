/*
 * Copyright (c) 2017 IpTronix S.r.l.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP3912_MCP3912_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP3912_MCP3912_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#define MCP3912_DEACTIVATE 0
#define MCP3912_ACTIVATE 1
#define MCP3912_ENABLE_0   0x000E0000
#define MCP3912_ENABLE_1   0x000D0000
#define MCP3912_ENABLE_2   0x000B0000
#define MCP3912_ENABLE_3   0x00070000
#define MCP3912_DISABLE_0  0x00010000
#define MCP3912_DISABLE_1  0x00020000
#define MCP3912_DISABLE_2  0x00040000
#define MCP3912_DISABLE_3  0x00080000

#define MCP3912_ADD  0x40
#define MCP3912_READ  0x01
#define MCP3912_WRITE  0x00

#define MCP3912_DEV_ADD  0x40

#define MCP3912_CHAN_0  0x00       //00   |        | -GROUP
#define MCP3912_CHAN_1  0x02       //01   |-TYPE  _|
#define MCP3912_CHAN_2  0x04       //02   |        | -GROUP
#define MCP3912_CHAN_3  0x06       //03  _|       _|

#define MCP3912_MOD_VAL  0x10      //08   |        |
#define MCP3912_PHASE  0x14        //0A   |        |-GROUP
#define MCP3912_GAIN  0x16         //0B   |       _|
#define MCP3912_STATUSCOM  0x18    //0C   |        |
#define MCP3912_CONFIG_0  0x1A     //0D   |        |-GROUP
#define MCP3912_CONFIG_1  0x1C     //0E   |       _|
#define MCP3912_OFFCAL_0  0x1E     //0F   |-TYPE   | -GROUP
#define MCP3912_GAINCAL_0  0x20    //10   |       _|
#define MCP3912_OFFCAL_1  0x22     //11   |        | -GROUP
#define MCP3912_GAINCAL_1  0x24    //12   |       _|
#define MCP3912_OFFCAL_2  0x26     //13   |        | -GROUP
#define MCP3912_GAINCAL_2  0x28    //14   |       _|
#define MCP3912_OFFCAL_3  0x2A     //15   |        | -GROUP
#define MCP3912_GAINCAL_3  0x2C    //16   |       _|
#define MCP3912_LOK_CRC  0x3E      //1F  _|        >-GROUP

#define MCP3912_GAIN_1  0x00,0x00,0x00
#define MCP3912_GAIN_2  0x00,0x02,0x49
#define MCP3912_GAIN_4  0x00,0x04,0x92
#define MCP3912_GAIN_8  0x00,0x06,0xCB
#define MCP3912_GAIN_16 0x00,0x09,0x24
#define MCP3912_GAIN_32 0x00,0x0B,0x6D

enum SAMPLE_RATE {
	SAMPLE_RATE_25600,
	SAMPLE_RATE_12800,
	SAMPLE_RATE_6400,
	SAMPLE_RATE_3200,
	SAMPLE_RATE_1600,
	SAMPLE_RATE_800,
	SAMPLE_RATE_400,
	SAMPLE_RATE_200
};

struct mcp3912_config {
	char *spi_name;
	uint32_t spi_max_frequency;
	uint16_t spi_slave;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	const char *gpio_cs_port;
	gpio_pin_t cs_gpio;
	gpio_dt_flags_t cs_flags;
#endif

	const char *gpio_drdy_port;
	gpio_pin_t drdy_gpio;
	gpio_dt_flags_t drdy_flags;
	// uint8_t int1_config;

	const char *gpio_rst_port;
	gpio_pin_t rst_gpio;
	gpio_dt_flags_t rst_flags;
};

struct mcp3912_data {
	const struct device *spi;
	struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	struct spi_cs_control mcp3912_cs_ctrl;
#endif
	
	int16_t channelData[4];

	const struct device *rst_gpio;

	const struct device *dev;
	const struct device *drdy_gpio;
	struct gpio_callback drdy_gpio_cb;
	struct k_mutex trigger_mutex;

	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;

#if defined(CONFIG_MCP3912_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MCP3912_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_MCP3912_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

};

int mcp3912_init_interrupt(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP3912_MCP3912_H_ */
