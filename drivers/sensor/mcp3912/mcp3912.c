/* mcp3912.c - MCP3912 4-Channel Analog Front End */

#define DT_DRV_COMPAT microchip_mcp3912

#include <kernel.h>
#include <string.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <drivers/spi.h>
#include <logging/log.h>
#include <device.h>
#include "mcp3912.h"

LOG_MODULE_REGISTER(MCP3912, CONFIG_SENSOR_LOG_LEVEL);

static struct mcp3912_data mcp3912_data;

static int mcp3912_reg_access(const struct device *dev, uint8_t cmd,
			      uint8_t address, void *message, size_t length)
{
	struct mcp3912_data *data = dev->data;

	uint8_t command = MCP3912_DEV_ADD | address | cmd;
	const struct spi_buf buf[2] = {
		{
			.buf = &command,
			.len = 1
		},
		{
			.buf = message,
			.len = length
		}
	};
	struct spi_buf_set tx = {
		.buffers = buf,
	};

	if (cmd == MCP3912_READ) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		tx.count = 1;

		return spi_transceive(data->spi, &data->spi_cfg, &tx, &rx);
	}

	tx.count = 2;

	return spi_write(data->spi, &data->spi_cfg, &tx);
}

static inline int mcp3912_set_reg(const struct device *dev,
				  uint8_t *write_buf,
				  uint8_t register_address, uint8_t count)
{
	return mcp3912_reg_access(dev,
				  MCP3912_WRITE,
				  register_address,
				  write_buf,
				  count);
}

static inline int mcp3912_get_reg(const struct device *dev, uint8_t *read_buf,
				  uint8_t register_address, uint8_t count)
{
	return mcp3912_reg_access(dev,
				  MCP3912_READ,
				  register_address,
				  read_buf, count);
}

int mcp3912_get_status(const struct device *dev, uint8_t *status)
{
	return mcp3912_get_reg(dev, status, MCP3912_STATUSCOM, 3);
}

static int mcp3912_reset(const struct device *dev)
{
	const struct mcp3912_config *config = dev->config;
	struct mcp3912_data *data = dev->data;

	gpio_pin_set(data->rst_gpio, config->rst_gpio, false);
	k_sleep(K_USEC(100));
	gpio_pin_set(data->rst_gpio, config->rst_gpio, true);
        k_sleep(K_USEC(100));
	return 0;
}

static int mcp3912_set_power_mode(const struct device *dev, uint8_t mode)
{
	// uint8_t old_power_ctl;
	// uint8_t new_power_ctl;
	// int ret;

	// ret = mcp3912_get_reg(dev, &old_power_ctl, MCP3912_REG_POWER_CTL, 1);
	// if (ret) {
	// 	return ret;
	// }

	// new_power_ctl = old_power_ctl & ~MCP3912_POWER_CTL_MEASURE(0x3);
	// new_power_ctl = new_power_ctl |
	// 	      (mode *
	// 	       MCP3912_POWER_CTL_MEASURE(MCP3912_MEASURE_ON));
	// return mcp3912_set_reg(dev, new_power_ctl, MCP3912_REG_POWER_CTL, 1);
	return 0;
}



// static int adxl362_acc_config(const struct device *dev,
// 			     enum sensor_channel chan,
// 			     enum sensor_attribute attr,
// 			     const struct sensor_value *val)
// {
// 	switch (attr) {
// #if defined(CONFIG_MCP3912_ACCEL_RANGE_RUNTIME)
// 	case SENSOR_ATTR_FULL_SCALE:
// 	{
// 		int range_reg;

// 		range_reg = mcp3912_range_to_reg_val(sensor_ms2_to_g(val));
// 		if (range_reg < 0) {
// 			LOG_DBG("invalid range requested.");
// 			return -ENOTSUP;
// 		}

// 		return mcp3912_set_range(dev, range_reg);
// 	}
// 	break;
// #endif
// #if defined(CONFIG_MCP3912_ACCEL_ODR_RUNTIME)
// 	case SENSOR_ATTR_SAMPLING_FREQUENCY:
// 	{
// 		int out_rate;

// 		out_rate = mcp3912_freq_to_odr_val(val->val1,
// 						   val->val2 / 1000);
// 		if (out_rate < 0) {
// 			LOG_DBG("invalid output rate.");
// 			return -ENOTSUP;
// 		}

// 		return mcp3912_set_output_rate(dev, out_rate);
// 	}
// 	break;
// #endif
// 	default:
// 		LOG_DBG("Accel attribute not supported.");
// 		return -ENOTSUP;
// 	}

// 	return 0;
// }

static int mcp3912_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	// switch (attr) {
	// case SENSOR_ATTR_UPPER_THRESH:
	// case SENSOR_ATTR_LOWER_THRESH:
	// 	return mcp3912_attr_set_thresh(dev, chan, attr, val);
	// default:
	// 	/* Do nothing */
	// 	break;
	// }

	// switch (chan) {
	// case SENSOR_CHAN_ACCEL_X:
	// case SENSOR_CHAN_ACCEL_Y:
	// case SENSOR_CHAN_ACCEL_Z:
	// case SENSOR_CHAN_ACCEL_XYZ:
	// 	return axl362_acc_config(dev, chan, attr, val);
	// default:
	// 	LOG_DBG("attr_set() not supported on this channel.");
	// 	return -ENOTSUP;
	// }

	return 0;
}

static int mcp3912_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mcp3912_data *data = dev->data;
	uint8_t buf[4*2];
	int ret;

	ret = mcp3912_get_reg(dev, buf, MCP3912_CHAN_0, sizeof(buf));
	if (ret) {
		return ret;
	}

	//data->channelData[0] = (int16_t)(buf[0]<<8 | buf[1]);
        data->channelData[0] = 0;
	data->channelData[1] = (int16_t)(buf[2]<<8 | buf[3]);
	data->channelData[2] = (int16_t)(buf[4]<<8 | buf[5]);
	data->channelData[3] = (int16_t)(buf[6]<<8 | buf[7]);

	return 0;
}

static int mcp3912_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mcp3912_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		for (size_t i = 0; i < 4; i++) {
			val[i].val1 = data->channelData[i];
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

uint8_t temp_buf[12];

static int mcp3912_chip_init(const struct device *dev)
{
	int ret;

        //mcp3912_get_status(dev,temp_buf);

	uint8_t buf[]={
		MCP3912_GAIN_1,							// GAIN_1, _2, _4, _8, _16, _32
		0xB8, 0x00, 0x0F,						// STATUSCOM: auto increment TYPES, DR in High
		0x3c, 0x00|(SAMPLE_RATE_200 << 5), 0x50,                        // CONFIG_0:  dither on max, boost 2x, mclk/8, OSR 4096,
		0x0F, 0x00, 0x00 						// CONFIG_1:  put the ADCs in reset, external oscillator
	};

	ret = mcp3912_set_reg(dev, buf, MCP3912_GAIN, sizeof(buf));
        if(ret){
          return -ENODEV;
        }
        //mcp3912_get_reg(dev,temp_buf,MCP3912_GAIN,sizeof(temp_buf));

        uint8_t channel_enable[]={
		0x00, 0x00, 0x00	// All channels enable
	};
        ret = mcp3912_set_reg(dev, channel_enable, MCP3912_CONFIG_1, sizeof(channel_enable));
        if(ret){
          return -ENODEV;
        }
        
        return 0;
}

/**
 * @brief Initializes communication with the device and checks if the part is
 *        present by reading the device id.
 *
 * @return  0 - the initialization was successful and the device is present;
 *         -1 - an error occurred.
 *
 */
static int mcp3912_init(const struct device *dev)
{
	const struct mcp3912_config *config = dev->config;
	struct mcp3912_data *data = dev->data;
	int err;

	data->spi = device_get_binding(config->spi_name);
	if (!data->spi) {
		LOG_DBG("spi device not found: %s", config->spi_name);
		return -EINVAL;
	}

	data->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB ;//| SPI_MODE_CPHA | SPI_MODE_CPOL
	data->spi_cfg.frequency = config->spi_max_frequency;
	data->spi_cfg.slave = config->spi_slave;

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->mcp3912_cs_ctrl.gpio_dev =
				device_get_binding(config->gpio_cs_port);
	if (!data->mcp3912_cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	data->mcp3912_cs_ctrl.gpio_pin = config->cs_gpio;
	data->mcp3912_cs_ctrl.gpio_dt_flags = config->cs_flags;

	data->spi_cfg.cs = &data->mcp3912_cs_ctrl;
#endif

	data->rst_gpio = device_get_binding(config->gpio_rst_port);
	if (data->rst_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			config->gpio_rst_port);
		return -EINVAL;
	}

	if (mcp3912_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}

	gpio_pin_configure(data->rst_gpio, config->rst_gpio, GPIO_OUTPUT);

	mcp3912_reset(dev);

	if (mcp3912_chip_init(dev) < 0) {
		return -ENODEV;
	}

	return 0;
}


// -------------------------------------- TRIGGER --------------------------------------------

static void mcp3912_thread_cb(const struct device *dev)
{
	struct mcp3912_data *drv_data = dev->data;

	k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
	if (drv_data->drdy_handler != NULL) {
		drv_data->drdy_handler(dev, &drv_data->drdy_trigger);
	}
	k_mutex_unlock(&drv_data->trigger_mutex);
}

static void mcp3912_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct mcp3912_data *drv_data =
		CONTAINER_OF(cb, struct mcp3912_data, drdy_gpio_cb);

#if defined(CONFIG_MCP3912_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_MCP3912_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_MCP3912_TRIGGER_OWN_THREAD)
static void mcp3912_thread(struct mcp3912_data *drv_data)
{
	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		mcp3912_thread_cb(drv_data->dev);
	}
}
#elif defined(CONFIG_MCP3912_TRIGGER_GLOBAL_THREAD)
static void mcp3912_work_cb(struct k_work *work)
{
	struct mcp3912_data *drv_data =
		CONTAINER_OF(work, struct mcp3912_data, work);

	mcp3912_thread_cb(drv_data->dev);
}
#endif

int mcp3912_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct mcp3912_data *drv_data = dev->data;
	uint8_t int_mask, int_en, status_buf;

	switch (trig->type) {
		case SENSOR_TRIG_DATA_READY:
			k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
			drv_data->drdy_handler = handler;
			drv_data->drdy_trigger = *trig;
			k_mutex_unlock(&drv_data->trigger_mutex);
			break;
		default:
			LOG_ERR("Unsupported sensor trigger");
			return -ENOTSUP;
	}
        return 0;
}

int mcp3912_init_interrupt(const struct device *dev)
{
	const struct mcp3912_config *cfg = dev->config;
        struct mcp3912_data *drv_data = dev->data;

	k_mutex_init(&drv_data->trigger_mutex);

	drv_data->drdy_gpio = device_get_binding(cfg->gpio_drdy_port);
	if (drv_data->drdy_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			cfg->gpio_drdy_port);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->drdy_gpio, cfg->drdy_gpio,
			   GPIO_INPUT | cfg->drdy_flags);

	gpio_init_callback(&drv_data->drdy_gpio_cb,
			   mcp3912_gpio_callback,
			   BIT(cfg->drdy_gpio));

	if (gpio_add_callback(drv_data->drdy_gpio, &drv_data->drdy_gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback!");
		return -EIO;
	}

	drv_data->dev = dev;

#if defined(CONFIG_MCP3912_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_MCP3912_THREAD_STACK_SIZE,
			(k_thread_entry_t)mcp3912_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_MCP3912_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_MCP3912_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = mcp3912_work_cb;
#endif

	gpio_pin_interrupt_configure(drv_data->drdy_gpio, cfg->drdy_gpio,
				     GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

// ----------------------------------------------------------------------------------------------------

static const struct mcp3912_config mcp3912_config = {
	.spi_name = DT_INST_BUS_LABEL(0),
	.spi_slave = DT_INST_REG_ADDR(0),
	.spi_max_frequency = DT_INST_PROP(0, spi_max_frequency),
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
	.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
	.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
#endif
	.gpio_drdy_port = DT_INST_GPIO_LABEL(0, drdy_gpios),
	.drdy_gpio = DT_INST_GPIO_PIN(0, drdy_gpios),
	.drdy_flags = DT_INST_GPIO_FLAGS(0, drdy_gpios),

	.gpio_rst_port = DT_INST_GPIO_LABEL(0, rst_gpios),
	.rst_gpio = DT_INST_GPIO_PIN(0, rst_gpios),
	.rst_flags = DT_INST_GPIO_FLAGS(0, rst_gpios),
};

static const struct sensor_driver_api mcp3912_api_funcs = {
	.attr_set     = mcp3912_attr_set,
	.sample_fetch = mcp3912_sample_fetch,
	.channel_get  = mcp3912_channel_get,
	.trigger_set  = mcp3912_trigger_set,
};

DEVICE_DT_INST_DEFINE(0, mcp3912_init, NULL,
		    &mcp3912_data, &mcp3912_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &mcp3912_api_funcs);
