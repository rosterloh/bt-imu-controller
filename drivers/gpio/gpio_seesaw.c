#include <kernel.h>
#include <device.h>
#include <gpio.h>
#include <i2c.h>

#define LOG_LEVEL LOG_LEVEL_DEBUG
#include <logging/log.h>
LOG_MODULE_REGISTER(seesaw);

/** Module Base Addreses
 *  The module base addresses for different seesaw modules.
 */
enum {
	SEESAW_STATUS_BASE = 0x00,
	SEESAW_GPIO_BASE = 0x01,
	SEESAW_SERCOM0_BASE = 0x02,

	SEESAW_TIMER_BASE = 0x08,
	SEESAW_ADC_BASE = 0x09,
	SEESAW_DAC_BASE = 0x0A,
	SEESAW_INTERRUPT_BASE = 0x0B,
	SEESAW_DAP_BASE = 0x0C,
	SEESAW_EEPROM_BASE = 0x0D,
	SEESAW_NEOPIXEL_BASE = 0x0E,
	SEESAW_TOUCH_BASE = 0x0F,
	SEESAW_KEYPAD_BASE = 0x10,
	SEESAW_ENCODER_BASE = 0x11,
};

/** status module function addres registers
 */
enum {
	SEESAW_STATUS_HW_ID = 0x01,
	SEESAW_STATUS_VERSION = 0x02,
	SEESAW_STATUS_OPTIONS = 0x03,
	SEESAW_STATUS_TEMP = 0x04,
	SEESAW_STATUS_SWRST = 0x7F,
};

#define SEESAW_HW_ID_CODE			0x55

/** Cache of the output configuration and data of the pins */
struct gpio_seesaw_pin_state {
	u16_t input_disable;
	u16_t pull_up;
	u16_t pull_down;
	u16_t open_drain;
	u16_t polarity;
	u16_t dir;
	u16_t data;
};

/** Runtime driver data */
struct gpio_seesaw_drv_data {
	struct device *i2c_master;
	struct gpio_seesaw_pin_state pin_state;
	struct k_sem lock;
};

/** Configuration data */
struct gpio_seesaw_config {
	const char *i2c_master_dev_name;
	u16_t i2c_slave_addr;
};

/**
 * @brief Read a register from the device
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for reading.
 * @param module_addr Module address register from which the data is being read.
 * @param function_addr Function address register from which the data is being read.
 * @param buf Memory pool that stores the retrieved data.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int seesaw_read_reg(struct device *dev, u16_t dev_addr,
				 u8_t module_addr, u8_t function_addr, u8_t *buf)
{
	struct i2c_msg msg[3];

	msg[0].buf = &module_addr;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = &function_addr;
	msg[1].len = 1;
	msg[1].flags = I2C_MSG_WRITE;

	msg[2].buf = buf;
	msg[2].len = 1;
	msg[2].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, msg, 3, dev_addr);
}

/**
 * @brief Write to a register on the device
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for reading.
 * @param module_addr Module address register from which the data is being read.
 * @param function_addr Function address register from which the data is being read.
 * @param val Value to write.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int seesaw_write_reg(struct device *dev, u16_t dev_addr,
				 u8_t module_addr, u8_t function_addr, u8_t val)
{
	struct i2c_msg msg[3];

	msg[0].buf = &module_addr;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = &function_addr;
	msg[1].len = 1;
	msg[1].flags = I2C_MSG_WRITE;

	msg[2].buf = &val;
	msg[2].len = 1;
	msg[2].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(dev, msg, 3, dev_addr);
}

/**
 * @brief Configure pin or port
 *
 * @param dev Device struct of the SeeSaw
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_seesaw_config(struct device *dev, int access_op, u32_t pin,
			       int flags)
{
	//const struct gpio_seesaw_config *cfg = dev->config->config_info;
	struct gpio_seesaw_drv_data *drv_data = dev->driver_data;
	struct gpio_seesaw_pin_state *pins = &drv_data->pin_state;
	int ret = 0;

	if (flags & GPIO_INT) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			pins->dir |= BIT(pin);
			pins->input_disable &= ~BIT(pin);
		} else {
			pins->dir &= ~BIT(pin);
			pins->input_disable |= BIT(pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			pins->pull_up |= BIT(pin);
		} else {
			pins->pull_up &= ~BIT(pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			pins->pull_down |= BIT(pin);
		} else {
			pins->pull_down &= ~BIT(pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			pins->pull_up &= ~BIT(pin);
			pins->pull_down &= ~BIT(pin);
		}
		if (flags & GPIO_DS_DISCONNECT_HIGH) {
			pins->open_drain |= BIT(pin);
		} else {
			pins->open_drain &= ~BIT(pin);
		}
		if (flags & GPIO_POL_INV) {
			pins->polarity |= BIT(pin);
		} else {
			pins->polarity &= ~BIT(pin);
		}
		break;
	case GPIO_ACCESS_BY_PORT:
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			pins->dir = 0xffff;
			pins->input_disable = 0x0000;
		} else {
			pins->dir = 0x0000;
			pins->input_disable = 0xffff;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			pins->pull_up = 0xffff;
		} else {
			pins->pull_up = 0x0000;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			pins->pull_down = 0xffff;
		} else {
			pins->pull_down = 0x0000;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			pins->pull_up = 0x0000;
			pins->pull_down = 0x0000;
		}
		if (flags & GPIO_DS_DISCONNECT_HIGH) {
			pins->open_drain = 0xffff;
		} else {
			pins->open_drain = 0x0000;
		}
		if (flags & GPIO_POL_INV) {
			pins->polarity = 0xffff;
		} else {
			pins->polarity = 0x0000;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}
/*
	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_DIR, pins->dir);
	if (ret)
		goto out;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_INPUT_DISABLE,
				    pins->input_disable);
	if (ret)
		goto out;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_PULL_UP,
				    pins->pull_up);
	if (ret)
		goto out;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_PULL_DOWN,
				    pins->pull_down);
	if (ret)
		goto out;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_OPEN_DRAIN,
				    pins->open_drain);
*/
out:
	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Set the pin or port output
 *
 * @param dev Device struct of the SeeSaw
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_seesaw_write(struct device *dev, int access_op, u32_t pin,
			      u32_t value)
{
	//const struct gpio_seesaw_config *cfg = dev->config->config_info;
	struct gpio_seesaw_drv_data *drv_data = dev->driver_data;
	u16_t *pin_data = &drv_data->pin_state.data;
	int ret = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		if (value) {
			*pin_data |= BIT(pin);
		} else {
			*pin_data &= ~BIT(pin);
		}
		break;
	case GPIO_ACCESS_BY_PORT:
		*pin_data = value;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}
/*
	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_DATA, *pin_data);*/
out:
	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Read the pin or port data
 *
 * @param dev Device struct of the SeeSaw
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_seesaw_read(struct device *dev, int access_op, u32_t pin,
			    u32_t *value)
{
	//const struct gpio_seesaw_config *cfg = dev->config->config_info;
	struct gpio_seesaw_drv_data *drv_data = dev->driver_data;
	//u16_t pin_data;
	int ret = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);
/*
	ret = i2c_burst_read(drv_data->i2c_master, cfg->i2c_slave_addr,
			     SX1509B_REG_DATA, (u8_t *)&pin_data,
			     sizeof(pin_data));
	if (ret)
		goto out;

	pin_data = sys_be16_to_cpu(pin_data);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		*value = !!(pin_data & (BIT(pin)));
		break;
	case GPIO_ACCESS_BY_PORT:
		*value = pin_data;
		break;
	default:
		ret = -ENOTSUP;
	}
*/
//out:
	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Initialisation function of SeeSaw
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_seesaw_init(struct device *dev)
{
	const struct gpio_seesaw_config *cfg = dev->config->config_info;
	struct gpio_seesaw_drv_data *drv_data = dev->driver_data;
	int ret;

        LOG_DBG("SeeSaw init");

	drv_data->i2c_master = device_get_binding(cfg->i2c_master_dev_name);
	if (!drv_data->i2c_master) {
                LOG_ERR("SeeSaw[0x%X]: Error getting i2c master binding", cfg->i2c_slave_addr);
		ret = -EINVAL;
		goto out;
	}

	/* Reset state */
	drv_data->pin_state = (struct gpio_seesaw_pin_state) {
		.input_disable	= 0x0000,
		.pull_up	= 0x0000,
		.pull_down	= 0x0000,
		.open_drain	= 0x0000,
		.dir		= 0xffff,
		.data		= 0xffff,
	};

	ret = seesaw_write_reg(drv_data->i2c_master, cfg->i2c_slave_addr,
			       SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xFF);

	if (ret)
		LOG_ERR("SeeSaw[0x%X]: Failed to reset device", cfg->i2c_slave_addr);
		goto out;

	u8_t id;
	ret = seesaw_read_reg(drv_data->i2c_master, cfg->i2c_slave_addr,
			      SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, &id);
						
	if (id != SEESAW_HW_ID_CODE) {
		LOG_ERR("SeeSaw[0x%X]: Incorrect device id reported (0x%X)", cfg->i2c_slave_addr, id);
	}

out:
	k_sem_give(&drv_data->lock);
	return ret;
};

static const struct gpio_seesaw_config gpio_seesaw_cfg = {
	.i2c_master_dev_name = CONFIG_GPIO_SEESAW_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr	= CONFIG_GPIO_SEESAW_I2C_ADDR,
};

static struct gpio_seesaw_drv_data gpio_seesaw_drvdata = {
	.lock = _K_SEM_INITIALIZER(gpio_seesaw_drvdata.lock, 1, 1),
};

static const struct gpio_driver_api gpio_seesaw_drv_api_funcs = {
	.config	= gpio_seesaw_config,
	.write	= gpio_seesaw_write,
	.read	= gpio_seesaw_read,
};

DEVICE_AND_API_INIT(gpio_seesaw, CONFIG_GPIO_SEESAW_DEV_NAME,
                    gpio_seesaw_init, &gpio_seesaw_drvdata, &gpio_seesaw_cfg,
                    POST_KERNEL, CONFIG_GPIO_SEESAW_INIT_PRIORITY,
                    &gpio_seesaw_drv_api_funcs);