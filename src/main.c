#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <sensor.h>

#include <logging/log.h>
#define LOG_MODULE_NAME main_module
#define LOG_LEVEL CONFIG_MAIN_MODULE_LOG_LEVEL
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

struct k_delayed_work led_timer;

enum periph_device {
	DEV_IDX_LED0 = 0,
	DEV_IDX_LED1,
	DEV_IDX_BUTTON,
	DEV_IDX_APDS9960,
	DEV_IDX_NUMOF,
};

struct periph_device_info {
	struct device *dev;
	char *name;
};

static struct periph_device_info dev_info[] = {
	{NULL, LED0_GPIO_CONTROLLER},
	{NULL, LED1_GPIO_CONTROLLER},
	{NULL, SW0_GPIO_CONTROLLER},
	{NULL, CONFIG_APDS9960_DRV_NAME},
};

static void button_pressed(struct device *gpiob, struct gpio_callback *cb,
			   u32_t pins)
{
	LOG_DBG("Button pressed at %d", k_cycle_get_32());
}

static struct gpio_callback gpio_cb;

static int get_apds9960_val(struct sensor_value *val)
{
	if (sensor_sample_fetch(dev_info[DEV_IDX_APDS9960].dev)) {
		LOG_ERR("Failed to fetch sample for device %s",
		       dev_info[DEV_IDX_APDS9960].name);
		return -1;
	}
 	if (sensor_channel_get(dev_info[DEV_IDX_APDS9960].dev,
			       SENSOR_CHAN_LIGHT,
			       &val[0])) {
		return -1;
	}
 	if (sensor_channel_get(dev_info[DEV_IDX_APDS9960].dev,
			       SENSOR_CHAN_PROX,
			       &val[1])) {
		return -1;
	}
 	LOG_DBG("ambient light intensity: %d, proximity %d",
	       val[0].val1, val[1].val1);
 	return 0;
}

static void led_timeout(struct k_work *work)
{
        static int led_cntr;
        u32_t led_interval = K_MSEC(100);

        gpio_pin_write(dev_info[DEV_IDX_LED0].dev, LED0_GPIO_PIN, 1);
	gpio_pin_write(dev_info[DEV_IDX_LED1].dev, LED1_GPIO_PIN, 1);

        if (led_cntr == 0) {
		gpio_pin_write(dev_info[DEV_IDX_LED0].dev,
			       LED0_GPIO_PIN, 0);
		led_cntr += 1;
	} else if (led_cntr == 1) {
		gpio_pin_write(dev_info[DEV_IDX_LED1].dev,
			       LED1_GPIO_PIN, 0);
		led_cntr += 1;
	} else if (led_cntr == 2) {
		led_cntr = 0;
		led_interval = K_MSEC(5000);
	}

        k_delayed_work_submit(&led_timer, led_interval);
}

void main(void)
{
        struct sensor_value val[3];
        unsigned int i;
    
        LOG_INF("*** IMU Controller ***");

        for (i = 0; i < ARRAY_SIZE(dev_info); i++) {
                dev_info[i].dev = device_get_binding(dev_info[i].name);
                if (dev_info[i].dev == NULL) {
                        LOG_ERR("Failed to get %s device", dev_info[i].name);
                        return;
                }
        }

	gpio_pin_configure(dev_info[DEV_IDX_LED0].dev, LED0_GPIO_PIN,
			   GPIO_DIR_OUT);
	gpio_pin_write(dev_info[DEV_IDX_LED0].dev, LED0_GPIO_PIN, 1);

 	gpio_pin_configure(dev_info[DEV_IDX_LED1].dev, LED1_GPIO_PIN,
			   GPIO_DIR_OUT);
	gpio_pin_write(dev_info[DEV_IDX_LED1].dev, LED1_GPIO_PIN, 1);

 	gpio_pin_configure(dev_info[DEV_IDX_BUTTON].dev, SW0_GPIO_PIN,
			   GPIO_DIR_IN | GPIO_INT |  SW0_GPIO_FLAGS |
			   GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);

 	gpio_init_callback(&gpio_cb, button_pressed, BIT(SW0_GPIO_PIN));
	gpio_add_callback(dev_info[DEV_IDX_BUTTON].dev, &gpio_cb);
	gpio_pin_enable_callback(dev_info[DEV_IDX_BUTTON].dev, SW0_GPIO_PIN);

        k_delayed_work_init(&led_timer, led_timeout);
	k_delayed_work_submit(&led_timer, K_MSEC(100));

        while (1) {
		if (get_apds9960_val(val)) {
			goto _error_get;
		}

		k_sleep(K_SECONDS(1));
	}

_error_get:
	LOG_ERR("Failed to get sensor data or print a string");
}