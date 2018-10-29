#include <zephyr.h>
#include <board.h>
#include <sensor.h>

#include <logging/log.h>
#define LOG_MODULE_NAME imu_module
#define LOG_LEVEL CONFIG_IMU_MODULE_LOG_LEVEL
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define IMU_STACK_SIZE 1024
#define IMU_PRIORITY 7

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(struct device *dev,
				    struct sensor_trigger *trig)
{
        struct sensor_value accel_x, accel_y, accel_z;
	struct sensor_value gyro_x, gyro_y, gyro_z;

        sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

        LOG_DBG("TRIG");
}
#endif

void imu_thread(void)
{
        static struct device *imu;

        LOG_INF("IMU Task Started");

	imu = device_get_binding(CONFIG_LSM6DSL_DEV_NAME);
	if (!imu) {
		LOG_ERR("Could not get pointer to %s sensor", CONFIG_LSM6DSL_DEV_NAME);
		return;
	};

#if defined(CONFIG_LSM6DSL_ACCEL_ODR) && (CONFIG_LSM6DSL_ACCEL_ODR == 0)
	struct sensor_value a_odr_attr;

	/* set sampling frequency to 104Hz for accel */
	a_odr_attr.val1 = 104;
	a_odr_attr.val2 = 0;

	if (sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &a_odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for accelerometer.");
		return;
	}
#endif

#if defined(CONFIG_LSM6DSL_ACCEL_FS) && (CONFIG_LSM6DSL_ACCEL_FS == 0)
	struct sensor_value a_fs_attr;

	/* set full scale to 16g for accel */
	sensor_g_to_ms2(16, &a_fs_attr);

	if (sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &a_fs_attr) < 0) {
		LOG_ERR("Cannot set fs for accelerometer.");
		return;
	}
#endif

#if defined(CONFIG_LSM6DSL_GYRO_ODR) && (CONFIG_LSM6DSL_GYRO_ODR == 0)
	struct sensor_value g_odr_attr;

	/* set sampling frequency to 104Hz for accel */
	g_odr_attr.val1 = 104;
	g_odr_attr.val2 = 0;

	if (sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &g_odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for gyro.");
		return;
	}
#endif

#if defined(CONFIG_LSM6DSL_GYRO_FS) && (CONFIG_LSM6DSL_GYRO_FS == 0)
	struct sensor_value g_fs_attr;

	/* set full scale to 245dps for accel */
	sensor_g_to_ms2(245, &g_fs_attr);

	if (sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &g_fs_attr) < 0) {
		LOG_ERR("Cannot set fs for gyroscope.");
		return;
	}
#endif

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(imu, &trig, lsm6dsl_trigger_handler);
#endif

    while (1) {
        struct sensor_value accel_x, accel_y, accel_z;
        struct sensor_value gyro_x, gyro_y, gyro_z;

        /* lsm6dsl accel */
        sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &accel_z);
        

        /* lsm6dsl gyro */
        sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gyro_x);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gyro_y);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gyro_z);
        LOG_INF("accel (%f %f %f) m/s2 gyro (%f %f %f) dps", 
                out_ev(&accel_x), out_ev(&accel_y), out_ev(&accel_z),
                out_ev(&gyro_x), out_ev(&gyro_y), out_ev(&gyro_z));

        k_sleep(2000);
    }
}

K_THREAD_DEFINE(imu_id, IMU_STACK_SIZE, imu_thread, NULL, NULL, NULL,
		IMU_PRIORITY, 0, K_NO_WAIT);