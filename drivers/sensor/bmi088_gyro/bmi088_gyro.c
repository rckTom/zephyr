#include <logging/log.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include "bmi088_gyro.h"
#include <math.h>
#include <stdio.h>

LOG_MODULE_REGISTER(bmi088_gyro, CONFIG_SENSOR_LOG_LEVEL);

static const struct bmi088_gyr_config bmi088_gyr_config = {
	.bmi088_com_dev_name = DT_INST_0_BOSCH_BMI088_GYRO_BUS_NAME,
};

static struct bmi088_gyr_data bmi088_gyr_data;

static void conv_single_sample(float lsb, s16_t raw,
			       struct sensor_value *val)
{
	float fval = (float) raw * lsb;
	float it;
	float fr = modff(fval, &it);

	val->val1 = (s32_t)it;
	val->val2 = (s32_t)(fr * 1000000);
}

static void bmi088_samples_convert(struct bmi088_gyr_data *data,
				   enum sensor_channel channel)
{
	float range;

	switch (data->gyro_range) {
	case BMI088_GYRO_RANGE_2000:
		range = 2000.0;
		break;
	case BMI088_GYRO_RANGE_1000:
		range = 1000.0;
		break;
	case BMI088_GYRO_RANGE_500:
		range = 500.0;
		break;
	case BMI088_GYRO_RANGE_250:
		range = 250.0;
		break;
	case BMI088_GYRO_RANGE_125:
		range = 125.0;
		break;
	default:
		LOG_ERR("no valid range setting");
		return;
	}

	float lsb = (float)range / (float)((1 << 15) - 1);

	switch (channel) {
	case SENSOR_CHAN_GYRO_X:
		conv_single_sample(lsb,
				   data->gyro_raw_values.x_value,
				   &data->gyro_values.x_value);
		break;
	case SENSOR_CHAN_GYRO_Y:
		conv_single_sample(lsb,
				   data->gyro_raw_values.y_value,
				   &data->gyro_values.y_value);
		break;
	case SENSOR_CHAN_GYRO_Z:
		conv_single_sample(lsb,
				   data->gyro_raw_values.z_value,
				   &data->gyro_values.z_value);
		break;
	default:
		conv_single_sample(lsb,
				   data->gyro_raw_values.x_value,
				   &data->gyro_values.x_value);
		conv_single_sample(lsb,
				   data->gyro_raw_values.y_value,
				   &data->gyro_values.y_value);
		conv_single_sample(lsb,
				   data->gyro_raw_values.z_value,
				   &data->gyro_values.z_value);
	}
}

static int bmi088_gyr_sample_fetch(struct device *dev,
				   enum sensor_channel channel)
{
	struct bmi088_gyr_data *data = dev->driver_data;
	void *data_start;
	u8_t start_reg;
	int count;
	int res;

	switch (channel) {
	case SENSOR_CHAN_GYRO_X:
		start_reg = BMI088_REG_RATE_X_LSB;
		data_start = &(data->gyro_raw_values.x_value);
		count = 2;
		break;
	case SENSOR_CHAN_GYRO_Y:
		start_reg = BMI088_REG_RATE_Y_LSB;
		data_start = &(data->gyro_raw_values.y_value);
		count = 2;
		break;
	case SENSOR_CHAN_GYRO_Z:
		start_reg = BMI088_REG_RATE_Z_LSB;
		data_start = &(data->gyro_raw_values.z_value);
		count = 2;
		break;
	case SENSOR_CHAN_ALL:
	/* Fall through is intentional */
	case SENSOR_CHAN_GYRO_XYZ:
		start_reg = BMI088_REG_RATE_X_LSB;
		data_start = data->gyro_raw_values.array;
		count = 6;
		break;
	default:
		return -EINVAL;
	}

	res = data->tf->read_register(data, start_reg, count, data_start);


	if (res !=  0) {
		LOG_ERR("unable to fetch sensor values");
		return -EINVAL;
	}

	bmi088_samples_convert(data, channel);
	return 0;
}

static int bmi088_gyr_attr_set(struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr,
			       const struct sensor_value *val)
{
	struct bmi088_gyr_data *data = dev->driver_data;
	u8_t reg;
	u8_t reg_val;

	if (chan !=  SENSOR_CHAN_ALL) {
		return -EINVAL;
	}

	if (attr == SENSOR_ATTR_FULL_SCALE) {
		reg = BMI088_REG_GYRO_RANGE;
		switch (val->val1) {
		case 2000:
			reg_val = BMI088_GYRO_RANGE_2000;
			break;
		case 1000:
			reg_val = BMI088_GYRO_RANGE_1000;
			break;
		case 500:
			reg_val = BMI088_GYRO_RANGE_500;
			break;
		case 250:
			reg_val = BMI088_GYRO_RANGE_250;
			break;
		case 125:
			reg_val = BMI088_GYRO_RANGE_125;
			break;
		default:
			return -EINVAL;
		}
	} else if (attr == (enum sensor_attribute) BMI088_GYRO_ATTR_BW) {
		switch (val->val1) {
		case 12:
			reg_val = BMI088_GYRO_BANDWIDTH_12;
			break;
		case 23:
			reg_val = BMI088_GYRO_BANDWIDTH_23;
			break;
		case 32:
			reg_val = BMI088_GYRO_BANDWIDTH_32;
			break;
		case 47:
			reg_val = BMI088_GYRO_BANDWIDTH_47;
			break;
		case 64:
			reg_val = BMI088_GYRO_BANDWIDTH_64;
			break;
		case 116:
			reg_val = BMI088_GYRO_BANDWIDTH_116;
			break;
		case 230:
			reg_val = BMI088_GYRO_BANDWIDTH_230;
			break;
		case 532:
			reg_val = BMI088_GYRO_BANDWIDTH_532;
			break;
		default:
			return -EINVAL;
		}
	}

	int res = data->tf->write_register(data, reg, 1, &reg_val);

	if (res != 0) {
		LOG_ERR("unable to set attribute");
		return res;
	}

	return 0;
}

static int bmi088_gyr_channel_get(struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct bmi088_gyr_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		*val = data->gyro_values.x_value;
		return 0;
	case SENSOR_CHAN_GYRO_Y:
		*val = data->gyro_values.y_value;
		return 0;
	case SENSOR_CHAN_GYRO_Z:
		*val = data->gyro_values.z_value;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bmi088_gyr_init(struct device *dev)
{
	struct bmi088_gyr_data *data = dev->driver_data;
	const struct bmi088_gyr_config *config = dev->config->config_info;

	data->bmi088_com_dev = device_get_binding(config->bmi088_com_dev_name);
	if (!data->bmi088_com_dev) {
		LOG_ERR("Communicaton  master device not found");
		return -EINVAL;
	}

	memset(data->gyro_values.array, 0, ARRAY_SIZE(data->gyro_values.array));

	data->gyro_odr =        BMI088_GYRO_DEFAULT_BANDWIDTH;
	data->gyro_range =      BMI088_GYRO_DEFAULT_RANGE;

#ifdef DT_BOSCH_BMI088_GYRO_BUS_SPI
	bmi088_gyr_spi_init(data);
#else
	bmi088_gyr_i2c_init(data);
#endif

	int res;
	u8_t chip_id;

	/* Get chip id */
	res = data->tf->read_register(data, BMI088_REG_GYRO_CHIP_ID, 1,
				      &chip_id);
	if (res != 0) {
		LOG_ERR("error getting chip id");
		return -EINVAL;
	} else if (chip_id  != 0x0F) {
		LOG_ERR("received wrong chip id");
		return -EINVAL;
	}
	LOG_INF("Chip id #%X", chip_id);

	/* Perform self test */
	u8_t gyro_self_test_val = BMI088_MASK_GYRO_SELF_TEST_TRIG_BIST;

	res =  data->tf->write_register(data, BMI088_REG_GYRO_SELF_TEST,
					1, &gyro_self_test_val);
	if (res != 0) {
		LOG_ERR("unable to start self test");
		return -EINVAL;
	}

	while (!(gyro_self_test_val & BMI088_MASK_GYRO_SELF_TEST_BIST_RDY)) {
		k_sleep(10);
		data->tf->read_register(data, BMI088_REG_GYRO_SELF_TEST,
					1, &gyro_self_test_val);
	}
	data->tf->read_register(data, BMI088_REG_GYRO_SELF_TEST,
				1, &gyro_self_test_val);
	LOG_INF("self test result %d",
		gyro_self_test_val & BMI088_MASK_GYRO_SELF_TEST_BIST_FAIL);

	/* Set default configuration */
	u8_t reg_val = BMI088_GYRO_DEFAULT_BANDWIDTH;

	res = data->tf->write_register(data, BMI088_REG_GYRO_BANDWIDTH, 1,
				       &reg_val);
	if (res != 0) {
		LOG_ERR("unable to set default bandwidth");
		return res;
	}

	reg_val = BMI088_GYRO_DEFAULT_RANGE;
	res = data->tf->write_register(data, BMI088_REG_GYRO_RANGE, 1,
				       &reg_val);
	if (res != 0) {
		LOG_ERR("unable to set default range");
		return res;
	}

	return 0;
}

static const struct sensor_driver_api bmi088_gyr_api_funcs = {
	.attr_set = bmi088_gyr_attr_set,
	.sample_fetch = bmi088_gyr_sample_fetch,
	.channel_get = bmi088_gyr_channel_get,
};

DEVICE_AND_API_INIT(bmi088_gyro,
		    DT_INST_0_BOSCH_BMI088_GYRO_LABEL,
		    bmi088_gyr_init,
		    &bmi088_gyr_data,
		    &bmi088_gyr_config,
		    POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY,
		    &bmi088_gyr_api_funcs);
