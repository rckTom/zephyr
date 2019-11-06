#include <logging/log.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include "bmi088_acc.h"
#include <math.h>

LOG_MODULE_REGISTER(bmi088_accel, CONFIG_SENSOR_LOG_LEVEL);

static struct bmi088_acc_data bmi088_acc_data;

static struct bmi088_acc_config bmi088_acc_config = {
	.bmi088_com_dev_name = DT_INST_0_BOSCH_BMI088_ACCEL_BUS_NAME
};

static void conv_single_sample(float lsb, s16_t raw,
			       struct sensor_value *val)
{
	float fval = (float) raw * lsb;
	float it;
	float fr = modff(fval, &it);

	val->val1 = (s32_t)it;
	val->val2 = (s32_t)(fr * 1000000);
}

static void bmi088_temp_sample_convert(struct bmi088_acc_data *data,
				       enum sensor_channel channel)
{
	s16_t temp;
	float lsb = 0.125;

	data->temp_raw = sys_le16_to_cpu(data->temp_raw);
	data->temp_raw = (0xFF00 & data->temp_raw) * 8 +
			 (data->temp_raw & 0xFF) / 32;

	if (data->temp_raw > 1023) {
		temp = data->temp_raw - 2048;
	} else {
		temp = data->temp_raw;
	}

	conv_single_sample(lsb, temp, &data->temp);
	data->temp.val1 += 23;
}

static void bmi088_acc_samples_convert(struct bmi088_acc_data *data,
				       enum sensor_channel channel)
{
	float range;

	switch (data->acc_range) {
	case BMI088_ACC_RANGE_24:
		range = 24.0;
		break;
	case BMI088_ACC_RANGE_12:
		range = 12.0;
		break;
	case BMI088_ACC_RANGE_6:
		range = 6.0;
		break;
	case BMI088_ACC_RANGE_3:
		range = 3.0;
		break;
	default:
		LOG_ERR("no valid range setting");
		return;
	}

	float lsb = (float)range / (float)((1 << 15) - 1);

	switch (channel) {
	case SENSOR_CHAN_ACCEL_X:
		data->acc_raw_values.x_value =
			sys_le16_to_cpu(data->acc_raw_values.x_value);
		conv_single_sample(lsb,
				   data->acc_raw_values.x_value,
				   &data->acc_values.x_value);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		data->acc_raw_values.y_value =
			sys_le16_to_cpu(data->acc_raw_values.y_value);
		conv_single_sample(lsb,
				   data->acc_raw_values.y_value,
				   &data->acc_values.y_value);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		data->acc_raw_values.z_value =
			sys_le16_to_cpu(data->acc_raw_values.z_value);
		conv_single_sample(lsb,
				   data->acc_raw_values.z_value,
				   &data->acc_values.z_value);
		break;
	default:
		data->acc_raw_values.x_value =
			sys_le16_to_cpu(data->acc_raw_values.x_value);
		data->acc_raw_values.y_value =
			sys_le16_to_cpu(data->acc_raw_values.y_value);
		data->acc_raw_values.z_value =
			sys_le16_to_cpu(data->acc_raw_values.z_value);
		conv_single_sample(lsb,
				   data->acc_raw_values.x_value,
				   &data->acc_values.x_value);
		conv_single_sample(lsb,
				   data->acc_raw_values.y_value,
				   &data->acc_values.y_value);
		conv_single_sample(lsb,
				   data->acc_raw_values.z_value,
				   &data->acc_values.z_value);
	}
}

static void bmi088_samples_convert(struct bmi088_acc_data *data,
				   enum sensor_channel channel)
{
	bool conv_tmp, conv_acc = false;

	if ((channel == SENSOR_CHAN_ACCEL_X) |
	    (channel == SENSOR_CHAN_ACCEL_Y) |
	    (channel == SENSOR_CHAN_ACCEL_Z) |
	    (channel == SENSOR_CHAN_ACCEL_XYZ)) {
		conv_acc = true;
	} else if (channel == SENSOR_CHAN_DIE_TEMP) {
		conv_tmp = true;
	} else if (channel == SENSOR_CHAN_ALL) {
		conv_tmp = true;
		conv_acc = true;
	} else {
		return;
	}

	if (conv_acc) {
		bmi088_acc_samples_convert(data, channel);
	}

	if (conv_tmp) {
		bmi088_temp_sample_convert(data, channel);
	}

}

static int bmi088_acc_sample_fetch(struct device *dev,
				   enum sensor_channel channel)
{
	struct bmi088_acc_data *data = dev->driver_data;
	void *data_start;
	u8_t start_reg;
	int count;
	int res;

	switch (channel) {
	case SENSOR_CHAN_ACCEL_X:
		start_reg = BMI088_REG_ACC_X_LSB;
		data_start = &(data->acc_raw_values.x_value);
		count = 2;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		start_reg = BMI088_REG_ACC_Y_LSB;
		data_start = &(data->acc_raw_values.y_value);
		count = 2;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		start_reg = BMI088_REG_ACC_Z_LSB;
		data_start = &(data->acc_raw_values.z_value);
		count = 2;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		start_reg = BMI088_REG_TEMP_LSB;
		data_start = &(data->temp_raw);
		count = 2;
		break;
	case SENSOR_CHAN_ALL:
	/* Fall through is intentional */
	case SENSOR_CHAN_ACCEL_XYZ:
		start_reg = BMI088_REG_ACC_X_LSB;
		data_start = data->acc_raw_values.array;
		count = 6;
		break;
	default:
		return -EINVAL;
	}

	res = data->tf->read_register(data, start_reg, count, data_start);
	if (res !=  0) {
		LOG_ERR("unable to fetch sensor values");
		return res;
	}

	bmi088_samples_convert(data, channel);
	return 0;
}

static int bmi088_acc_enable(struct bmi088_acc_data *data)
{
	u8_t reg_val = BMI088_ACC_PWR_CONF_ACTIVE;
	int res = data->tf->write_register(data, BMI088_REG_ACC_PWR_CONF, 1,
					   &reg_val);

	if (res != 0) {
		return res;
	}

	k_sleep(1);

	reg_val = BMI088_ACC_PWR_CTRL_ACC_ON;
	res = data->tf->write_register(data, BMI088_REG_ACC_PWR_CTRL, 1,
				       &reg_val);
	if (res != 0) {
		return res;
	}

	k_sleep(5);
	return 0;
}

/*
 *  Performs a self test described in the datasheet
 *  (https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI088-DS001.pdf)
 *  section 4.6.1
 */

static int bmi088_self_test(struct device *dev)
{
	struct bmi088_acc_data *data = dev->driver_data;
	const struct bmi088_transfer_function *tf = data->tf;
	struct sensor_value val_p[3];
	struct sensor_value val_n[3];
	u8_t reg_val = 0;
	int res;

	reg_val = BMI088_ACC_RANGE_24;
	res = tf->write_register(data, BMI088_REG_ACC_RANGE, 1, &reg_val);
	if (res != 0) {
		return res;
	}
	data->acc_odr =  BMI088_ACC_CONF_ODR_1600;
	data->acc_bandwidth = BMI088_ACC_CONF_BANDWIDTH_OSR4;
	data->acc_range = BMI088_ACC_RANGE_24;

	reg_val = 0xA7;
	res = tf->write_register(data, BMI088_REG_ACC_CONF, 1, &reg_val);
	if (res != 0) {
		return res;
	}
	k_sleep(3);

	reg_val = BMI088_ACC_SELF_TEST_POS;
	res = tf->write_register(data, BMI088_REG_ACC_SELF_TEST, 1, &reg_val);
	if (res != 0) {
		return res;
	}
	k_sleep(51);

	bmi088_acc_sample_fetch(dev, SENSOR_CHAN_ACCEL_XYZ);
	memcpy(val_p, data->acc_values.array, sizeof(val_p));

	reg_val = BMI088_ACC_SELF_TEST_NEG;
	res = tf->write_register(data, BMI088_REG_ACC_SELF_TEST, 1, &reg_val);
	if (res != 0) {
		return res;
	}
	k_sleep(51);

	bmi088_acc_sample_fetch(dev, SENSOR_CHAN_ACCEL_XYZ);
	memcpy(val_n, data->acc_values.array, sizeof(val_n));

	reg_val = BMI088_ACC_SELF_TEST_OFF;
	res = tf->write_register(data, BMI088_REG_ACC_SELF_TEST, 1, &reg_val);
	if (res != 0) {
		return res;
	}

	reg_val = BMI088_ACC_SOFTRESET_RESET;
	res = tf->write_register(data, BMI088_REG_ACC_SOFTRESET, 1, &reg_val);
	if (res != 0) {
		return res;
	}

	if (reg_val != 0x1E) {
		return -EINVAL;
	}

	k_sleep(1);

	res = bmi088_acc_enable(data);
	return res;
}


static int bmi088_acc_attr_set(struct device *dev,
			       enum sensor_channel chan,
			       enum sensor_attribute attr,
			       const struct sensor_value *val)
{
	struct bmi088_acc_data *data = dev->driver_data;
	u8_t reg;
	u8_t reg_val = 0;

	if (chan != SENSOR_CHAN_ALL) {
		return -EINVAL;
	}

	if (attr == SENSOR_ATTR_FULL_SCALE) {
		reg = BMI088_REG_ACC_RANGE;
		switch (val->val1) {
		case 3:
			data->acc_range = BMI088_ACC_RANGE_3;
			break;
		case 6:
			data->acc_range = BMI088_ACC_RANGE_6;
			break;
		case 12:
			data->acc_range = BMI088_ACC_RANGE_12;
			break;
		case 24:
			data->acc_range = BMI088_ACC_RANGE_24;
			break;
		default:
			return -EINVAL;
		}
		reg_val = data->acc_range;
	} else if (attr == SENSOR_ATTR_OVERSAMPLING) {
		reg = BMI088_REG_ACC_CONF;
		switch (val->val1) {
		case 1:
			data->acc_bandwidth = BMI088_ACC_CONF_BANDWIDTH_OSR1;
			break;
		case 2:
			data->acc_bandwidth = BMI088_ACC_CONF_BANDWIDTH_OSR1;
			break;
		case 4:
			data->acc_bandwidth = BMI088_ACC_CONF_BANDWIDTH_OSR1;
			break;
		default:
			return -EINVAL;
		}
		reg_val = data->acc_bandwidth | data->acc_odr;
	} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
		reg = BMI088_REG_ACC_CONF;
		switch (val->val1) {
		case 12:
			data->acc_odr = BMI088_ACC_CONF_ODR_12_5;
			break;
		case 25:
			data->acc_odr = BMI088_ACC_CONF_ODR_25;
			break;
		case 50:
			data->acc_odr =  BMI088_ACC_CONF_ODR_50;
			break;
		case 100:
			data->acc_odr = BMI088_ACC_CONF_ODR_100;
			break;
		case 200:
			data->acc_odr = BMI088_ACC_CONF_ODR_200;
			break;
		case 400:
			data->acc_odr = BMI088_ACC_CONF_ODR_400;
			break;
		case 800:
			data->acc_odr = BMI088_ACC_CONF_ODR_800;
			break;
		case 1600:
			data->acc_odr = BMI088_ACC_CONF_ODR_1600;
			break;
		default:
			return -EINVAL;
		}
		reg_val = data->acc_odr | data->acc_bandwidth;
	} else {
		return -EINVAL;
	}

	return data->tf->write_register(data, reg, 1, &reg_val);
}

static int bmi088_acc_channel_get(struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct bmi088_acc_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		*val = data->acc_values.x_value;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		*val = data->acc_values.x_value;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		*val = data->acc_values.x_value;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		*val = data->temp;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bmi088_acc_init(struct device *dev)
{
	struct bmi088_acc_data *data = dev->driver_data;
	const struct bmi088_acc_config *config = dev->config->config_info;
	int res;
	u8_t reg;

	data->bmi088_com_dev = device_get_binding(config->bmi088_com_dev_name);
	if (!data->bmi088_com_dev) {
		LOG_ERR("Communicaton master device not found");
		return -EINVAL;
	}

	memset(data->acc_values.array, 0, ARRAY_SIZE(data->acc_values.array));

	data->acc_odr =         BMI088_ACC_DEFAULT_ODR;
	data->acc_range =       BMI088_ACC_DEFAULT_RANGE;
	data->acc_bandwidth =   BMI088_ACC_DEFAULT_BANDWIDTH;

#if defined(DT_BOSCH_BMI088_ACCEL_BUS_SPI)
	bmi088_acc_spi_init(data);
#else
	bmi088_i2c_init(data);
#endif

	res = bmi088_acc_enable(data);
	if (res != 0) {
		return res;
	}

	res = data->tf->read_register(data, BMI088_REG_ACC_CHIP_ID, 1, &reg);
	if (res != 0) {
		return res;
	}

	res = bmi088_self_test(dev);
	return res;
}

static const struct sensor_driver_api bmi088_acc_api_funcs = {
	.attr_set = bmi088_acc_attr_set,
	.sample_fetch = bmi088_acc_sample_fetch,
	.channel_get = bmi088_acc_channel_get,
};

DEVICE_AND_API_INIT(bmi088_accel,
		    DT_INST_0_BOSCH_BMI088_ACCEL_LABEL,
		    bmi088_acc_init,
		    &bmi088_acc_data,
		    &bmi088_acc_config,
		    POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY,
		    &bmi088_acc_api_funcs);

