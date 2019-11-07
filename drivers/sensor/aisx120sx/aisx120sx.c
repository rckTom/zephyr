/*
 * Copyright (c) 2019 Thomas Schmid <tom@lfence.de>

 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <math.h>
#include <drivers/sensor.h>
#include <drivers/spi.h>
#include "aisx120sx.h"
#include <sys/byteorder.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(aisx120sx, 4);

#define SELF_TEST_TOLERANCE		0.15f
#define SELF_TEST_EXPACTATION		24.0f
#define SELF_TEST_MAX_OFFSET		1

enum aisx120sx_opcode {
	OP_READ         = 0x3,
	OP_WRITE        = 0x1
};

enum aisx120sx_sensor_req_type {
	REQ_SENSOR_DATA         = 0x1,
	REQ_NON_SENSOR_DATA     = 0x0
};

enum aisx120sx_channel {
	CHANNEL_X       = 0x0,
	CHANNEL_Y       = 0x1,
};

enum aisx120sx_self_test_cmd {
	SELF_TEST_ZERO	= 0x0,
	SELF_TEST_POS	= 0x1,
	SELF_TEST_NEG	= 0x2,
};

enum aisx120sx_none_acc_status_flags {
	NONE_ACC_STAT_NORMAL            = 0x0,
	NONE_ACC_STAT_EEPROM_ERROR      = 0x1,
	NONE_ACC_STAT_SPI_ERROR         = 0x2,
	NONE_ACC_STAT_REQUEST_ERROR     = 0x3,
	NONE_ACC_STAT_TEST_MODE         = 0xF
};

enum aisx120sx_acc_status_flags {
	ACC_STAT_NO_ERROR               = 0x0,
	ACC_STAT_EEPROM_ERROR           = 0x1,
	ACC_STAT_SPI_ERROR              = 0x2,
	ACC_STAT_REQUEST_ERROR          = 0x3,
	ACC_STAT_CONDITION_NOT_CORRECT  = 0x4,
	ACC_STAT_NO_DATA_AVAILABLE      = 0x5,
	ACC_STAT_HARDWARE_ERROR         = 0x6,
	ACC_STAT_ADC_SATURATION_ERROR   = 0x7,
	ACC_STAT_TEST_MODE              = 0xF
};

union none_acc_req_data {
	struct {
		u8_t crc : 8;
		u8_t reserved_1 : 5;
		u16_t write_data : 8;
		u16_t addr : 5;
		u8_t reserved_0 : 2;
		u8_t odd_parity : 1;
		u8_t sen : 1;
		u8_t op_code : 2;
	} __packed fields;
	u32_t data_word;
	u8_t data_buf[4];
};

union none_acc_response {
	struct {
		u8_t crc : 8;
		u8_t status_flags : 4;
		u8_t reserved_1 : 1;
		u16_t read_data : 8;
		u16_t addr : 5;
		u8_t reserved_0 : 2;
		u8_t odd_parity : 1;
		u8_t op_code : 2;
		u8_t sen : 1;
	} __packed fields;
	u32_t data_word;
	u8_t data_buf[4];
};

union acc_req_data {
	struct {
		u8_t crc : 8;
		u32_t zero : 20;
		u8_t odd_parity : 1;
		u8_t sen : 1;
		u8_t channel_select : 2;
	} __packed fields;
	u32_t data_word;
	u8_t data_buf[4];
};

union acc_response {
	struct {
		u8_t crc : 8;
		enum aisx120sx_acc_status_flags status_flags : 4;
		u16_t data : 14;
		u8_t status : 2;
		u8_t odd_parity : 1;
		u8_t channel_select : 2;
		u8_t sen : 1;
	} __packed fields;
	u32_t data_word;
	u8_t data_buf[4];
};

static const struct aisx120sx_config aisx120sx_config = {
	.aisx120sx_dev_name = DT_INST_0_ST_AISX120SX_BUS_NAME,
};

static struct spi_config aisx120sx_spi_conf = {
	.frequency = DT_INST_0_ST_AISX120SX_SPI_MAX_FREQUENCY,
	.operation = (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		      SPI_TRANSFER_MSB |
		      SPI_LINES_SINGLE),
	.slave = DT_INST_0_ST_AISX120SX_BASE_ADDRESS,
	.cs = NULL,
};

#if defined(DT_INST_0_ST_AISX120SX_CS_GPIOS_CONTROLLER)
static struct spi_cs_control aisx120sx_cs_ctrl;
#endif

static u8_t crc_table[16];

static void make_crc_table(u8_t poly)
{
	for (u8_t val = 0; val <= 0xF; val++) {
		u8_t res =  0;

		res ^= val;

		for (int i = 0; i < 8; i++) {
			if (res & 0x80) {
				res = (res << 1) ^ poly;
			} else {
				res <<= 1;
			}
		}

		crc_table[val] = res;
	}
}

static u8_t crc8_caicheva_c2(const void *buf, int cnt)
{
	u8_t val = 0;
	const u8_t *ptr = buf;

	for (int i = (cnt - 1); i >= 0; i--) {
		val = crc_table[(val >> 4) ^ (ptr[i] >> 4)] ^ (val << 4);
		val = crc_table[(val >> 4) ^ (ptr[i] & 0xF)] ^ (val << 4);
	}

	return val;
}

static int has_odd_parity(void *buf, int cnt)
{
	u8_t *ptr = buf;
	int val = 0;

	for (int i = 0; i < cnt; i++) {
		u8_t reg = ptr[i];

		while (reg) {
			val ^= (reg & 0x1);
			reg = reg >> 1;
		}
	}
	return val;
}

static int check_response(u32_t data_word)
{
	u32_t payload = sys_cpu_to_le32(data_word);

	if (crc8_caicheva_c2(&payload, 4) != 0) {
		LOG_ERR("crc error detected");
		return 0;
	}

	return 0;
}

int aisx120sx_handle_none_acc_response_err(const union none_acc_response *resp)
{
	if (resp->fields.status_flags == NONE_ACC_STAT_NORMAL ||
	    resp->fields.status_flags == NONE_ACC_STAT_TEST_MODE) {
		return 0;
	}

	LOG_ERR(
	     "None acceleration command: Device responded with error status %d",
	     resp->fields.status_flags);
	switch (resp->fields.status_flags) {
	case NONE_ACC_STAT_EEPROM_ERROR:
		LOG_ERR("EEPROM error detected");
		break;
	case NONE_ACC_STAT_REQUEST_ERROR:
		LOG_ERR("Request error detected");
		break;
	case NONE_ACC_STAT_SPI_ERROR:
		LOG_ERR("Spi error detected");
		break;
	case NONE_ACC_STAT_TEST_MODE:
		return 0;
	default:
		LOG_ERR("Unknown error state");
		break;
	}

	return -EPERM;
}

int aisx120sx_handle_acc_response_err(const union acc_response *resp)
{
	if (resp->fields.status_flags == ACC_STAT_NO_ERROR ||
	    resp->fields.status_flags == ACC_STAT_TEST_MODE) {
		return 0;
	}

	LOG_ERR("Acceleration command: Device responded with errors");
	switch (resp->fields.status_flags) {
	case ACC_STAT_NO_ERROR:
		return 0;
	case ACC_STAT_REQUEST_ERROR:
		LOG_ERR("Request error detected");
		break;
	case ACC_STAT_SPI_ERROR:
		LOG_ERR("Spi error detected");
		break;
	case ACC_STAT_TEST_MODE:
		return 0;
	case ACC_STAT_ADC_SATURATION_ERROR:
		LOG_WRN("Warning: ADC saturated");
		return 0;
	case ACC_STAT_CONDITION_NOT_CORRECT:
		LOG_ERR("Error: conditions not correct");
		break;
	case ACC_STAT_NO_DATA_AVAILABLE:
		LOG_ERR("Error: no data available");
		break;
	case ACC_STAT_HARDWARE_ERROR:
		LOG_ERR("Hardware error detected");
		break;
	case ACC_STAT_EEPROM_ERROR:
		LOG_ERR("EEPROM error detected");
		break;
	default:
		break;
	}

	return -EPERM;
}

static int aisx120sx_raw_cmd_acc(const struct aisx120sx_data *data,
				 enum aisx120sx_channel channel,
				 enum aisx120sx_sensor_req_type sen,
				 union acc_response *resp)
{
	union acc_req_data req = { .data_word = 0 };

	req.fields.channel_select = channel,
	req.fields.sen = sen;
	req.fields.odd_parity = !has_odd_parity(req.data_buf + 1, 3);
	u32_t crc_payload = sys_cpu_to_le32(req.data_word >> 8);

	req.fields.crc = crc8_caicheva_c2(&crc_payload, 3);
	req.data_word = sys_cpu_to_be32(req.data_word);

	const struct spi_buf_set tx_buf = {
		.buffers = (const struct spi_buf[]) {
			{
				.buf = req.data_buf,
				.len = ARRAY_SIZE(req.data_buf),
			},
		},
		.count = 1
	};

	const struct spi_buf_set rx_buf = {
		.buffers = (const struct spi_buf[]) {
			{
				.buf = resp->data_buf,
				.len = ARRAY_SIZE(resp->data_buf),
			},
		},
		.count = 1
	};

	int res = spi_write(data->spi_dev, &aisx120sx_spi_conf, &tx_buf);

	if (res != 0) {
		LOG_ERR("unable to send acceleration command");
		return res;
	}

	res = spi_read(data->spi_dev, &aisx120sx_spi_conf, &rx_buf);

	if (res != 0) {
		LOG_ERR("unable to receive response to acceleration command");
		return res;
	}

	resp->data_word = sys_be32_to_cpu(resp->data_word);

	if (check_response(resp->data_word) != 0) {
		return -1;
	}

	return aisx120sx_handle_acc_response_err(resp);
}

static int aisx120sx_raw_cmd_single(const struct aisx120sx_data *dev,
				    enum aisx120sx_opcode op,
				    enum aisx120sx_sensor_req_type sen,
				    const u8_t addr, const u8_t data,
				    union none_acc_response *resp)
{
	union none_acc_req_data req = { .data_word = 0 };

	req.data_word = 0;
	req.fields.addr = addr;
	req.fields.write_data = data;
	req.fields.op_code = op;
	req.fields.sen = sen;
	req.fields.odd_parity = !has_odd_parity(req.data_buf + 1, 3);

	u32_t crc_payload = sys_cpu_to_le32(req.data_word >> 8);

	req.fields.crc = crc8_caicheva_c2(&crc_payload, 3);
	req.data_word = sys_cpu_to_be32(req.data_word);

	const struct spi_buf_set tx_buf = {
		.buffers = (const struct spi_buf[]) {
			{
				.buf = req.data_buf,
				.len = ARRAY_SIZE(req.data_buf),
			},
		},
		.count = 1
	};

	const struct spi_buf_set rx_buf = {
		.buffers = (const struct spi_buf[]) {
			{
				.buf = resp->data_buf,
				.len = ARRAY_SIZE(resp->data_buf),
			},
		},
		.count = 1
	};

	int res = spi_transceive(dev->spi_dev, &aisx120sx_spi_conf, &tx_buf,
				 &rx_buf);

	resp->data_word = sys_be32_to_cpu(resp->data_word);

	return res;
}

static int aisx120sx_raw_cmd_no_check(const struct aisx120sx_data *dev,
				      enum aisx120sx_opcode op,
				      enum aisx120sx_sensor_req_type sen,
				      const u8_t addr, const u8_t data,
				      union none_acc_response *resp)
{
	union none_acc_req_data req = { .data_word = 0 };

	req.data_word = 0;
	req.fields.addr = addr;
	req.fields.write_data = data;
	req.fields.op_code = op;
	req.fields.sen = sen;
	req.fields.odd_parity = !has_odd_parity(req.data_buf + 1, 3);

	u32_t crc_payload = sys_cpu_to_le32(req.data_word >> 8);

	req.fields.crc = crc8_caicheva_c2(&crc_payload, 3);

	req.data_word = sys_cpu_to_be32(req.data_word);

	const struct spi_buf_set tx_buf = {
		.buffers = (const struct spi_buf[]) {
			{
				.buf = req.data_buf,
				.len = ARRAY_SIZE(req.data_buf),
			},
		},
		.count = 1
	};

	const struct spi_buf_set rx_buf = {
		.buffers = (const struct spi_buf[]) {
			{
				.buf = resp->data_buf,
				.len = ARRAY_SIZE(resp->data_buf),
			},
		},
		.count = 1
	};


	int res = spi_write(dev->spi_dev, &aisx120sx_spi_conf, &tx_buf);

	if (res != 0) {
		LOG_ERR("unable to send none acceleration command");
		return res;
	}

	res = spi_read(dev->spi_dev, &aisx120sx_spi_conf, &rx_buf);

	if (res != 0) {
		LOG_ERR("unable to receive none acceleration command response");
		return res;
	}

	resp->data_word = sys_be32_to_cpu(resp->data_word);

	return (check_response(resp->data_word)
}

static int aisx120sx_raw_cmd(const struct aisx120sx_data *dev,
			     enum aisx120sx_opcode op,
			     enum aisx120sx_sensor_req_type sen,
			     const u8_t addr, const u8_t data,
			     union none_acc_response *resp)
{
	int res = aisx120sx_raw_cmd_no_check(dev, op, sen, addr, data, resp);

	if (res != 0) {
		return res;
	}

	return aisx120sx_handle_none_acc_response_err(resp);
}

static int aisx120sx_reset(const struct aisx120sx_data *dev)
{
	union none_acc_response resp;
	int res;

	res = aisx120sx_raw_cmd_single(dev, OP_WRITE, REQ_NON_SENSOR_DATA,
				       AISX120SX_ADR_REG_RESET,
				       AISX120SX_RESET_SEQ_1, &resp);
	if (res != 0) {
		return res;
	}

	res = aisx120sx_raw_cmd_single(dev, OP_WRITE, REQ_NON_SENSOR_DATA,
				       AISX120SX_ADR_REG_RESET,
				       AISX120SX_RESET_SEQ_2, &resp);
	if (res != 0) {
		return res;
	}

	res = aisx120sx_raw_cmd_single(dev, OP_WRITE, REQ_NON_SENSOR_DATA,
				       AISX120SX_ADR_REG_RESET,
				       AISX120SX_RESET_SEQ_3, &resp);

	if (res != 0) {
		return res;
	}

	/* Wait for chip to be out of initialization phase */
	while (1) {
		union none_acc_response resp;
		union aisx120sx_reg_status_0 reg_status;

		k_sleep(100);

		aisx120sx_raw_cmd(dev, OP_READ, REQ_NON_SENSOR_DATA,
				  AISX120SX_ADR_REG_STATUS_0, 0, &resp);

		if (res != 0) {
			return res;
		}

		reg_status.reg = resp.fields.read_data;

		LOG_DBG("value: %X", reg_status.reg);
		LOG_DBG("status: %X", reg_status.fields.STATUS);

		if (reg_status.fields.END_OF_PWRUP) {
			break;
		}
	}

	return 0;
}

static int aisx120sx_attr_set(struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr,
			      const struct sensor_value *val)
{
	const struct aisx120sx_data *data = dev->driver_data;
	union none_acc_response resp;
	int res = 0;

	if (attr == (enum sensor_attribute) AISX120SX_ATTR_RESET) {
		return aisx120sx_reset(data);
	}

	if (attr == (enum sensor_attribute) AISX120SX_ATTR_END_OF_INIT) {
		return aisx120sx_raw_cmd(data, OP_WRITE, REQ_NON_SENSOR_DATA,
					 AISX120SX_ADR_REG_CTRL_0, 1, &resp);
	}

	res = aisx120sx_raw_cmd(data, OP_READ, REQ_NON_SENSOR_DATA,
				AISX120SX_ADR_REG_CTRL_0, 0, &resp);

	if (res != 0) {
		return res;
	}

	/* If end of initialization bit is set, the following attributes can
	 * not be set
	 */
	if (resp.fields.read_data != 0) {
		LOG_ERR(
			"Can not set requested attribute because device is not in initialization mode");
		return -EPERM;
	}

	res = aisx120sx_raw_cmd(data, OP_READ, REQ_NON_SENSOR_DATA,
				AISX120SX_ADR_REG_CONFIG, 0, &resp);

	if (res != 0) {
		return res;
	}

	union aisx120sx_reg_config reg = { .reg = resp.fields.read_data };

	if ((chan == SENSOR_CHAN_ACCEL_Y || chan == SENSOR_CHAN_ALL) &&
	    !data->is_dual_channel) {
		return -ENOTSUP;
	}

	if (attr == (enum sensor_attribute) AISX120SX_ATTR_BW) {
		u8_t field_val;

		switch (val->val1) {
		case 400:
			field_val = AISX120SX_BW_400Hz;
			break;
		case 800:
			field_val = AISX120SX_BW_800Hz;
			break;
		case 1600:
			field_val = AISX120SX_BW_1600Hz;
			break;
		default:
			return -EINVAL;
		}

		if (chan == SENSOR_CHAN_ACCEL_X) {
			reg.fields.FIR_BW_SEL_CHX = field_val;
		} else if (chan == SENSOR_CHAN_ACCEL_Y) {
			reg.fields.FIR_BW_SEL_CHY = field_val;
		} else if (chan == SENSOR_CHAN_ALL) {
			reg.fields.FIR_BW_SEL_CHX = field_val;
			reg.fields.FIR_BW_SEL_CHY = field_val;
		} else {
			return -EINVAL;
		}
	} else if (attr == (enum sensor_attribute) AISX120SX_ATTR_OFFSET_CANC) {
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}

		if (chan == SENSOR_CHAN_ACCEL_X) {
			reg.fields.DIS_OFF_CANC_CHX = val->val1;
		} else if (chan == SENSOR_CHAN_ACCEL_Y) {
			reg.fields.DIS_OFF_CANC_CHY = val->val1;
		} else if (chan == SENSOR_CHAN_ALL) {
			reg.fields.DIS_OFF_CANC_CHX = val->val1;
			reg.fields.DIS_OFF_CANC_CHY = val->val1;
		} else {
			return -EINVAL;
		}
	} else if (attr ==
		   (enum sensor_attribute) AISX120SX_ATTR_OFFSET_MONIT) {
		if (val->val1 < 0 || val->val1 > 1) {
			return -EINVAL;
		}

		if (chan == SENSOR_CHAN_ACCEL_X) {
			reg.fields.DIS_OFF_MON_CHX = val->val1;
		} else if (chan == SENSOR_CHAN_ACCEL_Y) {
			reg.fields.DIS_OFF_MON_CHY = val->val1;
		} else if (chan == SENSOR_CHAN_ALL) {
			reg.fields.DIS_OFF_MON_CHX = val->val1;
			reg.fields.DIS_OFF_MON_CHY = val->val1;
		} else {
			return -EINVAL;
		}

	}

	return aisx120sx_raw_cmd(data, OP_WRITE, REQ_NON_SENSOR_DATA,
				 AISX120SX_ADR_REG_CONFIG, reg.reg, &resp);
}

static int aisx120sx_sample_fetch(struct device *dev,
				  enum sensor_channel channel)
{
	int res;
	union acc_response resp;
	struct aisx120sx_data *data = dev->driver_data;

	if ((channel == SENSOR_CHAN_ACCEL_Y || channel == SENSOR_CHAN_ALL) &&
	    !data->is_dual_channel) {
		return -EINVAL;
	}

	if (channel == SENSOR_CHAN_ACCEL_X || channel == SENSOR_CHAN_ALL) {
		res = aisx120sx_raw_cmd_acc(data, CHANNEL_X, REQ_SENSOR_DATA,
					    &resp);
		if (res != 0) {
			return res;
		}

		data->acc_x = (s16_t)(resp.fields.data << 2) / 4;
	}

	if (channel == SENSOR_CHAN_ACCEL_Y || channel == SENSOR_CHAN_ALL) {
		res = aisx120sx_raw_cmd_acc(data, CHANNEL_Y, REQ_SENSOR_DATA,
					    &resp);

		if (res != 0) {
			return res;
		}

		data->acc_y = (s16_t)(resp.fields.data << 2) / 4;
	}

	return 0;
}

static void aisx120sx_raw_to_sensor_val(const s16_t raw_value,
					struct sensor_value *val)
{
	s32_t raw = raw_value * (1000000 / 68);

	val->val1 = raw / 1000000;
	val->val2 = raw % 1000000;
}

static int aisx120sx_channel_get(struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	const struct aisx120sx_data *data = dev->driver_data;

	if (chan == SENSOR_CHAN_ACCEL_X) {
		aisx120sx_raw_to_sensor_val(data->acc_x, val);
		return 0;
	} else if (chan == SENSOR_CHAN_ACCEL_Y && data->is_dual_channel) {
		aisx120sx_raw_to_sensor_val(data->acc_y, val);
		return 0;
	}

	return -EINVAL;
}

static int aisx120sx_get_dev_version(struct aisx120sx_data *data)
{
	int res;
	union none_acc_response resp;

	data->is_dual_channel = false;

	res = aisx120sx_raw_cmd(data, OP_READ, REQ_NON_SENSOR_DATA,
				AISX120SX_ADR_REG_ID_SENSOR_TYPE, 0, &resp);

	if (res != 0) {
		return res;
	}

	LOG_DBG("sensor version: 0x%X", resp.fields.read_data);
	if (resp.fields.read_data == AISX120SX_SENSOR_DUAL_CHANNEL) {
		data->is_dual_channel = true;
	}

	return 0;
}

static int check_self_test_value(float value, enum aisx120sx_self_test_cmd cmd)
{
	switch (cmd) {
	case SELF_TEST_ZERO:
		if (value >= SELF_TEST_MAX_OFFSET ||
		    value <= -SELF_TEST_MAX_OFFSET) {
			return -1;
		}
	break;
	case SELF_TEST_POS:
		if (fabs((SELF_TEST_EXPACTATION - value) /
				SELF_TEST_EXPACTATION) > SELF_TEST_TOLERANCE) {
			return -1;
		}
	break;
	case SELF_TEST_NEG:
		if (fabs((-SELF_TEST_EXPACTATION - value) /
				SELF_TEST_EXPACTATION) > SELF_TEST_TOLERANCE) {
			return -1;
		}
	break;
	}

	return 0;
}

static int aisx120sx_self_test_single(struct device *dev,
				      enum sensor_channel chan)
{
	int res;
	union aisx120sx_reg_ctrl_1 reg;
	union none_acc_response resp;
	struct sensor_value val;

	const struct aisx120sx_data *data = dev->driver_data;
	u8_t self_test_cmds[] = { SELF_TEST_ZERO, SELF_TEST_POS, SELF_TEST_ZERO,
				  SELF_TEST_NEG, SELF_TEST_ZERO };
	int offset = 0;

	if (chan == SENSOR_CHAN_ACCEL_Y) {
		offset = 4;
	}

	res = aisx120sx_raw_cmd(data, OP_READ, REQ_NON_SENSOR_DATA,
				AISX120SX_ADR_REG_CTRL_1, 0, &resp);

	if (res != 0) {
		return res;
	}

	if (resp.fields.status_flags != NONE_ACC_STAT_NORMAL) {
		return -EFAULT;
	}

	reg.reg = resp.fields.read_data;

	for (int i = 0; i < ARRAY_SIZE(self_test_cmds); i++) {
		reg.fields.SELF_TEST_CMD = self_test_cmds[i] + offset;

		res = aisx120sx_raw_cmd(data, OP_WRITE, REQ_NON_SENSOR_DATA,
					AISX120SX_ADR_REG_CTRL_1, reg.reg,
					&resp);

		if (res != 0) {
			return res;
		}

		k_sleep(K_MSEC(4));

		res = aisx120sx_sample_fetch(dev, SENSOR_CHAN_ALL);

		if (res != 0) {
			return res;
		}

		res = aisx120sx_channel_get(dev, SENSOR_CHAN_ACCEL_X, &val);

		if (res != 0) {
			return res;
		}

		float test_value = sensor_value_to_double(&val);

		res = check_self_test_value(test_value, self_test_cmds[i]);

		if (res != 0) {
			return res;
		}

		LOG_DBG("test sensor value: %d.%06d", val.val1,
			abs(val.val2));
	}
	return 0;
}

static int aisx120sx_self_test(struct device *dev)
{
	int res;
	const struct aisx120sx_data *data = dev->driver_data;

	res = aisx120sx_self_test_single(dev, SENSOR_CHAN_ACCEL_X);

	if (res != 0) {
		goto self_test_fail;
	}

	if (data->is_dual_channel) {
		res = aisx120sx_self_test_single(dev, SENSOR_CHAN_ACCEL_Y);
		if (res != 0) {
			goto self_test_fail;
		}
	}

	return 0;

self_test_fail:
	LOG_ERR("self test failed");
	return res;
}

static int aisx120sx_init(struct device *dev)
{
	int res = 0;
	struct aisx120sx_data *data =  dev->driver_data;

	data->spi_dev = device_get_binding(aisx120sx_config.aisx120sx_dev_name);
	BUILD_ASSERT_MSG(DT_INST_0_ST_AISX120SX_SPI_MAX_FREQUENCY <= 5000000,
			 "Spi clock frequency to high for aisx120sx sensor. Maximum value allowed: 5 Mhz");

	/* different polynom as in datasheet because of different
	 * representations (ommitting coefficient of x^8 but x^0)
	 */
	make_crc_table(0x2F);

#if defined(DT_INST_0_ST_AISX120SX_CS_GPIOS_CONTROLLER)
	aisx120sx_cs_ctrl.gpio_dev = device_get_binding(
		DT_INST_0_ST_AISX120SX_CS_GPIOS_CONTROLLER);
	if (!aisx120sx_cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		res = -ENODEV;
		goto init_error;
	}

	aisx120sx_cs_ctrl.gpio_pin = DT_INST_0_ST_AISX120SX_CS_GPIOS_PIN;
	aisx120sx_cs_ctrl.delay = 0U;

	aisx120sx_spi_conf.cs = &aisx120sx_cs_ctrl;

	LOG_DBG("SPI GPIO CS configured on %s:%u",
		DT_INST_0_ST_AISX120SX_CS_GPIOS_CONTROLLER,
		DT_INST_0_ST_AISX120SX_CS_GPIOS_PIN);
#endif
	if (k_uptime_get() < 10) {
		k_sleep(K_MSEC(10));
	}

	res = aisx120sx_reset(data);
	if (res != 0) {
		return res;
	}

	res = aisx120sx_get_dev_version(data);
	if (res != 0) {
		goto init_error;
	}

	res = aisx120sx_self_test(dev);

	if (res != 0) {
		goto init_error;
	}

	struct sensor_value val;

	val.val1 = AISX120SX_BW_X_DEFAULT;
	res = aisx120sx_attr_set(dev, SENSOR_CHAN_ACCEL_X,
				 (enum sensor_attribute) AISX120SX_ATTR_BW,
				 &val);

	if (res != 0) {
		goto init_error;
	}

	if (data->is_dual_channel) {
		val.val1 = AISX120SX_BW_Y_DEFAULT;
		res = aisx120sx_attr_set(dev,
					 SENSOR_CHAN_ACCEL_Y,
					 (enum sensor_attribute)
					 AISX120SX_ATTR_BW,
					 &val);
	}

	val.val1 = 1;
	#if CONFIG_AISX120SX_DISABLE_OFF_MON_X
	res = aisx120sx_attr_set(dev,
				 SENSOR_CHAN_ACCEL_X,
				 (enum sensor_attribute)
				 AISX120SX_ATTR_OFFSET_MONIT,
				 &val);
	if (res != 0) {
		goto init_error;
	}
	#endif

	#if CONFIG_AISX120SX_DISABLE_OFF_MON_Y
	if (data->is_dual_channel) {
		res = aisx120sx_attr_set(dev,
					 SENSOR_CHAN_ACCEL_Y,
					 (enum sensor_attribute)
					 AISX120SX_ATTR_OFFSET_MONIT,
					 &val);
		if (res != 0) {
			goto init_error;
		}
	}
	#endif

	#if CONFIG_AISX120SX_DISABLE_OFF_CANC_X
	res = aisx120sx_attr_set(dev,
				 SENSOR_CHAN_ACCEL_X,
				 (enum sensor_attribute)
				 AISX120SX_ATTR_OFFSET_CANC,
				 &val);
	if (res != 0) {
		goto init_error;
	}
	#endif

	#if CONFIG_AISX120SX_DISABLE_OFF_CANC_Y
	if (data->is_dual_channel) {
		res = aisx120sx_attr_set(dev,
					 SENSOR_CHAN_ACCEL_Y,
					 (enum sensor_attribute)
					 AISX120SX_ATTR_OFFSET_CANC,
					 &val);
		if (res != 0) {
			goto init_error;
		}
	}
	#endif

	return aisx120sx_attr_set(dev, SENSOR_CHAN_ALL,
				  (enum sensor_attribute)
				  AISX120SX_ATTR_END_OF_INIT,
				  &val);

init_error:
	LOG_ERR("Error initializing device");
	return res;
}

static const struct sensor_driver_api aisx120sx_api_funcs = {
	.attr_set = aisx120sx_attr_set,
	.sample_fetch = aisx120sx_sample_fetch,
	.channel_get = aisx120sx_channel_get,
};

static struct aisx120sx_data aisx120sx_data;

DEVICE_AND_API_INIT(aisx120sx,
		    DT_INST_0_ST_AISX120SX_LABEL,
		    aisx120sx_init,
		    &aisx120sx_data,
		    &aisx120sx_config,
		    POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY,
		    &aisx120sx_api_funcs);
