
#define DT_DRV_COMPAT bosch_bmi088_accel

#include "bmi088_acc.h"
#include <drivers/spi.h>
#include <logging/log.h>
#include <drivers/sensor.h>

LOG_MODULE_DECLARE(bmi088_accel);

#define SPI_CS NULL
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
static struct spi_cs_control bmi088_acc_cs_ctrl;
#endif

static struct spi_config bmi088_acc_spi_conf = {
	.frequency = DT_INST_PROP(0, spi_max_frequency),
	.operation = (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		      SPI_MODE_CPOL | SPI_MODE_CPHA |
		      SPI_TRANSFER_MSB |
		      SPI_LINES_SINGLE),
	.slave = DT_INST_REG_ADDR(0),
	.cs = SPI_CS,
};

int read_register(struct bmi088_acc_data *data, uint16_t reg, int count, void *val)
{
	int res;
	struct spi_buf_set rx_buf_set;
	struct spi_buf_set tx_buf_set;
	struct spi_buf tx_buf[2];
	struct spi_buf rx_buf[2];

	rx_buf_set.buffers = rx_buf;
	tx_buf_set.buffers = tx_buf;

	reg |= (1 << 7);

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 2;
	rx_buf[1].buf = val;
	rx_buf[1].len = count;
	rx_buf_set.count = 2;

	tx_buf[0].buf = &reg;
	tx_buf[0].len = 1;
	tx_buf[1].buf = NULL;
	tx_buf[1].len = count + 1;
	tx_buf_set.count = 2;

	res = spi_transceive(data->bmi088_com_dev,
			     &bmi088_acc_spi_conf,
			     &tx_buf_set,
			     &rx_buf_set);

	if (res != 0) {
		return res;
	}

	return 0;
}

int write_register(struct bmi088_acc_data *data, uint16_t reg, int count,
		   void *val)
{
	int res;
	struct spi_buf_set tx_buf_set;
	struct spi_buf_set rx_buf_set;

	reg = reg & ~(1 << 7);

	struct spi_buf tx_buf[2] = {
		{
			.buf = &reg,
			.len = 1,
		},
		{
			.buf = val,
			.len = count,
		},
	};

	struct spi_buf rx_buf = {
		.buf = NULL,
		.len = count + 1,
	};

	tx_buf_set.buffers =  tx_buf;
	tx_buf_set.count = 2;
	rx_buf_set.buffers = &rx_buf;
	rx_buf_set.count = 1;

	res = spi_transceive(data->bmi088_com_dev,
			     &bmi088_acc_spi_conf,
			     &tx_buf_set,
			     &rx_buf_set);

	if (res != 0) {
		return res;
	}

	return 0;
}

static const struct bmi088_transfer_function bmi088_acc_spi_tf = {
	.read_register = read_register,
	.write_register = write_register,
};

int bmi088_acc_spi_init(struct bmi088_acc_data *data)
{
	data->tf = &bmi088_acc_spi_tf;

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	bmi088_acc_cs_ctrl.gpio_dev = device_get_binding(
		DT_INST_SPI_DEV_CS_GPIOS_LABEL(0)
		);

	if (!bmi088_acc_cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get gpio device for chip select");
		return -ENODEV;
	}

	bmi088_acc_cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	bmi088_acc_cs_ctrl.delay = 0U;
	bmi088_acc_cs_ctrl.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0);
	bmi088_acc_spi_conf.cs = &bmi088_acc_cs_ctrl;
#endif

	/* performe a dummy id read to set the device to spi mode */
	uint8_t reg =  BMI088_REG_ACC_CHIP_ID;
	uint8_t reg_val;

	int res = data->tf->read_register(data, reg, 1, &reg_val);

	return res;
}
