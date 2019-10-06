#include "bmi088_gyro.h"
#include <drivers/spi.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(bmi088_gyro);

#define SPI_CS NULL
#if defined(DT_INST_0_MEAS_MS5607_CS_GPIOS_CONTROLLER)
static struct spi_cs_control bmi088_gyr_cs_ctrl;
#endif

static struct spi_config bmi088_gyr_spi_conf = {
    .frequency = DT_INST_0_BOSCH_BMI088_GYRO_SPI_MAX_FREQUENCY,
    .operation = (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
                  SPI_MODE_CPOL | SPI_MODE_CPHA |
                  SPI_TRANSFER_MSB |
                  SPI_LINES_SINGLE),
    .slave     = DT_INST_0_BOSCH_BMI088_GYRO_BASE_ADDRESS,
    .cs        = SPI_CS,
};

int read_register(struct bmi088_gyr_data *data, u16_t reg, int count, void *val)
{
    int res;
    struct spi_buf_set rx_buf_set;
    struct spi_buf_set tx_buf_set;
    struct spi_buf tx_buf[2];
    struct spi_buf rx_buf[2];

    rx_buf_set.buffers = rx_buf;
    tx_buf_set.buffers = tx_buf;

    reg = reg | (1 << 7);

    rx_buf[0].buf = NULL;
    rx_buf[0].len = 1;
    rx_buf[1].buf = val;
    rx_buf[1].len = count;
    rx_buf_set.count = 2;

    tx_buf[0].buf = &reg;
    tx_buf[0].len = 1;
    tx_buf[1].buf = NULL;
    tx_buf[1].len = count;
    tx_buf_set.count = 2;

    res = spi_transceive(data->bmi088_com_dev,
                   &bmi088_gyr_spi_conf,
                   &tx_buf_set,
                   &rx_buf_set);
    
    if (res != 0){
        return res;
    }

    return 0;
}

int write_register(struct bmi088_gyr_data *data, u16_t reg, int count,
                   void *val)
{
    int res;
    struct spi_buf_set tx_buf_set;
    struct spi_buf_set rx_buf_set;

    reg = reg & ~(1<<7);

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
                         &bmi088_gyr_spi_conf,
                         &tx_buf_set,
                         &rx_buf_set);

    if (res != 0) {
        return res;
    }

    return 0;
}

static const struct bmi088_transfer_function bmi088_gyr_spi_tf = {
    .read_register = read_register,
    .write_register = write_register,
};

int bmi088_gyr_spi_init(struct bmi088_gyr_data *data)
{
    data->tf = &bmi088_gyr_spi_tf;

#if defined(DT_INST_0_BOSCH_BMI088_GYRO_CS_GPIOS_CONTROLLER)
    bmi088_gyr_cs_ctrl.gpio_dev = device_get_binding(
        DT_INST_0_BOSCH_BMI088_GYRO_CS_GPIOS_CONTROLLER
    );
    
    if (!bmi088_gyr_cs_ctrl.gpio_dev) {
        LOG_ERR("Unable to get gpio device for chip select");
        return -ENODEV;
    }
    
    bmi088_gyr_cs_ctrl.gpio_pin = DT_INST_0_BOSCH_BMI088_GYRO_CS_GPIOS_PIN;
    bmi088_gyr_cs_ctrl.delay = 1U;
    bmi088_gyr_spi_conf.cs = &bmi088_gyr_cs_ctrl;
#endif
    return 0;
}
