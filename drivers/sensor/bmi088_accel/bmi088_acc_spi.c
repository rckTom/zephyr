#include "bmi088.h"
#include <drivers/spi.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(bmi088);


static struct spi_config bmi088_acc_spi_conf = {
    .frequency = DT_INST_0_BOSCH_BMI088_SPI_MAX_FREQUENCY,
    .operation = (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
                  SPI_MODE_CPOL | SPI_MODE_CPHA |
                  SPI_TRANSFER_MSB |
                  SPI_LINES_SINGLE),
    .slave     = DT_INST_0_BOSCH_BMI088_BASE_ADDRESS,
    .cs        = SPI_CS,                  
};

static struct spi_config bmi088_gyro_spi_conf = {
    .frequency = DT_INST_1_BOSCH_BMI088_SPI_MAX_FREQUENCY,
    .operation = (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
                  SPI_MODE_CPOL | SPI_MODE_CPHA |
                  SPI_TRANSFER_MSB |
                  SPI_LINES_SINGLE),
    .slave     = DT_INST_1_BOSCH_BMI088_BASE_ADDRESS,
    .cs        = SPI_CS,
}
int read_register(struct bmi088_data *data, u16_t reg, int count, void *val)
{
    int res;
    u8_t num_pad = 1;
    struct spi_config *spi_conf = &bmi088_gyro_spi_conf;
    struct spi_buf_set rx_buf_set;
    struct spi_buf_set tx_buf_set;
    struct spi_buf tx_buf[2];
    struct spi_buf rx_buf[2];

    rx_buf_set.buffers = rx_buf;
    tx_buf_set.buffers = tx_buf;

    reg |= (1 << 7);

    if(reg & (1<<8)) {
        spi_conf = &bmi088_acc_spi_conf;
        num_pad = 2;
    }

    rx_buf[0].buf = NULL;
    rx_buf[0].len = num_pad;
    rx_buf[1].buf = val;
    rx_buf[1].len = count;
    rx_buf_set.count = 2;

    tx_buf[0].buf = &reg;
    tx_buf[0].len = 1;
    tx_buf[1].buf = NULL;
    tx_buf[1].len = num_pad - 1 + count;
    tx_buf_set.count = 2;

    res = spi_transceive(data->bmi088_com_dev,
                   spi_conf,
                   &tx_buf_set,
                   &rx_buf_set);
    
    if (res != 0){
        return res;
    }

    return 0;
}

int write_register(struct bmi088_data *data, u16_t reg, int count, void *val)
{
    int res;

    struct spi_config *spi_conf = &bmi088_gyro_spi_conf;
    if (reg & (1<<8)) {
        spi_conf = &bmi088_acc_spi_conf;
    }

    struct spi_buf_set tx_buf_set;
    struct spi_buf_set rx_buf_set;
    struct spi_buf tx_buf[2] = {
        {
            .buf = reg,
            .len = 1,
        },
        {
            .buf = val,
            .len = count,
        },
    };

    struct spi_buf rx_buf = {
        .buf = NULL;
        .len = count + 1;
    };

    tx_buf_set.buffers =  tx_buf;
    tx_buf_set.count = 2;
    rx_buf_set.buffers = &rx_buf;
    rx_buf_set.count = 1;

    res = spi_transceive(data->bmi088_com_dev,
                         spi_conf,
                         &tx_buf_set,
                         &rx_buf_set);

    if (res != 0) {
        return res;
    }

    return 0;
}

static const struct bmi088_transfer_function bmi088_spi_tf = {
    .read_register = read_register,
    .write_register = write_register,
};

int bmi088_spi_init(struct bmi088_data *data)
{
    data->tf = &bmi088_spi_tf;
}