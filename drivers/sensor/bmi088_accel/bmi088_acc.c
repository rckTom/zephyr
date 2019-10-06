#include <logging/log.h>
#include "bmi088.h"

LOG_MODULE_REGISTER(bmi088, CONFIG_SENSOR_LOG_LEVEL);

static int bmi088_sample_fetch()
{

}

static int bmi088_init(struct device *dev)
{
    struct bmi088_data *data = dev->driver_data;
    struct bmi088_config *config = dev->config->config_info;

    data->bmi088_com_dev = device_get_binding(config->bmi088_com_dev_name);
    if (!data->bmi088_com_dev){
        LOG_ERR("Communicaton  master device not found");
        return EINVAL;
    }

    memset(data->gyro_values.array, 0, ARRAY_SIZE(data->gyro_values.array));
    memset(data->acc_values.array, 0, ARRAY_SIZE(data->acc_values.array));

    data->gyro_odr =        BMI088_GYRO_DEFUALT_BANDWITH;
    data->gyro_range =      BMI088_GYRO_DEFAULT_RANGE;
    data->acc_odr =         BMI088_ACC_DEFAULT_ODR;
    data->acc_range =       BMI088_ACC_DEFAULT_RANGE;
    data->acc_bandwidth =   BMI088_ACC_DEFAULT_BANDWIDTH;

#ifdef DT_BOSCH_BMI088_BUS_SPI
    bmi088_spi_init(data);
#else
    bmi088_i2c_init(data);
#endif

    return  0;
}

