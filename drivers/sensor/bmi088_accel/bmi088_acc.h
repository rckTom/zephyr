#ifndef __BMI088_GYRO_H__
#define __BMI088_GYRO_H__

#include <zephyr/types.h>
#include <zephyr.h>
#include <device.h>


/* The registers are prefixed with a bit that determins  if the register is
part of the acceleration module or gyro module. This is necessarey because spi
opearations are different on accelration registers than on gyro registers */

#define BMI088_REG_ACC_SOFTRESET       (u16_t) 0x7E | ((u16_t)1 << 8)
#define BMI088_REG_ACC_PWR_CTRL        (u16_t) 0x7D | ((u16_t)1 << 8)
#define BMI088_REG_ACC_PWR_CONF        (u16_t) 0x7C | ((u16_t)1 << 8)
#define BMI088_REG_ACC_SELF_TEST       (u16_t) 0x6D | ((u16_t)1 << 8)
#define BMI088_REG_INT_MAP_DATA        (u16_t) 0x58 | ((u16_t)1 << 8)
#define BMI088_REG_INT2_IO_CTRL        (u16_t) 0x54 | ((u16_t)1 << 8)
#define BMI088_REG_INT1_IO_CTRL        (u16_t) 0x53 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_RANGE           (u16_t) 0x41 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_CONF            (u16_t) 0x40 | ((u16_t)1 << 8)
#define BMI088_REG_TEMP_LSB            (u16_t) 0x23 | ((u16_t)1 << 8)
#define BMI088_REG_TEMP_MSB            (u16_t) 0x22 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_INT_STAT_1      (u16_t) 0x1D | ((u16_t)1 << 8)
#define BMI088_REG_SENSORTIME_2        (u16_t) 0x1A | ((u16_t)1 << 8)
#define BMI088_REG_SENSORTIME_1        (u16_t) 0x19 | ((u16_t)1 << 8)
#define BMI088_REG_SENSORTIME_0        (u16_t) 0x18 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_Z_MSB           (u16_t) 0x17 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_Z_LSB           (u16_t) 0x16 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_Y_MSB           (u16_t) 0x15 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_Y_LSB           (u16_t) 0x14 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_X_MSB           (u16_t) 0x13 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_X_LSB           (u16_t) 0x12 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_STATUS          (u16_t) 0x03 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_ERR_REG         (u16_t) 0x02 | ((u16_t)1 << 8)
#define BMI088_REG_ACC_CHIP_ID         (u16_t) 0x00 | ((u16_t)1 << 8)

#define BMI088_MASK_ACC_CONF_BANDWITH  0b11110000
#define BMI088_MASK_ACC_CONF_ODR       0b00001111
#define BMI088_MASK_ACC_RANGE          0b00000011

#define BMI088_ACC_CONF_BANDWIDTH_OSR4 0x80
#define BMI088_ACC_CONF_BANDWIDTH_OSR2 0x90
#define BMI088_ACC_CONF_BANDWIDTH_OSR1 0xA0

#define BMI088_ACC_CONF_ODR_12_5       0x05
#define BMI088_ACC_CONF_ODR_25         0x06
#define BMI088_ACC_CONF_ODR_50         0x07
#define BMI088_ACC_CONF_ODR_100        0x08
#define BMI088_ACC_CONF_ODR_200        0x09
#define BMI088_ACC_CONF_ODR_400        0x0A
#define BMI088_ACC_CONF_ODR_800        0x0B
#define BMI088_ACC_CONF_ODR_1600       0x0C

#define BMI088_ACC_RANGE_3             0x00
#define BMI088_ACC_RANGE_6             0x01
#define BMI088_ACC_RANGE_12            0x02
#define BMI088_ACC_RANGE_24            0x03

#define BMI088_REG_GYRO_SELF_TEST      (u16_t) 0x3C
#define BMI088_REG_INT3_INT4_IO_MAP    (u16_t) 0x18
#define BMI088_REG_INT3_INT4_IO_CONF   (u16_t) 0x16
#define BMI088_REG_GYRO_INT_CTRL       (u16_t) 0x15
#define BMI088_REG_GYR0_SOFTRESET      (u16_t) 0x14
#define BMI088_REG_GYRO_LPM1           (u16_t) 0x11
#define BMI088_REG_GYRO_BANDWITH       (u16_t) 0x10
#define BMI088_REG_GYRO_RANGE          (u16_t) 0x0F
#define BMI088_REG_GYRO_INT_STAT_1     (u16_t) 0x0A
#define BMI088_REG_RATE_Z_MSB          (u16_t) 0x07
#define BMI088_REG_RATE_Z_LSB          (u16_t) 0x06
#define BMI088_REG_RATE_Y_MSB          (u16_t) 0x05
#define BMI088_REG_RATE_Y_LSB          (u16_t) 0x04
#define BMI088_REG_RATE_X_MSB          (u16_t) 0x03
#define BMI088_REG_RATE_X_LSB          (u16_t) 0x02
#define BMI088_REG_GYRO_CHIP_ID        (u16_t) 0x00


#define BMI088_GYRO_RANGE_2000         0x00
#define BMI088_GYRO_RANGE_1000         0x01
#define BMI088_GYRO_RANGE_500          0x02
#define BMI088_GYRO_RANGE_250          0x03
#define BMI088_GYRO_RANGE_125          0x04

#define BMI088_GYRO_BANDWITH_532       0x00
#define BMI088_GYRO_BANDWITH_230       0x01
#define BMI088_GYRO_BANDWITH_116       0x02
#define BMI088_GYRO_BANDWITH_47        0x03
#define BMI088_GYRO_BANDWITH_23        0x04
#define BMI088_GYRO_BANDWITH_12        0x05
#define BMI088_GYRO_BANDWITH_64        0x06
#define BMI088_GYRO_BANDWITH_32        0x07

#define BMI088_GYRO_LPM1_NORMAL        0x00
#define BMI088_GYRO_LPM1_SUSPEND       0x80
#define BMI088_GYRO_LPM1_DEEP_SUSPEND  0x20

#if defined(CONFIG_BMI088_ACCEL_RANGE_3G)
#   define BMI088_ACC_DEFAULT_RANGE           BMI088_ACC_CONF_RANGE_3
#elif defined(CONFIG_BMI0088_ACCEL_RANGE_6G)
#   define BMI088_ACC_DEFAULT_RANGE           BMI088_ACC_CONF_RANGE_6
#elif defined(CONFIG_BMI0088_ACCEL_RANGE_12G)
#   define BMI088_ACC_DEFAULT_RANGE           BMI088_ACC_CONF_RANGE_12
#elif defined(CONFIG_BMI0088_ACCEL_RANGE_24G)
#   define BMI088_ACC_DEFAULT_RANGE           BMI088_ACC_CONF_RANGE_24
#else
#   define BMI088_ACC_DEFAULT_RANGE           BMI088_ACC_CONF_RANGE_24
#endif

#if defined(CONFIG_BMI088_ACCEL_BANDWITH_ODR_1)
#   define BMI088_ACC_DEFAULT_BANDWIDTH       BMI088_ACC_CONF_BANDWIDTH_OSR1
#elif defined(CONFIG_BMI088_ACCEL_BANDWITH_ODR_2)
#   define BMI088_ACC_DEFAULT_BANDWIDTH       BMI088_ACC_CONF_BANDWIDTH_OSR2 
#elif defined(CONFIG_BMI088_ACCEL_BANDWITH_ODR_4)
#   define BMI088_ACC_DEFAULT_BANDWIDTH       BMI088_ACC_CONF_BANDWIDTH_OSR4 
#else
#   define BMI088_ACC_DEFAULT_BANDWIDTH       BMI088_ACC_CONF_BANDWIDTH_OSR1
#endif

#if defined(CONFIG_BMI088_ACCEL_ODR_25_2)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_12_5
#elif defined(CONFIG_BMI088_ACCEL_ODR_25)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_25
#elif defined(CONFIG_BMI088_ACCEL_ODR_50)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_50
#elif defined(CONFIG_BMI088_ACCEL_ODR_100)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_100
#elif defined(CONFIG_BMI088_ACCEL_ODR_200)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_200
#elif defined(CONFIG_BMI088_ACCEL_ODR_400)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_400
#elif defined(CONFIG_BMI088_ACCEL_ODR_800)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_800
#elif defined(CONFIG_BMI088_ACCEL_ODR_1600)
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACC_CONF_ODR_1600
#else
#   define BMI088_ACC_DEFAULT_ODR          BMI088_ACCEL_ODR_200
#endif   

#if defined(CONFIG_BMI088_GYRO_RANGE_2000DPS)
#   define BMI088_GYRO_DEFAULT_RANGE       BMI088_GYRO_RANGE_2000
#elif defined(CONFIG_BMI088_GYRO_RANGE_1000DPS)
#   define BMI088_GYRO_DEFAULT_RANGE       BMI088_GYRO_RANGE_1000
#elif defined(CONFIG_BMI088_GYRO_RANGE_500DPS)
#   define BMI088_GYRO_DEFAULT_RANGE       BMI088_GYRO_RANGE_500
#elif defined(CONFIG_BMI088_GYRO_RANGE_250DPS)
#   define BMI088_GYRO_DEFAULT_RANGE       BMI088_GYRO_RANGE_250
#elif defined(CONFIG_BMI088_GYRO_RANGE_125DPS)
#   define BMI088_GYRO_DEFAULT_RANGE       BMI088_GYRO_RANGE_125
#else
#   define BMI088_GYRO_DEFAULT_RANGE       BMI088_GYRO_RANGE_1000
#endif

#if defined(CONFIG_BMI_GYRO_ODR_2000_BW_532)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_532
#elif defined(BMI088_GYRO_ODR_2000_BW_230)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_230
#elif defined(BMI088_GYRO_ODR_2000_BW_116)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_116
#elif defined(BMI088_GYRO_ODR_2000_BW_47)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_47
#elif defined(BMI088_GYRO_ODR_2000_BW_23)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_23
#elif defined(BMI088_GYRO_ODR_2000_BW_64)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_64
#elif defined(BMI088_GYRO_ODR_2000_BW_32)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_32
#elif defined(BMI088_GYRO_ODR_2000_BW_12)
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_12
#else
#   define BMI088_GYRO_DEFUALT_BANDWITH    BMI088_GYRO_BANDWITH_230
#endif

struct bmi088_transfer_function {
    int (*read_register)(struct bmi088_data *data, u16_t register,
                         int count, void *val);
    int (*write_register)(struct bmi088_data *data, u16_t register,
                         int count, void *val);
};

struct bmi088_config {
    char *bmi088_com_dev_name;
}

union bmi088_values {
    s16_t x_value;
    s16_t y_value;
    s16_t z_value;
    s16_t array[3];
};

struct bmi088_data {
    struct device *bmi088_com_dev;
    struct bmi088_transfer_function *tf;

    u8_t acc_bandwidth;
    u8_t acc_odr;
    u8_t acc_range;

    u8_t gyro_odr;
    u8_t gyro_range;

    union bmi088_values gyro_values;
    union bmi088_values acc_values;
};

#endif