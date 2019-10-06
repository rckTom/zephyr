#ifndef __BMI088_GYRO_H__
#define __BMI088_GYRO_H__

#include <zephyr/types.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#define UBIT(n) (1u << n)

#define BMI088_REG_GYRO_SELF_TEST      (u8_t) 0x3C
#define BMI088_REG_INT3_INT4_IO_MAP    (u8_t) 0x18
#define BMI088_REG_INT3_INT4_IO_CONF   (u8_t) 0x16
#define BMI088_REG_GYRO_INT_CTRL       (u8_t) 0x15
#define BMI088_REG_GYR0_SOFTRESET      (u8_t) 0x14
#define BMI088_REG_GYRO_LPM1           (u8_t) 0x11
#define BMI088_REG_GYRO_BANDWITH       (u8_t) 0x10
#define BMI088_REG_GYRO_RANGE          (u8_t) 0x0F
#define BMI088_REG_GYRO_INT_STAT_1     (u8_t) 0x0A
#define BMI088_REG_RATE_Z_MSB          (u8_t) 0x07
#define BMI088_REG_RATE_Z_LSB          (u8_t) 0x06
#define BMI088_REG_RATE_Y_MSB          (u8_t) 0x05
#define BMI088_REG_RATE_Y_LSB          (u8_t) 0x04
#define BMI088_REG_RATE_X_MSB          (u8_t) 0x03
#define BMI088_REG_RATE_X_LSB          (u8_t) 0x02
#define BMI088_REG_GYRO_CHIP_ID        (u8_t) 0x00

#define BMI088_MASK_GYRO_SELF_TEST_TRIG_BIST    UBIT(0)
#define BMI088_MASK_GYRO_SELF_TEST_BIST_RDY     UBIT(1)
#define BMI088_MASK_GYRO_SELF_TEST_BIST_FAIL    UBIT(2)
#define BMI088_MASK_GYRO_SELF_TEST_RATE_OK      UBIT(4)

#define BMI088_GYRO_RANGE_2000         (u8_t) 0x00
#define BMI088_GYRO_RANGE_1000         (u8_t) 0x01
#define BMI088_GYRO_RANGE_500          (u8_t) 0x02
#define BMI088_GYRO_RANGE_250          (u8_t) 0x03
#define BMI088_GYRO_RANGE_125          (u8_t) 0x04

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
#   define BMI088_GYRO_DEFAULT_BANDWITH    BMI088_GYRO_BANDWITH_230
#endif

enum BMI088_GYRO_ATTR {
    BMI088_GYRO_ATTR_BW = SENSOR_ATTR_PRIV_START + 1,
};

struct bmi088_gyr_data {
    struct device *bmi088_com_dev;
    const struct bmi088_transfer_function *tf;

    u8_t gyro_odr;
    u8_t gyro_range;

    union {
        struct {
            s16_t x_value;
            s16_t y_value;
            s16_t z_value;
        }__attribute__((packed));
        s16_t array[3];
    } gyro_raw_values;

    union {
        struct {
            struct sensor_value x_value;
            struct sensor_value y_value;
            struct sensor_value z_value;
        }__attribute__((packed));
        struct sensor_value array[3];
    } gyro_values;
};

struct bmi088_transfer_function {
    int (*read_register)(struct bmi088_gyr_data *data, u16_t reg,
                         int count, void *val);
    int (*write_register)(struct bmi088_gyr_data *data, u16_t reg,
                         int count, void *val);
};

struct bmi088_gyr_config {
    char *bmi088_com_dev_name;
};

int bmi088_gyr_spi_init(struct bmi088_gyr_data *data);

#endif