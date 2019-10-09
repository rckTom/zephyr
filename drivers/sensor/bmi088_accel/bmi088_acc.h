#ifndef __BMI088_GYRO_H__
#define __BMI088_GYRO_H__

#include <zephyr/types.h>
#include <zephyr.h>
#include <device.h>

#define BMI088_REG_ACC_SOFTRESET       (u8_t) 0x7E
#define BMI088_REG_ACC_PWR_CTRL        (u8_t) 0x7D
#define BMI088_REG_ACC_PWR_CONF        (u8_t) 0x7C
#define BMI088_REG_ACC_SELF_TEST       (u8_t) 0x6D
#define BMI088_REG_INT_MAP_DATA        (u8_t) 0x58
#define BMI088_REG_INT2_IO_CONF        (u8_t) 0x54
#define BMI088_REG_INT1_IO_CONF        (u8_t) 0x53
#define BMI088_REG_ACC_RANGE           (u8_t) 0x41
#define BMI088_REG_ACC_CONF            (u8_t) 0x40
#define BMI088_REG_TEMP_LSB            (u8_t) 0x23
#define BMI088_REG_TEMP_MSB            (u8_t) 0x22
#define BMI088_REG_ACC_INT_STAT_1      (u8_t) 0x1D
#define BMI088_REG_SENSORTIME_2        (u8_t) 0x1A
#define BMI088_REG_SENSORTIME_1        (u8_t) 0x19
#define BMI088_REG_SENSORTIME_0        (u8_t) 0x18
#define BMI088_REG_ACC_Z_MSB           (u8_t) 0x17
#define BMI088_REG_ACC_Z_LSB           (u8_t) 0x16
#define BMI088_REG_ACC_Y_MSB           (u8_t) 0x15
#define BMI088_REG_ACC_Y_LSB           (u8_t) 0x14
#define BMI088_REG_ACC_X_MSB           (u8_t) 0x13
#define BMI088_REG_ACC_X_LSB           (u8_t) 0x12
#define BMI088_REG_ACC_STATUS          (u8_t) 0x03
#define BMI088_REG_ACC_ERR_REG         (u8_t) 0x02
#define BMI088_REG_ACC_CHIP_ID         (u8_t) 0x00

#define BMI088_MASK_ACC_CONF_BANDWITH  (u8_t) 0b11110000
#define BMI088_MASK_ACC_CONF_ODR       (u8_t) 0b00001111
#define BMI088_MASK_ACC_RANGE          (u8_t) 0b00000011

#define BMI088_MASK_INT_IO_CONF_IN     (u8_t) 0b00010000
#define BMI088_MASK_INT_IO_CONF_OUT    (u8_t) 0b00001000
#define BMI088_MASK_INT_IO_CONF_OD     (u8_t) 0b00000100
#define BMI088_MASK_INT_IO_CONF_LVL    (u8_t) 0b00000010

#define BMI088_MASK_INT_MAP_DATA_INT2_DRDY      (u8_t) 0b01000000
#define BMI088_MASK_INT_MAP_DATA_INT1_DRDY      (u8_t) 0b01000010

#define BMI088_ACC_CONF_BANDWIDTH_OSR4 ((u8_t) 0x08 << 4)
#define BMI088_ACC_CONF_BANDWIDTH_OSR2 ((u8_t) 0x09 << 4)
#define BMI088_ACC_CONF_BANDWIDTH_OSR1 ((u8_t) 0x0A << 4)

#define BMI088_ACC_SELF_TEST_OFF       (u8_t) 0x00
#define BMI088_ACC_SELF_TEST_POS       (u8_t) 0x0D
#define BMI088_ACC_SELF_TEST_NEG       (u8_t) 0x09

#define BMI088_ACC_PWR_CONF_SUSPEND    (u8_t) 0x03
#define BMI088_ACC_PWR_CONF_ACTIVE     (u8_t) 0x00
#define BMI088_ACC_PWR_CTRL_ACC_OFF    (u8_t) 0x00
#define BMI088_ACC_PWR_CTRL_ACC_ON     (u8_t) 0x04

#define BMI088_ACC_SOFTRESET_RESET     (u8_t) 0xB6

#define BMI088_ACC_CONF_ODR_12_5       (u8_t) 0x05
#define BMI088_ACC_CONF_ODR_25         (u8_t) 0x06
#define BMI088_ACC_CONF_ODR_50         (u8_t) 0x07
#define BMI088_ACC_CONF_ODR_100        (u8_t) 0x08
#define BMI088_ACC_CONF_ODR_200        (u8_t) 0x09
#define BMI088_ACC_CONF_ODR_400        (u8_t) 0x0A
#define BMI088_ACC_CONF_ODR_800        (u8_t) 0x0B
#define BMI088_ACC_CONF_ODR_1600       (u8_t) 0x0C

#define BMI088_ACC_RANGE_3             (u8_t) 0x00
#define BMI088_ACC_RANGE_6             (u8_t) 0x01
#define BMI088_ACC_RANGE_12            (u8_t) 0x02
#define BMI088_ACC_RANGE_24            (u8_t) 0x03

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


struct bmi088_acc_data {
    struct device *bmi088_com_dev;
    const struct bmi088_transfer_function *tf;

    u8_t acc_odr;
    u8_t acc_bandwith;
    u8_t acc_range;

    union {
        struct {
            s16_t x_value;
            s16_t y_value;
            s16_t z_value;
        }__attribute__((packed));
        s16_t array[3];
    } acc_raw_values;

    union {
        struct {
            struct sensor_value x_value;
            struct sensor_value y_value;
            struct sensor_value z_value;
        }__attribute__((packed));
        struct sensor_value array[3];
    } acc_values;

    u16_t temp_raw;
    struct sensor_value temp;
};

struct bmi088_transfer_function {
    int (*read_register)(struct bmi088_data *data, u16_t reg,
                         int count, void *val);
    int (*write_register)(struct bmi088_data *data, u16_t reg,
                         int count, void *val);
};

struct bmi088_acc_config {
    char *bmi088_com_dev_name;
}

int bmi088_acc_spi_init(struct bmi088_acc_data *data);

#endif