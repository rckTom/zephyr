#ifndef __BMI088_GYRO_H__
#define __BMI088_GYRO_H__

#include <zephyr/types.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#define UBIT(n) (1u << n)

#define BMI088_REG_GYRO_SELF_TEST      ((uint8_t) 0x3C)
#define BMI088_REG_INT3_INT4_IO_MAP    ((uint8_t) 0x18)
#define BMI088_REG_INT3_INT4_IO_CONF   ((uint8_t) 0x16)
#define BMI088_REG_GYRO_INT_CTRL       ((uint8_t) 0x15)
#define BMI088_REG_GYR0_SOFTRESET      ((uint8_t) 0x14)
#define BMI088_REG_GYRO_LPM1           ((uint8_t) 0x11)
#define BMI088_REG_GYRO_BANDWIDTH      ((uint8_t) 0x10)
#define BMI088_REG_GYRO_RANGE          ((uint8_t) 0x0F)
#define BMI088_REG_GYRO_INT_STAT_1     ((uint8_t) 0x0A)
#define BMI088_REG_RATE_Z_MSB          ((uint8_t) 0x07)
#define BMI088_REG_RATE_Z_LSB          ((uint8_t) 0x06)
#define BMI088_REG_RATE_Y_MSB          ((uint8_t) 0x05)
#define BMI088_REG_RATE_Y_LSB          ((uint8_t) 0x04)
#define BMI088_REG_RATE_X_MSB          ((uint8_t) 0x03)
#define BMI088_REG_RATE_X_LSB          ((uint8_t) 0x02)
#define BMI088_REG_GYRO_CHIP_ID        ((uint8_t) 0x00)

#define BMI088_MASK_GYRO_SELF_TEST_TRIG_BIST    UBIT(0)
#define BMI088_MASK_GYRO_SELF_TEST_BIST_RDY     UBIT(1)
#define BMI088_MASK_GYRO_SELF_TEST_BIST_FAIL    UBIT(2)
#define BMI088_MASK_GYRO_SELF_TEST_RATE_OK      UBIT(4)

#define BMI088_GYRO_RANGE_2000         ((uint8_t) 0x00)
#define BMI088_GYRO_RANGE_1000         ((uint8_t) 0x01)
#define BMI088_GYRO_RANGE_500          ((uint8_t) 0x02)
#define BMI088_GYRO_RANGE_250          ((uint8_t) 0x03)
#define BMI088_GYRO_RANGE_125          ((uint8_t) 0x04)

#define BMI088_GYRO_BANDWIDTH_532       0x00
#define BMI088_GYRO_BANDWIDTH_230       0x01
#define BMI088_GYRO_BANDWIDTH_116       0x02
#define BMI088_GYRO_BANDWIDTH_47        0x03
#define BMI088_GYRO_BANDWIDTH_23        0x04
#define BMI088_GYRO_BANDWIDTH_12        0x05
#define BMI088_GYRO_BANDWIDTH_64        0x06
#define BMI088_GYRO_BANDWIDTH_32        0x07

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
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_532
#elif defined(BMI088_GYRO_ODR_2000_BW_230)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_230
#elif defined(BMI088_GYRO_ODR_2000_BW_116)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_116
#elif defined(BMI088_GYRO_ODR_2000_BW_47)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_47
#elif defined(BMI088_GYRO_ODR_2000_BW_23)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_23
#elif defined(BMI088_GYRO_ODR_2000_BW_64)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_64
#elif defined(BMI088_GYRO_ODR_2000_BW_32)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_32
#elif defined(BMI088_GYRO_ODR_2000_BW_12)
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_12
#else
#   define BMI088_GYRO_DEFAULT_BANDWIDTH    BMI088_GYRO_BANDWIDTH_230
#endif

enum BMI088_GYRO_ATTR {
	BMI088_GYRO_ATTR_BW = SENSOR_ATTR_PRIV_START,
};

struct bmi088_gyr_data {
	const struct device *bmi088_com_dev;
	const struct bmi088_transfer_function *tf;

	uint8_t gyro_odr;
	uint8_t gyro_range;

	union {
		struct {
			int16_t x_value;
			int16_t y_value;
			int16_t z_value;
		} __packed;
		int16_t array[3];
	} gyro_raw_values;

	union {
		struct {
			struct sensor_value x_value;
			struct sensor_value y_value;
			struct sensor_value z_value;
		} __packed;
		struct sensor_value array[3];
	} gyro_values;
};

struct bmi088_transfer_function {
	int (*read_register)(struct bmi088_gyr_data *data, uint16_t reg,
			     int count, void *val);
	int (*write_register)(struct bmi088_gyr_data *data, uint16_t reg,
			      int count, void *val);
};

struct bmi088_gyr_config {
	char *bmi088_com_dev_name;
};

int bmi088_gyr_spi_init(struct bmi088_gyr_data *data);

#endif
