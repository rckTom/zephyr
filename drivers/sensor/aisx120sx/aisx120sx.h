/*
 * Copyright (c) 2019 Thomas Schmid <tom@lfence.de>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_AISX120SX_H__
#define __SENSOR_AISX120SX_H__

#include <zephyr.h>
#include <stdbool.h>

/* Register address definitions */
#define AISX120SX_ADR_REG_CTRL_0                0x00
#define AISX120SX_ADR_REG_CTRL_1                0x01
#define AISX120SX_ADR_REG_CONFIG                0x02
#define AISX120SX_ADR_REG_STATUS_0              0x03
#define AISX120SX_ADR_REG_STATUS_1              0x04
#define AISX120SX_ADR_REG_STATUS_2              0x05
#define AISX120SX_ADR_REG_CHID_REVID            0x06
#define AISX120SX_ADR_REG_ACC_CHX_LOW           0x07
#define AISX120SX_ADR_REG_ACC_CHX_HIGH          0x08
#define AISX120SX_ADR_REG_ACC_CHY_LOW           0x09
#define AISX120SX_ADR_REG_ACC_CHY_HIGH          0x0A
#define AISX120SX_ADR_REG_OSC_COUNTER           0x0B
#define AISX120SX_ADR_REG_ID_SENSOR_TYPE        0x0C
#define AISX120SX_ADR_REG_ID_VEH_MANUF          0x0D
#define AISX120SX_ADR_REG_ID_SENSOR_MANUF       0x0E
#define AISX120SX_ADR_REG_ID_LOT_0              0x0F
#define AISX120SX_ADR_REG_ID_LOT_1              0x10
#define AISX120SX_ADR_REG_ID_LOT_2              0x11
#define AISX120SX_ADR_REG_ID_LOT_3              0x12
#define AISX120SX_ADR_REG_ID_WAFER              0x13
#define AISX120SX_ADR_REG_ID_COOR_X             0x14
#define AISX120SX_ADR_REG_ID_COOR_Y             0x15
#define AISX120SX_ADR_REG_RESET                 0x16
#define AISX120SX_ADR_OFF_CHX_HIGH              0x17
#define AISX120SX_ADR_OFF_CHX_LOW               0x18
#define AISX120SX_ADR_OFF_CHY_HIGH              0x19
#define AISX120SX_ADR_OFF_CHY_LOW               0x1A

/* Register mask definitions */
#define AISX120SX_MSK_END_OF_INIT               GENMASK(0, 0)
#define AISX120SX_MSK_SELF_TEST_CMD             GENMASK(2, 0)
#define AISX120SX_MSK_FIR_BW_SEL_CHY            GENMASK(7, 6)
#define AISX120SX_MSK_FIR_BW_SEL_CHX            GENMASK(5, 4)
#define AISX120SX_MSK_DIS_OFF_MON_CHY           GENMASK(3, 3)
#define AISX120SX_MSK_DIS_OFF_MON_CHX           GENMASK(2, 2)
#define AISX120SX_MSK_DIS_OFF_CANC_CHY          GENMASK(1, 1)
#define AISX120SX_MSK_DIS_OFF_CANC_CHX          GENMASK(0, 0)
#define AISX120SX_MSK_SATUS                     GENMASK(7, 6)
#define AISX120SX_MSK_TESTMODE_ENABLED          GENMASK(5, 5)
#define AISX120SX_MSK_REG_CTRL_0_WR_ERR         GENMASK(4, 4)
#define AISX120SX_MSK_LOSS_CAP                  GENMASK(2, 2)
#define AISX120SX_MSK_END_OF_PWRUP              GENMASK(1, 1)
#define AISX120SX_MSK_RST_ACTIVE                GENMASK(0, 0)
#define AISX120SX_MSK_SPI_ERR                   GENMASK(7, 7)
#define AISX120SX_MSK_EEPROM_ERR                GENMASK(6, 6)
#define AISX120SX_MSK_OFF_CANC_CHY_ERR          GENMASK(3, 3)
#define AISX120SX_MSK_OFF_CANC_CHX_ERR          GENMASK(2, 2)
#define AISX120SX_MSK_REG_CONFIG_WR_ERR         GENMASK(1, 1)
#define AISX120SX_MSK_REG_CTRL_1_WR_ERR         GENMASK(0, 0)
#define AISX120SX_MSK_A2D_SAT_CHY               GENMASK(7, 7)
#define AISX120SX_MSK_A2D_SAT_CHX               GENMASK(6, 6)
#define AISX120SX_MSK_CHARGE_PUMP_ERR           GENMASK(4, 4)
#define AISX120SX_MSK_VREG_LOW_VOLT_DET         GENMASK(3, 3)
#define AISX120SX_MSK_VREG_HIGH_VOLT_DET        GENMASK(2, 2)
#define AISX120SX_MSK_VDD_LOW_VOLT_DET          GENMASK(1, 1)
#define AISX120SX_MSK_VDD_HIGH_VOLT_DET         GENMASK(0, 0)
#define AISX120SX_MSK_CHY_ACTIVE                GENMASK(5, 5)
#define AISX120SX_MSK_CHX_ACTIVE                GENMASK(4, 4)
#define AISX120SX_MSK_REVID                     GENMASK(2, 0)
#define AISX120SX_MSK_REG_ACC_CHX_LOW           GENMASK(7, 0)
#define AISX120SX_MSK_ACCEL_DATA_X_LATCH        GENMASK(7, 7)
#define AISX120SX_MSK_REG_ACC_CHX_HIGH          GENMASk(5, 0)
#define AISX120SX_MSK_REG_ACC_CHY_LOW           GENMASK(7, 0)
#define AISX120SX_MSK_ACCEL_DATA_Y_LATCH        GENMASK(7, 7)
#define AISX120SX_MSK_REG_ACC_CHY_HIGH          GENMASK(5, 0)
#define AISX120SX_MSK_OSC_COUNTER               GENMASK(7, 0)
#define AISX120SX_MSK_ID_SENSOR_TYPE            GENMASK(7, 0)
#define AISX120SX_MSK_ID_VEH_MANUF              GENMASK(3, 0)
#define AISX120SX_MSK_ID_SENSOR_MANUF           GENMASK(7, 0)
#define AISX120SX_MSK_ID_LOT_0                  GENMASK(7, 0)
#define AISX120SX_MSK_ID_LOT_1                  GENMASK(7, 0)
#define AISX120SX_MSK_ID_LOT_2                  GENMASK(7, 0)
#define AISX120SX_MSK_ID_LOT_3                  GENMASK(7, 0)
#define AISX120SX_MSK_ID_WAFER                  GENMASK(4, 0)
#define AISX120SX_MSK_ID_COOR_X                 GENMASK(7, 0)
#define AISX120SX_MSK_ID_COOR_Y                 GENMASK(7, 0)
#define AISX120SX_MSK_SOFT_RST                  GENMASK(1, 0)
#define AISX120SX_MSK_OFF_CHX_HIGH              GENMASK(7, 0)
#define AISX120SX_MSK_OFF_DATA_X_LATCH          GENMASK(7, 7)
#define AISX120SX_MSK_OFF_CHX_LOW               GENMASK(2, 0)
#define AISX120SX_MSK_OFF_CHY_HIGH              GENMASK(7, 0)
#define AISX120SX_MSK_OFF_DATA_Y_LATCH          GENMASK(7, 7)
#define AISX120SX_MSK_OFF_CHY_LOW               GENMASK(2, 0)

#define AISX120SX_BW_400Hz                      (0x00 << 6)
#define AISX120SX_BW_800Hz                      (0x01 << 6)
#define AISX120SX_BW_1600Hz                     (0x02 << 6)

#define AISX120SX_STATUS_INIT                   (0x00 << 6)
#define AISX120SX_STATUS_NORMAL                 (0x01 << 6)
#define AISX120SX_STATUS_TEST                   (0x02 << 6)
#define AISX120SX_STATUS_INIT_NORMAL_ERR        (0x03 << 6)

#define AISX120SX_RESET_SEQ_1                   0x2
#define AISX120SX_RESET_SEQ_2                   0x1
#define AISX120SX_RESET_SEQ_3                   0x2

#define AISX120SX_SENSOR_DUAL_CHANNEL           0x2A
#define AISX120SX_SENSOR_SINGLE_CHANNEL         0x1A

#if defined(CONFIG_AISX120SX_BW_CHANNEL_X_400)
	#define AISX120SX_BW_X_DEFAULT          400
#elif defined(CONFIG_AISX120SX_BW_CHANNEL_X_800)
	#define AISX120SX_BW_X_DEFAULT          800
#elif defined(CONFIG_AISX120SX_BW_CHANNEL_X_1600)
	#define AISX120SX_BW_X_DEFAULT          1600
#else
	#define AISX120SX_BW_X_DEFAULT          400
#endif

#if defined(CONFIG_AISX120SX_BW_CHANNEL_Y_400)
	#define AISX120SX_BW_Y_DEFAULT          400
#elif defined(CONFIG_AISX120SX_BW_CHANNEL_Y_800)
	#define AISX120SX_BW_Y_DEFAULT          800
#elif defined(CONFIG_AISX120SX_BW_CHANNEL_Y_1600)
	#define AISX120SX_BW_Y_DEFAULT          1600
#else
	#define AISX120SX_BW_Y_DEFAULT          400
#endif

union aisx120sx_reg_status_0 {
	struct {
		uint8_t RST_ACTIVE : 1;
		uint8_t END_OF_PWRUP : 1;
		uint8_t LOSS_CAP : 1;
		uint8_t RESERVED : 1;
		uint8_t REG_CTRL_0_WR_ERR : 1;
		uint8_t TESTMODE_ENABLED : 1;
		uint8_t STATUS : 2;
	} __packed fields;
	uint8_t reg;
};

union aisx120sx_reg_status_1 {
	struct {
		uint8_t REG_CTRL_1_WR_ERR : 1;
		uint8_t REG_CONFIG_WR_ERR : 1;
		uint8_t OFF_CANC_CHX_ERR : 1;
		uint8_t OFF_CANC_CHY_ERR : 1;
		uint8_t NOT_USED : 2;
		uint8_t EEPROM_ERR : 1;
		uint8_t SPI_ERR : 1;
	} __packed fields;
	uint8_t reg;
};


union aisx120sx_reg_status_2 {
	struct {
		uint8_t VDD_HIGH_VOLT_DET : 1;
		uint8_t VDD_LOW_VOLT_DET : 1;
		uint8_t VREG_HIGH_VOLT_DET : 1;
		uint8_t VREG_LOW_VOLT_DET : 1;
		uint8_t CHARGE_PUMP_ERR : 1;
		uint8_t NOT_USED : 1;
		uint8_t A2D_SAT_CHX : 1;
		uint8_t A2D_SAT_CHY : 1;
	} __packed fields;
	uint8_t reg;
};

union aisx120sx_reg_config {
	struct {
		uint8_t DIS_OFF_CANC_CHX : 1;
		uint8_t DIS_OFF_CANC_CHY : 1;
		uint8_t DIS_OFF_MON_CHX : 1;
		uint8_t DIS_OFF_MON_CHY : 1;
		uint8_t FIR_BW_SEL_CHX : 2;
		uint8_t FIR_BW_SEL_CHY : 2;
	} __packed fields;
	uint8_t reg;
};

union aisx120sx_reg_ctrl_1 {
	struct {
		uint8_t SELF_TEST_CMD : 2;
		uint8_t RESERVED : 6;
	} __packed fields;
	uint8_t reg;
};

enum AISX120SX_SENSOR_ATTR {
	AISX120SX_ATTR_BW		= SENSOR_ATTR_PRIV_START,
	AISX120SX_ATTR_OFFSET_CANC	= SENSOR_ATTR_PRIV_START + 1,
	AISX120SX_ATTR_OFFSET_MONIT	= SENSOR_ATTR_PRIV_START + 2,
	AISX120SX_ATTR_RESET		= SENSOR_ATTR_PRIV_START + 3,
	AISX120SX_ATTR_END_OF_INIT	= SENSOR_ATTR_PRIV_START + 4,
};

struct aisx120sx_data {
	struct device *spi_dev;
	bool is_dual_channel;
	int16_t acc_x, acc_y;
};

struct aisx120sx_config {
	const char *aisx120sx_dev_name;
};

#endif /* __SENSOR_AISX120SX_H__ */
