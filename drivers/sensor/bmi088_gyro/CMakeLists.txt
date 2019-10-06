# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BMI088_GYRO bmi088_gyro.c)
zephyr_library_sources_ifdef(CONFIG_BMI088_GYRO bmi088_gyro_spi.c)