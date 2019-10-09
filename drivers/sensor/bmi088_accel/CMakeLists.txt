# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BMI088_ACCEL bmi088_acc.c)
zephyr_library_sources_ifdef(CONFIG_BMI088_ACCEL bmi088_acc_spi.c)