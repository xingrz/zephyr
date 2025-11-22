/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_DRIVERS_SENSOR_QMI8658_H__
#define __ZEPHYR_DRIVERS_SENSOR_QMI8658_H__

#define DT_DRV_COMPAT qst_qmi8658

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define QMI8658_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define QMI8658_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#ifdef QMI8658_BUS_SPI
#error "SPI bus is not supported for QMI8658 driver yet"
#endif /* QMI8658_BUS_SPI */

union qmi8658_bus {
#if QMI8658_BUS_I2C
	struct i2c_dt_spec i2c;
#endif /* QMI8658_BUS_I2C */
};

#endif /* __ZEPHYR_DRIVERS_SENSOR_QMI8658_H__ */
