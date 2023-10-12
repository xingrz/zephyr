/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_i2c

#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bcm2711.h>
#if defined(CONFIG_PINCTRL)
#include <zephyr/drivers/pinctrl.h>
#endif /* CONFIG_PINCTRL */
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_bcm2711, CONFIG_I2C_LOG_LEVEL);

#define I2C_C(base)    (base + 0x00)
#define I2C_S(base)    (base + 0x04)
#define I2C_DLEN(base) (base + 0x08)
#define I2C_A(base)    (base + 0x0C)
#define I2C_FIFO(base) (base + 0x10)
#define I2C_DIV(base)  (base + 0x14)
#define I2C_DEL(base)  (base + 0x18)
#define I2C_CLKT(base) (base + 0x1C)

/* C Register */
#define I2C_C_READ  BIT(0)
#define I2C_C_CLEAR BIT(4)
#define I2C_C_ST    BIT(7)
#define I2C_C_INTD  BIT(8)
#define I2C_C_INTT  BIT(9)
#define I2C_C_INTR  BIT(10)
#define I2C_C_I2CEN BIT(15)

/* S Register */
#define I2C_S_TA   BIT(0)
#define I2C_S_DONE BIT(1)
#define I2C_S_TXW  BIT(2)
#define I2C_S_RXR  BIT(3)
#define I2C_S_TXD  BIT(4)
#define I2C_S_RXD  BIT(5)
#define I2C_S_TXE  BIT(6)
#define I2C_S_RXF  BIT(7)
#define I2C_S_ERR  BIT(8)
#define I2C_S_CLKT BIT(9)

#define DEV_CFG(dev)  ((const struct i2c_bcm2711_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct i2c_bcm2711_data *const)(dev)->data)

struct i2c_bcm2711_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);
#if defined(CONFIG_PINCTRL)
	const struct pinctrl_dev_config *pcfg;
#endif /* CONFIG_PINCTRL */
	const struct bcm2711_clock_config ccfg;

	void (*irq_config_func)(void);

	struct k_sem sem_transfer;
};

struct i2c_bcm2711_data {
	DEVICE_MMIO_NAMED_RAM(reg_base);
	mem_addr_t base;

	bool addr10;
};

static int i2c_bcm2711_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_bcm2711_config *cfg = DEV_CFG(dev);
	struct i2c_bcm2711_data *data = DEV_DATA(dev);
	uint32_t regval;
	uint32_t clk_freq;
	uint32_t cdiv;
	int ret;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		LOG_ERR("Only controller mode is supported");
		return -ENOTSUP;
	}

	data->addr10 = !!(dev_config & I2C_ADDR_10_BITS);

	ret = clock_control_get_rate(BCM2711_CLOCK_CONTROLLER, (clock_control_subsys_t *)&cfg->ccfg,
				     &clk_freq);
	if (ret) {
		LOG_ERR("Failed to get clock rate (err %d)", ret);
		return ret;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		cdiv = clk_freq / KHZ(100);
		break;
	case I2C_SPEED_FAST:
		cdiv = clk_freq / KHZ(400);
		break;
	default:
		LOG_ERR("Unsupported speed");
		return -ENOTSUP;
	}

	if (cdiv > BIT_MASK(16)) {
		LOG_ERR("Given clock source is too fast");
		return -EINVAL;
	}

	/* Reset controller */
	regval = sys_read32(I2C_C(data->base));
	regval |= I2C_C_CLEAR;
	regval &= ~(I2C_C_INTD | I2C_C_INTT | I2C_C_INTR | I2C_C_I2CEN);
	sys_write32(regval, I2C_C(data->base));

	/* Set clock divider */
	sys_write32(cdiv, I2C_DIV(data->base));

	return 0;
}

static int i2c_bcm2711_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr)
{
	const struct i2c_bcm2711_config *cfg = DEV_CFG(dev);
	struct i2c_bcm2711_data *data = DEV_DATA(dev);

	if (!num_msgs) {
		return 0;
	}

	for (uint8_t i = 0; i < num_msgs; i++) {
		sys_write32(addr, I2C_A(data->base));
		sys_write32(msgs[i].len, I2C_DLEN(data->base));
		sys_write32(I2C_C_ST | I2C_C_INTT | I2C_C_I2CEN, I2C_C(data->base));
	}

	return 0;
}

static void i2c_bcm2711_isr(const struct device *dev)
{
}

static int i2c_bcm2711_init(const struct device *dev)
{
	const struct i2c_bcm2711_config *cfg = DEV_CFG(dev);
	struct i2c_bcm2711_data *data = DEV_DATA(dev);

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE);
	data->base = DEVICE_MMIO_NAMED_GET(dev, reg_base);

#if defined(CONFIG_PINCTRL)
	pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
#endif /* CONFIG_PINCTRL */

	clock_control_on(BCM2711_CLOCK_CONTROLLER, (clock_control_subsys_t *)&cfg->ccfg);

	cfg->irq_config_func();

	k_sem_init(&cfg->sem_transfer, 0, 1);

	LOG_DBG("Inited I2C device at 0x%80lx", data->base);

	return 0;
}

static const struct i2c_driver_api i2c_bcm2711_api = {
	.configure = i2c_bcm2711_configure,
	.transfer = i2c_bcm2711_transfer,
};

#define I2C_BCM2711_INST(n)                                                                        \
	static struct i2c_bcm2711_data i2c_bcm2711_data_##n;                                       \
                                                                                                   \
	COND_CODE_1(IS_ENABLED(CONFIG_PINCTRL), (PINCTRL_DT_INST_DEFINE(n);), ())                  \
                                                                                                   \
	static void i2c_bcm2711_config_func_##n(void)                                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2c_bcm2711_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct i2c_bcm2711_config i2c_bcm2711_config_##n = {                          \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                              \
		COND_CODE_1(IS_ENABLED(CONFIG_PINCTRL),                                            \
			    (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), ), ())                     \
			.ccfg = BCM2711_DT_INST_CLOCK_CFG(n),                                      \
		.irq_config_func = i2c_bcm2711_config_func_##n,                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &i2c_bcm2711_init, NULL, &i2c_bcm2711_data_##n,                   \
			      &i2c_bcm2711_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,      \
			      &i2c_bcm2711_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_BCM2711_INST)
