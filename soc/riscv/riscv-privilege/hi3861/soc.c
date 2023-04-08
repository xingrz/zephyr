/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/syscon.h>

#define GLB_CTL_SYS_CTL_ID_OFFSET             (0x0)
#define GLB_CTL_AON_SOFT_RST_W_OFFSET         (0x20)
#define GLB_CTL_SOFT_RST_WCPU_OFFSET          (0x24)
#define GLB_CTL_SOFT_GLB_RST_OFFSET           (0x28)
#define GLB_CTL_GLB_WDT_RST_SEL_OFFSET        (0x30)
#define GLB_CTL_WDT_RST_SEL_OFFSET            (0x34)
#define GLB_CTL_AON_CKEN_OFFSET               (0x40)
#define GLB_CTL_GLB_AON_32K_CLKEN_OFFSET      (0x50)
#define GLB_CTL_A32K_DIV_OFFSET               (0x70)
#define GLB_CTL_TCXO_DIV_OFFSET               (0x74)
#define GLB_CTL_AON_PERP_CLKSEL_W_OFFSET      (0x90)
#define GLB_CTL_RC_32K_TCXO_SEL_OFFSET        (0x94)
#define GLB_CTL_AON_32K_SEL_OFFSET            (0x98)
#define GLB_CTL_SYS_TICK_CFG_OFFSET           (0xC0)
#define GLB_CTL_SYS_TICK_VALUE_0_OFFSET       (0xD0)
#define GLB_CTL_SYS_TICK_VALUE_1_OFFSET       (0xD4)
#define GLB_CTL_SYS_TICK_VALUE_2_OFFSET       (0xD8)
#define GLB_CTL_SYS_TICK_VALUE_3_OFFSET       (0xDC)
#define GLB_CTL_CLKMUX_STS_OFFSET             (0x110)
#define GLB_CTL_DEBUG_CLKEN_OFFSET            (0x170)
#define GLB_CTL_SOFT_INT_EN_OFFSET            (0x280)
#define GLB_CTL_SOFT_INT_SET_OFFSET           (0x284)
#define GLB_CTL_SOFT_INT_CLR_OFFSET           (0x288)
#define GLB_CTL_SOFT_INT_STS_OFFSET           (0x28C)
#define GLB_CTL_INT_SEL_OFFSET                (0x290)
#define GLB_CTL_REFCLK_FEQ_STATUS_OFFSET      (0x358)
#define GLB_CTL_EXT_TSF_CTRL_OFFSET           (0x400)
#define GLB_CTL_CALI_32K_TCXO_CTL_OFFSET      (0x800)
#define GLB_CTL_CALI_32K_TCXO_CNT_L_OFFSET    (0x810)
#define GLB_CTL_CALI_32K_TCXO_CNT_H_OFFSET    (0x814)
#define GLB_CTL_CALI_32K_TCXO_RESULT_L_OFFSET (0x818)
#define GLB_CTL_CALI_32K_TCXO_RESULT_H_OFFSET (0x81C)
#define GLB_CTL_AON_ICM_PRIORITY_OFFSET       (0xF30)
#define GLB_CTL_MEM_CLK_FORCE_ON_OFFSET       (0xF50)
#define GLB_CTL_MARGIN_ADJ_OFFSET             (0xF54)
#define GLB_CTL_MARGIN_ADJ_AB_OFFSET          (0xF58)
#define GLB_CTL_DEFAULT_SLV_EN_OFFSET         (0xF84)
#define GLB_CTL_DEFAULT_SLV_HIT_STATUS_OFFSET (0xF88)
#define GLB_CTL_DEFAULT_SLV_HIT_CLR_OFFSET    (0xF8C)

static const struct device *glb_ctl = DEVICE_DT_GET(DT_NODELABEL(glb_ctl));

static int soc_hi3861_init(void)
{
	/*
	 * WDT was enabled by the romboot to check if the application is up and
	 * running. It is expected to be disabled by default in Zephyr, unless
	 * the WDT driver is explicitly enabled.
	 */
	syscon_write_reg(glb_ctl, GLB_CTL_GLB_WDT_RST_SEL_OFFSET, 0);

	return 0;
}

SYS_INIT(soc_hi3861_init, PRE_KERNEL_1, 0);
