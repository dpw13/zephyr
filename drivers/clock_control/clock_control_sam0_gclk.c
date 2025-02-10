/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2022 Google, LLC
 *
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <soc.h>

#define DT_DRV_COMPAT atmel_sam0_gclk

LOG_MODULE_REGISTER(clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct sam0_gclk_clock_config {
	uint32_t div_max;
	uint8_t div_bits;
};

struct sam0_gclk_config {
	struct sam0_gclk_clock_config gclk[GCLK_GEN_NUM];
};

struct sam0_gclk_clock_data {
	uint8_t enabled: 1;
	uint8_t standby_active: 1;
	uint8_t src: 5;

	uint16_t div;
};

struct sam0_gclk_data {
	struct sam0_gclk_clock_data gclk[GCLK_GEN_NUM];
};

static inline void sam0_gclk_wait_for_ready(void) {
	while (GCLK->STATUS.bit.SYNCBUSY) {
	}
}

static void sam0_gclk_apply(uint8_t gclk, const struct sam0_gclk_clock_config *cfg, struct sam0_gclk_clock_data *data)
{
	uint32_t flags = data->standby_active ? GCLK_GENCTRL_RUNSTDBY : 0;
	uint16_t div;

	/* Validation */
	if (data->div > cfg->div_max) {
		LOG_ERR("Divider value too large: %d > %d", data->div, cfg->div_max);
		return;
	}

	/* Disable if requested (or if an error occurred) */
	if (!data->enabled) {
		LOG_DBG("GCLK %d disabled", gclk);

		/* Write control with enable bit cleared */
		GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(gclk);

		sam0_gclk_wait_for_ready();

		return;
	}
	
	if (data->div < (1 << cfg->div_bits)) {
		div = data->div;
	} else {
		div = LOG2CEIL(data->div);
		flags |= GCLK_GENCTRL_DIVSEL;
	}

	LOG_DBG("GCLK %d enabled source %d divider %d flags %08x", gclk, data->src, div, flags);
	
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(gclk)
			 | GCLK_GENDIV_DIV(div);

	sam0_gclk_wait_for_ready();

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(gclk)
			  | GCLK_GENCTRL_GENEN
			  | ((uint32_t)data->src << GCLK_GENCTRL_SRC_Pos)
			  | flags;

	sam0_gclk_wait_for_ready();
}

static int sam0_gclk_inst_on(const struct device *dev,
			     clock_control_subsys_t sys)
{
	const struct sam0_gclk_config *cfg = dev->config;
	struct sam0_gclk_data *data = dev->data;
	struct sam0_gclk_clock_data *gclk = sys;
	int idx = gclk - &data->gclk[0];

	gclk->enabled = 1;
	sam0_gclk_apply(idx, &cfg->gclk[idx], gclk);

	return 0;
}

static int sam0_gclk_inst_off(const struct device *dev,
			      clock_control_subsys_t sys)
{
	const struct sam0_gclk_config *cfg = dev->config;
	struct sam0_gclk_data *data = dev->data;
	struct sam0_gclk_clock_data *gclk = sys;
	int idx = gclk - &data->gclk[0];

	gclk->enabled = 0;
	sam0_gclk_apply(idx, &cfg->gclk[idx], gclk);

	return 0;
}

static enum clock_control_status sam0_gclk_inst_get_status(const struct device *dev,
							   clock_control_subsys_t sys)
{
	const struct sam0_gclk_clock_data *data = sys;

	return data->enabled ? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static inline uint32_t sam0_gclk_inst_get_src_rate(uint8_t src)
{
	switch (src) {
		case GCLK_SOURCE_FDPLL:		/* Variable digital PLL */
		case GCLK_SOURCE_GCLKIN:
			LOG_ERR("Variable clock configurations not yet supported");
			return 0;
#ifdef CONFIG_SOC_ATMEL_SAMD_XOSC
		case GCLK_SOURCE_XOSC:		/* Crystal or external input */
			return CONFIG_SOC_ATMEL_SAMD_XOSC_FREQ_HZ;
#endif
		case GCLK_SOURCE_GCLKGEN1:
			return SOC_ATMEL_SAM0_GCLK1_FREQ_HZ;
		case GCLK_SOURCE_DFLL48M:	/* Digital Frequency Locked Loop */
			return SOC_ATMEL_SAM0_DFLL48M_MAX_FREQ_HZ;
		case GCLK_SOURCE_OSC8M:		/* Internal 8 MHz oscillator */
			return SOC_ATMEL_SAM0_OSC8M_FREQ_HZ;
		case GCLK_SOURCE_OSCULP32K:	/* Internal ultra low-power osc */
			return SOC_ATMEL_SAM0_OSCULP32K_FREQ_HZ;
		case GCLK_SOURCE_OSC32K:	/* High accuracy internal osc */
			return SOC_ATMEL_SAM0_OSC32K_FREQ_HZ;
		case GCLK_SOURCE_XOSC32K:	/* External 32kHz oscillator */
			return SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ;
		default:
			LOG_ERR("Invalid clock source %d", src);
			return 0;
	}
}

static int sam0_gclk_inst_get_rate(const struct device *dev,
				   clock_control_subsys_t sys,
				   uint32_t *rate)
{
	struct sam0_gclk_clock_data *gclk = sys;
	ARG_UNUSED(dev);

	*rate = sam0_gclk_inst_get_src_rate(gclk->src) / gclk->div;
	return 0;
}

static DEVICE_API(clock_control, sam0_gclk_inst_api) = {
	.on = sam0_gclk_inst_on,
	.off = sam0_gclk_inst_off,
	.get_status = sam0_gclk_inst_get_status,
	.get_rate = sam0_gclk_inst_get_rate
};

static int sam0_gclk_init(const struct device *dev)
{
	const struct sam0_gclk_config *cfg = dev->config;
	struct sam0_gclk_data *data = dev->data;

	for (int i=0; i < GCLK_GEN_NUM; i++) {
		sam0_gclk_apply(i, &cfg->gclk[i], &data->gclk[i]);
	}

	return 0;
}

#define SAM0_GCLK_INST_DATA_INIT(i) \
{								\
	.enabled = DT_NODE_HAS_STATUS_OKAY(i),			\
	.standby_active = DT_PROP_OR(i, standby_active, 0),	\
	.src = DT_PROP_OR(i, source, 0),			\
	.div = DT_PROP_OR(i, clock_div, 0),			\
}

#define SAM0_GCLK_INST_CONFIG_INIT(i) 				\
{								\
	.div_max = DT_PROP(i, div_max),				\
	.div_bits = DT_PROP(i, div_bits),			\
}

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
static const struct sam0_gclk_config sam0_gclk_cfg = {
	.gclk = {
		DT_FOREACH_CHILD_SEP(DT_NODELABEL(gclk), SAM0_GCLK_INST_CONFIG_INIT, (,))
	},
};

static struct sam0_gclk_data sam0_gclk_data = {
	.gclk = {
		DT_FOREACH_CHILD_SEP(DT_NODELABEL(gclk), SAM0_GCLK_INST_DATA_INIT, (,))
	},
};
#endif

DEVICE_DT_DEFINE(DT_NODELABEL(gclk), &sam0_gclk_init, NULL, &sam0_gclk_data, &sam0_gclk_cfg, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_OBJECTS, NULL);
