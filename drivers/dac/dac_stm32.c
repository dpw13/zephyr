/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_dac

#include <errno.h>

#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <soc.h>
#include <stm32_ll_dac.h>

#ifdef CONFIG_DAC_STM32_DMA
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/toolchain.h>
#include <stm32_ll_dma.h>
#endif

#define LOG_LEVEL CONFIG_DAC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dac_stm32);

#include <zephyr/drivers/clock_control/stm32_clock_control.h>

/* some low-end MCUs have DAC with only one channel */
#ifdef LL_DAC_CHANNEL_2
#define STM32_CHANNEL_COUNT		2
#else
#define STM32_CHANNEL_COUNT		1
#endif

/* first channel always named 1 */
#define STM32_FIRST_CHANNEL		1

#define CHAN(n)		LL_DAC_CHANNEL_##n
static const uint32_t table_channels[] = {
	CHAN(1),
#ifdef LL_DAC_CHANNEL_2
	CHAN(2),
#endif
};

/* Read-only driver configuration */
struct dac_stm32_cfg {
	/* DAC instance. */
	DAC_TypeDef *base;
	/* Clock configuration. */
	struct stm32_pclken pclken;
	/* pinctrl configurations. */
	const struct pinctrl_dev_config *pcfg;
};

#ifdef CONFIG_DAC_STM32_DMA
struct stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	bool continuous;
	dac_convert_callback callback;
};
#endif /* CONFIG_DAC_STM32_DMA */

/* Runtime driver data */
struct dac_stm32_data {
	uint8_t channel_count;
	uint8_t resolution;

#ifdef CONFIG_DAC_STM32_DMA
	volatile int dma_error;
	struct stream dma;
#endif
};

#ifdef CONFIG_DAC_STM32_DMA
static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	/* user_data directly holds the dac device */
	struct dac_stm32_data *data = user_data;

	LOG_DBG("callback");

	if (channel == data->dma.channel) {
		if (status < 0) {
			LOG_ERR("DMA conversion complete, but DMA reported error %d", status);
			data->dma_error = status;
			dma_stop(data->dma.dma_dev, data->dma.channel);
			return;
		}

		/* First half of buffer if we get DMA_STATUS_BLOCK, second half if complete. */
		uint16_t sampling_index = (status == DMA_STATUS_COMPLETE) ? 1 : 0;

		if (data->dma.callback != NULL && data->dma.callback(dev, sampling_index) == 0) {
			LOG_DBG("DMA callback requesting stop");
			/* Stop conversion */
			dma_stop(data->dma.dma_dev, data->dma.channel);
			return;
		}

		/* Nothing is required to change if we want to continue. The DMA
		 * is set up to auto-regenerate.
		 */
	}
}

static int dac_stm32_dma_start(const struct device *dev, const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_stm32_cfg *config = dev->config;
	DAC_TypeDef *dac = (DAC_TypeDef *)config->base;
	struct dac_stm32_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *dma = &data->dma;

	uint32_t reg_id;
	if (data->resolution == 8) {
		reg_id = LL_DAC_DMA_REG_DATA_8BITS_RIGHT_ALIGNED;
	} else {
		/* 12-bit resolution */
		/* TODO: Support left- or right-justified data */
		reg_id = LL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED;
	}

	uint32_t hal_channel_id;
	if (channel_cfg->channel_id == STM32_FIRST_CHANNEL) {
		/* I don't think using `dma_slot` to store the peripheral request
		 * ID is the right way to go, but that's the way the dma_stm32
		 * driver is written.
		 */
		dma->dma_cfg.dma_slot = LL_DMAMUX1_REQ_DAC1_CH1;
		hal_channel_id = LL_DAC_CHANNEL_1;
	} else {
		dma->dma_cfg.dma_slot = LL_DMAMUX1_REQ_DAC1_CH2;
		hal_channel_id = LL_DAC_CHANNEL_2;
	}

	blk_cfg = &dma->dma_blk_cfg;

	/* prepare the block */
	blk_cfg->block_size = channel_cfg->buffer_size;

	/* Source and destination. Buffers must be contiguous. */
	blk_cfg->source_address = (uintptr_t)channel_cfg->buffer_base;
	if (channel_cfg->buffer_size == 0) {
		/* If this is a register transfer, don't increment the address */
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}
	blk_cfg->source_reload_en = 1;


	blk_cfg->dest_address = (uint32_t)LL_DAC_DMA_GetRegAddr(dac, hal_channel_id, reg_id);
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->dest_reload_en = 1;

	/* Manually set the FIFO threshold to 1/4 because the
	* dmamux DTS entry does not contain fifo threshold
	*/
	blk_cfg->fifo_mode_control = 0;
	blk_cfg->next_block = NULL;

	LOG_DBG("DMA:%d configured with request id %d dest addr %p", channel_cfg->channel_id, dma->dma_cfg.dma_slot, (void *)blk_cfg->dest_address);

	dma->dma_cfg.block_count = 1;
	/* direction is given by the DT */
	dma->dma_cfg.complete_callback_en = 0; /* Callback only at TC */

	dma->dma_cfg.head_block = blk_cfg;
	dma->dma_cfg.user_data = data;
	dma->dma_cfg.cyclic = dma->continuous;
	dma->callback = channel_cfg->callback;

	ret = dma_config(data->dma.dma_dev, data->dma.channel, &dma->dma_cfg);
	if (ret != 0) {
		LOG_ERR("Problem setting up DMA: %d", ret);
		return ret;
	}

	/* Enable DMA */
	LL_DAC_EnableDMAReq(dac, hal_channel_id);

	data->dma_error = 0;
	ret = dma_start(data->dma.dma_dev, data->dma.channel);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

	LOG_DBG("DMA started: %d B to %p", blk_cfg->block_size, (void *)blk_cfg->dest_address);

	return ret;
}
#endif

static int dac_stm32_write_value(const struct device *dev,
					uint8_t channel, uint32_t value)
{
	struct dac_stm32_data *data = dev->data;
	const struct dac_stm32_cfg *cfg = dev->config;

	if (channel - STM32_FIRST_CHANNEL >= data->channel_count ||
					channel < STM32_FIRST_CHANNEL) {
		LOG_ERR("Channel %d is not valid", channel);
		return -EINVAL;
	}

	if (value >= BIT(data->resolution)) {
		LOG_ERR("Value %d is out of range", value);
		return -EINVAL;
	}

	if (data->resolution == 8) {
		LL_DAC_ConvertData8RightAligned(cfg->base,
			table_channels[channel - STM32_FIRST_CHANNEL], value);
	} else if (data->resolution == 12) {
		LL_DAC_ConvertData12RightAligned(cfg->base,
			table_channels[channel - STM32_FIRST_CHANNEL], value);
	}

	return 0;
}

static int dac_stm32_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	struct dac_stm32_data *data = dev->data;
	const struct dac_stm32_cfg *cfg = dev->config;
	uint32_t cfg_setting, channel;
	int ret;

	if ((channel_cfg->channel_id - STM32_FIRST_CHANNEL >=
			data->channel_count) ||
			(channel_cfg->channel_id < STM32_FIRST_CHANNEL)) {
		LOG_ERR("Channel %d is not valid", channel_cfg->channel_id);
		return -EINVAL;
	}

	if ((channel_cfg->resolution == 8) ||
			(channel_cfg->resolution == 12)) {
		data->resolution = channel_cfg->resolution;
	} else {
		LOG_ERR("Resolution not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->buffer_base) {
		ret = dac_stm32_dma_start(dev, channel_cfg);
		if (ret < 0) {
			LOG_ERR("Failed to start DAC DMA: %d", ret);
		} else {
			LOG_DBG("DMA started");
		}
	}

	channel = table_channels[channel_cfg->channel_id - STM32_FIRST_CHANNEL];

	if (channel_cfg->buffered) {
		cfg_setting = LL_DAC_OUTPUT_BUFFER_ENABLE;
	} else {
		cfg_setting = LL_DAC_OUTPUT_BUFFER_DISABLE;
	}

	LL_DAC_SetOutputBuffer(cfg->base, channel, cfg_setting);

#if defined(LL_DAC_OUTPUT_CONNECT_INTERNAL)
	/* If the DAC supports internal connections set it based on configuration */
	if (channel_cfg->internal) {
		cfg_setting = LL_DAC_OUTPUT_CONNECT_INTERNAL;
	} else {
		cfg_setting = LL_DAC_OUTPUT_CONNECT_GPIO;
	}

	LL_DAC_SetOutputConnection(cfg->base, channel, cfg_setting);
#else
	if (channel_cfg->internal) {
		LOG_ERR("Internal connections not supported");
		return -ENOTSUP;
	}
#endif /* LL_DAC_OUTPUT_CONNECT_INTERNAL */

	if (channel_cfg->trig_src >= 0) {
		LL_DAC_SetTriggerSource(cfg->base, channel, channel_cfg->trig_src);
	}
	LL_DAC_Enable(cfg->base, channel);

	if (channel_cfg->trig_src >= 0) {
		LL_DAC_EnableTrigger(cfg->base, channel);
	}

	LOG_DBG("Channel setup succeeded!");

	return 0;
}

static int dac_stm32_init(const struct device *dev)
{
	const struct dac_stm32_cfg *cfg = dev->config;
	int err;

	/* enable clock for subsystem */
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_on(clk,
			     (clock_control_subsys_t) &cfg->pclken) != 0) {
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if ((err < 0) && (err != -ENOENT)) {
		LOG_ERR("DAC pinctrl setup failed (%d)", err);
		return err;
	}

	return 0;
}

static DEVICE_API(dac, api_stm32_driver_api) = {
	.channel_setup = dac_stm32_channel_setup,
	.write_value = dac_stm32_write_value
};

#if defined(CONFIG_DAC_STM32_DMA)

#define DAC_DMA_CHANNEL_INIT(index, src_dev, dest_dev)                                             \
	.dma = {                                                                                   \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_IDX(index, 0)),                      \
		.channel = DT_INST_DMAS_CELL_BY_IDX(index, 0, channel),                            \
		.dma_cfg =                                                                         \
			{                                                                          \
				.dma_slot = STM32_DMA_SLOT_BY_IDX(index, 0, slot),                 \
				.channel_direction = STM32_DMA_CONFIG_DIRECTION(                   \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(        \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(         \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.source_burst_length = 1, /* SINGLE transfer */                    \
				.dest_burst_length = 8,   /* SINGLE transfer */                    \
				.channel_priority = STM32_DMA_CONFIG_PRIORITY(                     \
					STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                \
				.dma_callback = dma_callback,                                      \
				.complete_callback_en = 1, /* Callback at each block */            \
				.block_count = 1,                                                  \
			},                                                                         \
		.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(                       \
			STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                                \
		.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(                      \
			STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),                                \
	}

#else /* CONFIG_DAC_STM32_DMA */

#define DAC_DMA_CHANNEL_INIT(index, src_dev, dest_dev)

#endif

#define DAC_DMA_CHANNEL(id, src, dest)                                                             \
	COND_CODE_1(DT_INST_DMAS_HAS_IDX(id, 0),					\
			(DAC_DMA_CHANNEL_INIT(id, src, dest)),				\
			(/* Required for other dac instances without dma */))

#define STM32_DAC_INIT(index)                                                                      \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct dac_stm32_cfg dac_stm32_cfg_##index = {                                \
		.base = (DAC_TypeDef *)DT_INST_REG_ADDR(index),                                    \
		.pclken =                                                                          \
			{                                                                          \
				.enr = DT_INST_CLOCKS_CELL(index, bits),                           \
				.bus = DT_INST_CLOCKS_CELL(index, bus),                            \
			},                                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
	};                                                                                         \
                                                                                                   \
	static struct dac_stm32_data dac_stm32_data_##index = {                                    \
		.channel_count = STM32_CHANNEL_COUNT, DAC_DMA_CHANNEL(index, MEMORY, PERIPHERAL)}; \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &dac_stm32_init, NULL, &dac_stm32_data_##index,               \
			      &dac_stm32_cfg_##index, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,       \
			      &api_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(STM32_DAC_INIT)
