/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * Copyright (c) 2022-2023 Jamie McCrae
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * Copyright (c) 2024 Dane Wagner <dane.wagner@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT solomon_ssd1803a

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/auxdisplay.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_AUXDISPLAY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(disp_ssd1803a);

#define SSD1803A_CHAR_BUF_SIZE 80

/* Defines for the SSD1803A_RE0_INST_DISPLAY_ON_OFF */
#define SSD1803A_DO_BLINKING_ON (1 << 0)
#define SSD1803A_DO_CURSOR_ON   (1 << 1)
#define SSD1803A_DO_DISPLAY_ON  (1 << 2)

/* Defines for the SSD1803A_RE0_INST_FUNCTION_SET */
#define SSD1803A_FS_RE (1 << 1)

#define SSD1803A_FS_RE0_IS         (1 << 0)
#define SSD1803A_FS_RE0_DBL_HEIGHT (1 << 2)

#define SSD1803A_FS_RE1_REV      (1 << 0)
#define SSD1803A_FS_RE1_BLINK_EN (1 << 2)

#define SSD1803A_FS_ROWS_1_3  (0 << 3)
#define SSD1803A_FS_ROWS_2_4  (1 << 3)
#define SSD1803A_FS_4BIT_MODE (0 << 4)
#define SSD1803A_FS_8BIT_MODE (1 << 4)

#define SSD1803A_BRIGHTNESS_MIN 1
#define SSD1803A_BRIGHTNESS_MAX BIT_MASK(7)

/* Defines for the SSD1803A_RE0_INST_DDRAM_ADDRESS_SET */
#define SSD1803A_DA_BASE_ROW_1 (0x00)
#define SSD1803A_DA_BASE_ROW_2 (0x40)

/* Display Commands */
#define SSD1803A_INST_FUNCTION_SET BIT(5)

#define SSD1803A_RE0_INST_CLEAR_DISPLAY           BIT(0)
#define SSD1803A_RE0_INST_CURSOR_HOME             BIT(1)
#define SSD1803A_RE0_INST_ENTRY_MODE_SET          BIT(2)
#define SSD1803A_RE0_INST_DISPLAY_ON_OFF          BIT(3)
#define SSD1803A_RE0_INST_CURSOR_OR_DISPLAY_SHIFT BIT(4)
#define SSD1803A_RE0_INST_DDRAM_ADDRESS_SET       BIT(7)

#define SSD1803A_RE1_INST_POWER_DOWN (0x02)
#define SSD1803A_RE1_INST_BIDIR      (0x04)

#define SSD1803A_RE1_INST_EXT_FUNC_SET (0x08)
#define SSD1803A_EXT_FUNC_WIDE_FONT    BIT(2)
#define SSD1803A_EXT_FUNC_INV_CURSOR   BIT(1)
#define SSD1803A_EXT_FUNC_MORE_ROWS    BIT(0)

#define SSD1803A_RE1_INST_DBL_H_BIAS (0x10)
#define SSD1803A_RE1_INST_ROM_SEL    (0x72)

#define SSD1803A_IS1_RE0_INST_CONTRAST_H         (0x50)
#define SSD1803A_IS1_RE0_INST_FOLLOWER           (0x60)
#define SSD1803A_IS1_RE0_INST_CONTRAST_L         (0x70)
#define SSD1803A_IS1_RE0_INST_SEGRAM_ADDRESS_SET (0x40)

#define SSD1803A_CONTRAST_H_BOOST_EN BIT(2)
#define SSD1803A_CONTRAST_H_ICON_EN  BIT(3)

#define SSD1803A_IS0_RE0_INST_CGRAM_ADDRESS_SET BIT(6)

/* Start Byte */
#define SSD1803A_SB_RS_INST   (0 << 6)
#define SSD1803A_SB_RS_DATA   (1 << 6)
#define SSD1803A_SB_RW_WRITE  (0 << 5)
#define SSD1803A_SB_RW_READ   (1 << 5)
#define SSD1803A_SB_SYNC_BITS (BIT_MASK(5))

struct auxdisplay_ssd1803a_data {
	/* Enables */
	bool power;
	bool cursor;
	bool blinking;
	bool user_blink;
	bool invert;
	bool invert_cursor;
	bool dbl_height;
	bool icon;  /* SEGRAM */
	bool boost; /* Booster and regulator */
	bool wide_font;
	bool bias_divider;

	/* Register access state */
	bool re; /* Extended registers enabled */
	bool is; /* "Special registers" enabled */

	uint8_t osc_freq;
	uint8_t bias;
	uint8_t dbl_height_fmt;
	uint8_t cgrom_idx;
	uint8_t amp_ratio;

	uint8_t brightness;
	uint16_t cursor_x;
	uint16_t cursor_y;
};

struct auxdisplay_ssd1803a_config {
	struct auxdisplay_capabilities capabilities;
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec mode_gpio;
	struct spi_dt_spec bus;
};

static int auxdisplay_ssd1803a_spi_write(const struct device *dev, uint8_t flags, uint8_t val)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	int ret;

	/* 4-bit bus mode */
	uint8_t buf[3] = {SSD1803A_SB_SYNC_BITS | SSD1803A_SB_RW_WRITE | flags, val & 0x0F,
			  val >> 4};

	struct spi_buf tx_buf[] = {{.buf = buf, .len = sizeof(buf)}};
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 1};

	ret = spi_write_dt(&config->bus, &tx);
	k_usleep(100);
	return ret;
}

static int auxdisplay_ssd1803a_spi_write_multiple(const struct device *dev, uint8_t flags,
						  const uint8_t *val, size_t len)
{
	int ret;

	for (int i = 0; i < len; i++) {
		ret = auxdisplay_ssd1803a_spi_write(dev, flags, *val++);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

/**
 * Writes an instruction word regardless of the current IS or RE state.
 */
static inline int auxdisplay_ssd1803a_inst(const struct device *dev, uint8_t inst)
{
	LOG_DBG("%s inst: 0x%02x", dev->name, inst);

	return auxdisplay_ssd1803a_spi_write(dev, SSD1803A_SB_RS_INST, inst);
}

static inline int auxdisplay_ssd1803a_data(const struct device *dev, const uint8_t *data,
					   size_t len)
{
	LOG_DBG("%s %d B data", dev->name, len);
	return auxdisplay_ssd1803a_spi_write_multiple(dev, SSD1803A_SB_RS_DATA, data, len);
}

static int auxdisplay_ssd1803a_function_set(const struct device *dev, bool is, bool re)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	struct auxdisplay_ssd1803a_data *data = dev->data;
	uint8_t inst;
	int ret;

	/* The datasheet for the SSD1803A is really confusing. It seems to
	 * indicate that RE needs to be set according to its existing value
	 * (if RE starts at 0, you must write RE=0 when accessing this
	 * register). I suspect the intent is that RE is set when this register
	 * is written and the data is interpreted with the mode set in this
	 * write.
	 */
	if (is != data->is && re) {
		/* Set RE=0 to update IS, then proceed with RE=1 */
		ret = auxdisplay_ssd1803a_function_set(dev, is, false);
		if (ret) {
			return ret;
		}
	}

	LOG_DBG("re=%d is=%d", re, is);

	if (re) {
		inst = SSD1803A_FS_8BIT_MODE | SSD1803A_FS_RE |
		       (data->user_blink ? SSD1803A_FS_RE1_BLINK_EN : 0) |
		       (data->invert ? SSD1803A_FS_RE1_REV : 0);

	} else {
		inst = SSD1803A_FS_8BIT_MODE | (data->dbl_height ? SSD1803A_FS_RE0_DBL_HEIGHT : 0) |
		       (is ? SSD1803A_FS_RE0_IS : 0);
		data->is = is;
	}
	data->re = re;

	switch (config->capabilities.rows) {
	case 1:
	case 3:
		inst |= SSD1803A_FS_ROWS_1_3;
		break;
	case 2:
	case 4:
		inst |= SSD1803A_FS_ROWS_2_4;
		break;
	default:
		LOG_ERR("Invalid row count, assuming 4");
		inst |= SSD1803A_FS_ROWS_2_4;
	}

	return auxdisplay_ssd1803a_inst(dev, SSD1803A_INST_FUNCTION_SET | inst);
}

/* Executes function_set if needed to set the register mode */
static int auxdisplay_ssd1803a_set_reg_mode(const struct device *dev, bool is, bool re)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (is != data->is || re != data->re) {
		return auxdisplay_ssd1803a_function_set(dev, is, re);
	}
	return 0;
}

static int auxdisplay_ssd1803a_display_on_off(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	int ret;
	uint8_t inst;

	inst = (data->power ? SSD1803A_DO_DISPLAY_ON : 0) |
	       (data->cursor ? SSD1803A_DO_CURSOR_ON : 0) |
	       (data->blinking ? SSD1803A_DO_BLINKING_ON : 0);

	LOG_DBG("power %d cursor %d blink %d", data->power, data->cursor, data->blinking);
	ret = auxdisplay_ssd1803a_set_reg_mode(dev, data->is, false);
	if (ret != 0) {
		return ret;
	}

	return auxdisplay_ssd1803a_inst(dev, SSD1803A_RE0_INST_DISPLAY_ON_OFF | inst);
}

static int auxdisplay_ssd1803a_display_on(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (!data->power) {
		data->power = true;
		return auxdisplay_ssd1803a_display_on_off(dev);
	}
	return 0;
}

static int auxdisplay_ssd1803a_display_off(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (data->power) {
		data->power = false;
		return auxdisplay_ssd1803a_display_on_off(dev);
	}
	return 0;
}

static int auxdisplay_ssd1803a_cursor_set_enabled(const struct device *dev, bool enable)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (data->cursor != enable) {
		data->cursor = enable;
		return auxdisplay_ssd1803a_display_on_off(dev);
	}
	return 0;
}

static int auxdisplay_ssd1803a_position_blinking_set_enabled(const struct device *dev, bool enable)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (data->blinking != enable) {
		data->blinking = enable;
		return auxdisplay_ssd1803a_display_on_off(dev);
	}
	return 0;
}

static int auxdisplay_ssd1803a_ext_func_set(const struct device *dev)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	struct auxdisplay_ssd1803a_data *data = dev->data;
	uint8_t inst;
	int ret;

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, data->is, true);
	if (ret != 0) {
		return ret;
	}

	inst = SSD1803A_RE1_INST_EXT_FUNC_SET |
	       (data->invert_cursor ? SSD1803A_EXT_FUNC_INV_CURSOR : 0) |
	       (data->wide_font ? SSD1803A_EXT_FUNC_WIDE_FONT : 0) |
	       (config->capabilities.rows > 2 ? SSD1803A_EXT_FUNC_MORE_ROWS : 0);

	return auxdisplay_ssd1803a_inst(dev, inst);
}

static inline int auxdisplay_ssd1803a_ddram_address_set(const struct device *dev, uint8_t addr)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	auxdisplay_ssd1803a_set_reg_mode(dev, data->is, false);
	return auxdisplay_ssd1803a_inst(dev, SSD1803A_RE0_INST_DDRAM_ADDRESS_SET | (addr & 0x7F));
}

static int auxdisplay_ssd1803a_cursor_position_apply(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	uint8_t addr;

	addr = 0x20 * data->cursor_y + data->cursor_x;

	return auxdisplay_ssd1803a_ddram_address_set(dev, addr);
}

static int auxdisplay_ssd1803a_cursor_position_set(const struct device *dev,
						   enum auxdisplay_position type, int16_t x,
						   int16_t y)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (type == AUXDISPLAY_POSITION_RELATIVE) {
		x += data->cursor_x;
		y += data->cursor_y;
	} else if (type == AUXDISPLAY_POSITION_RELATIVE_DIRECTION) {
		return -EINVAL;
	}

	if (x < 0 || y < 0) {
		return -EINVAL;
	} else if (x >= config->capabilities.columns || y >= config->capabilities.rows) {
		return -EINVAL;
	}

	data->cursor_x = (uint16_t)x;
	data->cursor_y = (uint16_t)y;

	return auxdisplay_ssd1803a_cursor_position_apply(dev);
}

static int auxdisplay_ssd1803a_cursor_position_get(const struct device *dev, int16_t *x, int16_t *y)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	*x = (int16_t)data->cursor_x;
	*y = (int16_t)data->cursor_y;

	return 0;
}

static int auxdisplay_ssd1803a_capabilities_get(const struct device *dev,
						struct auxdisplay_capabilities *capabilities)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;

	memcpy(capabilities, &config->capabilities, sizeof(struct auxdisplay_capabilities));

	return 0;
}

static int auxdisplay_ssd1803a_clear(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	data->cursor_x = 0;
	data->cursor_y = 0;

	return auxdisplay_ssd1803a_inst(dev, SSD1803A_RE0_INST_CLEAR_DISPLAY);
}

static int auxdisplay_ssd1803a_brightness_set(const struct device *dev, uint8_t brightness)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	int ret;
	uint8_t inst;

	if (brightness < SSD1803A_BRIGHTNESS_MIN || brightness > SSD1803A_BRIGHTNESS_MAX) {
		return -EINVAL;
	}

	data->brightness = brightness;

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, true, false);
	if (ret != 0) {
		return ret;
	}

	/* Upper bits of contrast */
	inst = SSD1803A_IS1_RE0_INST_CONTRAST_H | (data->icon ? SSD1803A_CONTRAST_H_ICON_EN : 0) |
	       (data->boost ? SSD1803A_CONTRAST_H_BOOST_EN : 0) | (brightness >> 4);

	ret = auxdisplay_ssd1803a_inst(dev, inst);
	if (ret != 0) {
		return ret;
	}

	/* Lower bits of contrast */
	inst = SSD1803A_IS1_RE0_INST_CONTRAST_L | (brightness & BIT_MASK(4));

	return auxdisplay_ssd1803a_inst(dev, inst);
}

static int auxdisplay_ssd1803a_brightness_get(const struct device *dev, uint8_t *brightness)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;

	*brightness = data->brightness;

	return 0;
}

static int auxdisplay_ssd1803a_custom_character_set(const struct device *dev,
						    struct auxdisplay_character *character)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	int ret;
	/* Each custom character is 8 bytes */
	uint8_t buf[8];
	uint8_t *pxl;

	if (character->index >= config->capabilities.custom_characters) {
		LOG_ERR("invalid custom character index %d", character->index);
		return -EINVAL;
	}

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, false, false);
	if (ret != 0) {
		return ret;
	}

	/* Set the CGRAM starting address */
	ret = auxdisplay_ssd1803a_inst(dev, SSD1803A_IS0_RE0_INST_CGRAM_ADDRESS_SET |
						    (character->index << 3));
	if (ret != 0) {
		return ret;
	}

	/* Map the uint8_t buffer to the format used in CGRAM */
	pxl = character->data;
	memset(&buf[0], 0x00, sizeof(buf));
	for (int y = 0; y < config->capabilities.custom_character_height; y++) {
		for (int x = 0; x < config->capabilities.custom_character_width; x++) {
			if (*pxl++ > 0) {
				buf[y] |= BIT(x);
			}
		}
	}

	ret = auxdisplay_ssd1803a_data(dev, &buf[0], ARRAY_SIZE(buf));
	if (ret != 0) {
		return ret;
	}

	/* Set the character code used to index CGRAM */
	character->character_code = character->index;

	return 0;
}

static int auxdisplay_ssd1803a_select_cgrom(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	uint8_t val;
	int ret;

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, data->is, true);
	if (ret != 0) {
		return ret;
	}
	ret = auxdisplay_ssd1803a_inst(dev, SSD1803A_RE1_INST_ROM_SEL);
	if (ret != 0) {
		return ret;
	}
	val = data->cgrom_idx << 2;

	return auxdisplay_ssd1803a_data(dev, &val, 1);
}

static int auxdisplay_ssd1803a_set_dbl_height_fmt(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	int ret;
	uint8_t inst;

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, false, true);
	if (ret != 0) {
		return ret;
	}

	inst = BIT(4) | (data->dbl_height_fmt << 2) | (data->bias & 0x02);
	return auxdisplay_ssd1803a_inst(dev, inst);
}

static int auxdisplay_ssd1803a_set_osc_freq(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	int ret;
	uint8_t inst;

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, true, false);
	if (ret != 0) {
		return ret;
	}

	inst = BIT(4) | (data->osc_freq) | ((data->bias & 0x01) << 3);
	return auxdisplay_ssd1803a_inst(dev, inst);
}

static int auxdisplay_ssd1803a_follower_ctrl(const struct device *dev)
{
	struct auxdisplay_ssd1803a_data *data = dev->data;
	int ret;
	uint8_t inst;

	ret = auxdisplay_ssd1803a_set_reg_mode(dev, true, false);
	if (ret != 0) {
		return ret;
	}

	inst = SSD1803A_IS1_RE0_INST_FOLLOWER | (data->bias_divider ? BIT(3) : 0) |
	       (data->amp_ratio & BIT_MASK(3));
	return auxdisplay_ssd1803a_inst(dev, inst);
}

static int auxdisplay_ssd1803a_write(const struct device *dev, const uint8_t *text, uint16_t len)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	struct auxdisplay_ssd1803a_data *data = dev->data;
	int ret;
	uint16_t i = 0;

	while (i < len) {
		/* Calculate how much we can transfer at one time */
		uint16_t xfer_len = MIN(config->capabilities.columns - data->cursor_y, len);

		ret = auxdisplay_ssd1803a_data(dev, &text[i], xfer_len);
		if (ret) {
			return ret;
		}

		i += xfer_len;
		data->cursor_x += xfer_len;

		if (data->cursor_x >= config->capabilities.columns) {
			data->cursor_x = 0;
			data->cursor_y++;

			if (data->cursor_y == config->capabilities.rows) {
				data->cursor_y = 0;
			}

			ret = auxdisplay_ssd1803a_cursor_position_apply(dev);
			if (ret) {
				return ret;
			}
		}
	}

	return 0;
}

static int auxdisplay_ssd1803a_reset(const struct device *dev)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	int ret;

	/* Preset mode for SPI */
	ret = gpio_pin_configure_dt(&config->mode_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		printf("Configuring GPIO pin failed: %d\n", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		printf("Configuring GPIO pin failed: %d\n", ret);
		return ret;
	}

	/* Reset display */
	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret != 0) {
		LOG_ERR("Failed to reset display: %d", ret);
		return ret;
	}
	k_usleep(250);

	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (ret != 0) {
		LOG_ERR("Failed to deassert reset: %d", ret);
		return ret;
	}
	k_usleep(10);

	LOG_DBG("reset complete");

	return 0;
}

static int auxdisplay_ssd1803a_init(const struct device *dev)
{
	const struct auxdisplay_ssd1803a_config *config = dev->config;
	struct auxdisplay_ssd1803a_data *data = dev->data;

	if (!device_is_ready(config->bus.bus)) {
		return -ENODEV;
	}

	/* Validate settings */
	if (config->capabilities.columns * config->capabilities.rows > SSD1803A_CHAR_BUF_SIZE) {
		LOG_ERR("character count > %d", SSD1803A_CHAR_BUF_SIZE);
		return -EINVAL;
	}

	switch (config->capabilities.columns) {
	case 16:
	case 32:
	case 64:
		/* Use 6-column characters */
		data->wide_font = 1;
	default:
		/* Use 5-column characters */
		data->wide_font = 0;
	}

	auxdisplay_ssd1803a_reset(dev);

	/* RE=1 */
	auxdisplay_ssd1803a_select_cgrom(dev);
	auxdisplay_ssd1803a_ext_func_set(dev);
	auxdisplay_ssd1803a_set_dbl_height_fmt(dev);

	/* RE=0 */
	auxdisplay_ssd1803a_brightness_set(dev, data->brightness);
	auxdisplay_ssd1803a_set_osc_freq(dev);
	auxdisplay_ssd1803a_follower_ctrl(dev);
	auxdisplay_ssd1803a_clear(dev);
	auxdisplay_ssd1803a_display_on_off(dev);

	return 0;
}

static const struct auxdisplay_driver_api auxdisplay_ssd1803a_auxdisplay_api = {
	.display_on = auxdisplay_ssd1803a_display_on,
	.display_off = auxdisplay_ssd1803a_display_off,
	.cursor_set_enabled = auxdisplay_ssd1803a_cursor_set_enabled,
	.position_blinking_set_enabled = auxdisplay_ssd1803a_position_blinking_set_enabled,
	.cursor_position_set = auxdisplay_ssd1803a_cursor_position_set,
	.cursor_position_get = auxdisplay_ssd1803a_cursor_position_get,
	.capabilities_get = auxdisplay_ssd1803a_capabilities_get,
	.clear = auxdisplay_ssd1803a_clear,
	.brightness_get = auxdisplay_ssd1803a_brightness_get,
	.brightness_set = auxdisplay_ssd1803a_brightness_set,
	.custom_character_set = auxdisplay_ssd1803a_custom_character_set,
	.write = auxdisplay_ssd1803a_write,
};

#define AUXDISPLAY_SSD1803A_RE0_INST(n)                                                            \
	static const struct auxdisplay_ssd1803a_config auxdisplay_ssd1803a_config_##n = {          \
		.capabilities =                                                                    \
			{                                                                          \
				.columns = DT_INST_PROP(n, columns),                               \
				.rows = DT_INST_PROP(n, rows),                                     \
				.mode = 0,                                                         \
				.brightness.minimum = SSD1803A_BRIGHTNESS_MIN,                     \
				.brightness.maximum = SSD1803A_BRIGHTNESS_MAX,                     \
				.backlight.minimum = AUXDISPLAY_LIGHT_NOT_SUPPORTED,               \
				.backlight.maximum = AUXDISPLAY_LIGHT_NOT_SUPPORTED,               \
				.custom_characters = 8,                                            \
				.custom_character_width = 5,                                       \
				.custom_character_height = 8,                                      \
			},                                                                         \
		.reset_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(n), reset_gpios),                       \
		.mode_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(n), mode_gpios),                         \
		.bus = SPI_DT_SPEC_INST_GET(n,                                                     \
					    SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |   \
						    SPI_TRANSFER_LSB | SPI_WORD_SET(8),            \
					    0),                                                    \
	};                                                                                         \
												   \
	static struct auxdisplay_ssd1803a_data auxdisplay_ssd1803a_data_##n = {                    \
		.power = true,                                                                     \
		.cursor = DT_INST_PROP_OR(n, cursor, false),                                       \
		.blinking = DT_INST_PROP_OR(n, blink, false),                                      \
		.brightness = DT_INST_PROP_OR(n, brightness, 127),                                 \
		.invert = DT_INST_PROP_OR(n, invert, false),                                       \
		.dbl_height = DT_INST_PROP_OR(n, double_height, false),                            \
		.icon = DT_INST_PROP_OR(n, enable_segram, false),                                  \
		.boost = !DT_INST_PROP_OR(n, disable_boost, false),                                \
		.wide_font = false,                                                                \
		.bias_divider = true,                                                              \
		.user_blink = true,								   \
		.bias = 3,                                                                         \
		.dbl_height_fmt = 3,                                                               \
		.osc_freq = 3,                                                                     \
		.amp_ratio = 6,                                                                    \
		.cgrom_idx = DT_INST_PROP_OR(n, character_set, 0),                                 \
		.cursor_x = 0,                                                                     \
		.cursor_y = 0,                                                                     \
	};                                                                                         \
												   \
	DEVICE_DT_INST_DEFINE(n, &auxdisplay_ssd1803a_init, NULL, &auxdisplay_ssd1803a_data_##n,   \
			      &auxdisplay_ssd1803a_config_##n, POST_KERNEL,                        \
			      CONFIG_AUXDISPLAY_INIT_PRIORITY,                                     \
			      &auxdisplay_ssd1803a_auxdisplay_api);

DT_INST_FOREACH_STATUS_OKAY(AUXDISPLAY_SSD1803A_RE0_INST)
