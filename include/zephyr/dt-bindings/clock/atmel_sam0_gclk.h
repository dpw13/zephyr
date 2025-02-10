/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ATMEL_SAM0_GCLK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ATMEL_SAM0_GCLK_H_

#define GCLK_SRC_XOSC       (0x0)
#define GCLK_SRC_GCLKIN     (0x1)
#define GCLK_SRC_GCLKGEN1   (0x2)
#define GCLK_SRC_OSCULP32K  (0x3)
#define GCLK_SRC_OSC32K     (0x4)
#define GCLK_SRC_XOSC32K    (0x5)
#define GCLK_SRC_OSC8M      (0x6)
#define GCLK_SRC_DFLL48M    (0x7)
#define GCLK_SRC_FDPLL      (0x8)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ATMEL_SAM0_GCLK_H_ */
