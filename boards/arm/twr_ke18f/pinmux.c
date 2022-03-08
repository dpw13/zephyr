/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <drivers/pinctrl.h>
#include <fsl_port.h>

static int twr_ke18f_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(porta), okay)
	__unused const struct device *porta =
		DEVICE_DT_GET(DT_NODELABEL(porta));
	__ASSERT_NO_MSG(device_is_ready(porta));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portb), okay)
	__unused const struct device *portb =
		DEVICE_DT_GET(DT_NODELABEL(portb));
	__ASSERT_NO_MSG(device_is_ready(portb));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portc), okay)
	__unused const struct device *portc =
		DEVICE_DT_GET(DT_NODELABEL(portc));
	__ASSERT_NO_MSG(device_is_ready(portc));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portd), okay)
	__unused const struct device *portd =
		DEVICE_DT_GET(DT_NODELABEL(portd));
	__ASSERT_NO_MSG(device_is_ready(portd));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porte), okay)
	__unused const struct device *porte =
		DEVICE_DT_GET(DT_NODELABEL(porte));
	__ASSERT_NO_MSG(device_is_ready(porte));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwt), okay) && CONFIG_PWM_CAPTURE
	/* PWM capture input on J20 pin 8 */
	pinmux_pin_set(porte, 11, PORT_PCR_MUX(kPORT_MuxAlt2));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) && CONFIG_ADC
	/* Thermistor A, B */
	pinmux_pin_set(porta, 0, PORT_PCR_MUX(kPORT_PinDisabledOrAnalog));
	pinmux_pin_set(porta, 1, PORT_PCR_MUX(kPORT_PinDisabledOrAnalog));
#endif

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) && CONFIG_ADC) || \
    (DT_NODE_HAS_STATUS(DT_NODELABEL(cmp2), okay) && CONFIG_MCUX_ACMP)
	/* Potentiometer */
	pinmux_pin_set(portc, 14, PORT_PCR_MUX(kPORT_PinDisabledOrAnalog));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(dac0), okay) && CONFIG_DAC
	pinmux_pin_set(porte, 9, PORT_PCR_MUX(kPORT_PinDisabledOrAnalog));
#endif

#ifdef CONFIG_BOARD_TWR_KE18F_FLEXIO_CLKOUT
	int err;

	/* Declare pin configuration state for flexio pin here */
	PINCTRL_DT_DEFINE(DT_NODELABEL(flexio));

	/* Apply pinctrl state directly, since there is no flexio device driver */
	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(flexio)),
		PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

#endif /* CONFIG_BOARD_TWR_KE18F_FLEXIO_CLOCKOUT */

	return 0;
}

SYS_INIT(twr_ke18f_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
