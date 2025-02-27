/*
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <soc.h>

LOG_MODULE_REGISTER(power, CONFIG_PM_LOG_LEVEL);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	if (state == PM_STATE_ACTIVE) {
		gpio_pin_set_dt(&led, 1);
		/* Clear deepsleep bit */
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		return;
	}

	LOG_DBG("state %d substate %d", state, substate_id);
	gpio_pin_set_dt(&led, 0);

	switch (state) {
		case PM_STATE_RUNTIME_IDLE:
			/* Only halt the CPU; AHB and APB remain active */
			REG_PM_SLEEP = PM_SLEEP_IDLE_CPU;
			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
			break;
		case PM_STATE_SUSPEND_TO_IDLE:
			if (substate_id == 0) {
				/* Suspend AHB buses */
				REG_PM_SLEEP = PM_SLEEP_IDLE_AHB;
			} else {
				/* Suspend AHB and APB buses*/
				REG_PM_SLEEP = PM_SLEEP_IDLE_APB;
			}
			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
			break;
		case PM_STATE_STANDBY:
		default:
			SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
			break;
	}

	/* Set PRIMASK */
	__disable_irq();

	/* Set BASEPRI to 0 */
	irq_unlock(0);

	__DSB();
	__WFI();
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	gpio_pin_set_dt(&led, 1);
	/* Clear PRIMASK */
	__enable_irq();
}

