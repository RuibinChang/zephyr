/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/interrupt_controller/wuc_ite_it8xxx2.h>
#include <zephyr/dt-bindings/gpio/ite-it8xxx2-gpio.h>
#include <zephyr/dt-bindings/interrupt-controller/it8xxx2-wuc.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-intc.h>
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <soc_dt.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include "gpio_utils.h"

#define DT_DRV_COMPAT ite_it8xxx2_gpio

struct gpio_wuc_map_cfg {
	/* WUC control device structure */
	const struct device *wucs;
	/* WUC pin mask */
	uint8_t mask;
};

/*
 * Structure gpio_ite_cfg is about the setting of gpio
 * this config will be used at initial time
 */
struct gpio_ite_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* gpio port data register (bit mapping to pin) */
	uintptr_t reg_gpdr;
	/* gpio port control register (byte mapping to pin) */
	uintptr_t reg_gpcr;
	/* gpio port data mirror register (bit mapping to pin) */
	uintptr_t reg_gpdmr;
	/* gpio port output type register (bit mapping to pin) */
	uintptr_t reg_gpotr;
	/* Number of gpio pins in the group are in use. */
	int ngpios;
	/* Index in gpio_1p8v for voltage level control register element. */
	uint8_t index;
	/* gpio's irq */
	uint8_t gpio_irq[8];
	/* gpio wake-up input source configuration list */
	const struct gpio_wuc_map_cfg *wuc_map_list;
};

/* Structure gpio_ite_data is about callback function */
struct gpio_ite_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

/* dev macros for GPIO */
#define DEV_GPIO_DATA(dev) \
	((struct gpio_ite_data *)(dev)->data)

#define DEV_GPIO_CFG(dev) \
	((const struct gpio_ite_cfg *)(dev)->config)

/* 1.8v gpio group a, b, c, d, e, f, g, h, i, j, k, l, and m */
#define GPIO_GROUP_COUNT 13
#define GPIO_GROUP_INDEX(label) \
		(uint8_t)(DT_REG_ADDR(DT_NODELABEL(label)) - \
			  DT_REG_ADDR(DT_NODELABEL(gpioa)))

/* general control registers for selecting 1.8V/3.3V */
static const struct {
	uint8_t offset;
	uint8_t mask_1p8v;
} gpio_1p8v[GPIO_GROUP_COUNT][8] = {
	[GPIO_GROUP_INDEX(gpioa)] = {
		[4] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(0)},
		[5] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(1)},
		[6] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(5)},
		[7] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(6)} },
	[GPIO_GROUP_INDEX(gpiob)] = {
		[3] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(1)},
		[4] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(0)},
		[5] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(7)},
		[6] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(6)},
		[7] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(4)} },
	[GPIO_GROUP_INDEX(gpioc)] = {
		[0] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(7)},
		[1] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(5)},
		[2] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(4)},
		[4] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(2)},
		[6] = {IT8XXX2_GPIO_GCR24_OFFSET, BIT(3)},
		[7] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(3)} },
	[GPIO_GROUP_INDEX(gpiod)] = {
		[0] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(2)},
		[1] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(1)},
		[2] = {IT8XXX2_GPIO_GCR19_OFFSET, BIT(0)},
		[3] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(7)},
		[4] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(6)},
		[5] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(4)},
		[6] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(5)},
		[7] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(6)} },
	[GPIO_GROUP_INDEX(gpioe)] = {
		[0] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(5)},
		[1] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(6)},
		[2] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(7)},
		[4] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(2)},
		[5] = {IT8XXX2_GPIO_GCR22_OFFSET, BIT(3)},
		[6] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(4)},
		[7] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(3)} },
	[GPIO_GROUP_INDEX(gpiof)] = {
		[0] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(4)},
		[1] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(5)},
		[2] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(2)},
		[3] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(1)},
		[4] = {IT8XXX2_GPIO_GCR20_OFFSET, BIT(0)},
		[5] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(7)},
		[6] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(6)},
		[7] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(5)} },
	[GPIO_GROUP_INDEX(gpiog)] = {
		[0] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(2)},
		[1] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(4)},
		[2] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(3)},
		[6] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(3)} },
	[GPIO_GROUP_INDEX(gpioh)] = {
		[0] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(2)},
		[1] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(1)},
		[2] = {IT8XXX2_GPIO_GCR21_OFFSET, BIT(0)},
		[5] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(7)},
		[6] = {IT8XXX2_GPIO_GCR28_OFFSET, BIT(0)} },
	[GPIO_GROUP_INDEX(gpioi)] = {
		[0] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(3)},
		[1] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(4)},
		[2] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(5)},
		[3] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(6)},
		[4] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(7)},
		[5] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(4)},
		[6] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(5)},
		[7] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(6)} },
	[GPIO_GROUP_INDEX(gpioj)] = {
		[0] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(0)},
		[1] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(1)},
		[2] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(2)},
		[3] = {IT8XXX2_GPIO_GCR23_OFFSET, BIT(3)},
		[4] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(0)},
		[5] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(1)},
		[6] = {IT8XXX2_GPIO_GCR27_OFFSET, BIT(2)},
		[7] = {IT8XXX2_GPIO_GCR33_OFFSET, BIT(2)} },
	[GPIO_GROUP_INDEX(gpiok)] = {
		[0] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(0)},
		[1] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(1)},
		[2] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(2)},
		[3] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(3)},
		[4] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(4)},
		[5] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(5)},
		[6] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(6)},
		[7] = {IT8XXX2_GPIO_GCR26_OFFSET, BIT(7)} },
	[GPIO_GROUP_INDEX(gpiol)] = {
		[0] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(0)},
		[1] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(1)},
		[2] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(2)},
		[3] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(3)},
		[4] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(4)},
		[5] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(5)},
		[6] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(6)},
		[7] = {IT8XXX2_GPIO_GCR25_OFFSET, BIT(7)} },
	/*
	 * M group's voltage level is according to chip's VCC is connected
	 * to 1.8V or 3.3V.
	 */
	[GPIO_GROUP_INDEX(gpiom)] = {
		[0] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)},
		[1] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)},
		[2] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)},
		[3] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)},
		[4] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)},
		[5] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)},
		[6] = {IT8XXX2_GPIO_GCR30_OFFSET, BIT(4)} },
};

/**
 * Driver functions
 */
static int gpio_ite_configure(const struct device *dev,
				gpio_pin_t pin,
				gpio_flags_t flags)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);

	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;
	volatile uint8_t *reg_gpcr = (uint8_t *)(gpio_config->reg_gpcr + pin);
	volatile uint8_t *reg_gpotr = (uint8_t *)gpio_config->reg_gpotr;
	volatile uint8_t *reg_1p8v;
	volatile uint8_t mask_1p8v;
	uint8_t mask = BIT(pin);

	__ASSERT(gpio_config->index < GPIO_GROUP_COUNT,
		"Invalid GPIO group index");

	/* Don't support "open source" mode */
	if (((flags & GPIO_SINGLE_ENDED) != 0) &&
	    ((flags & GPIO_LINE_OPEN_DRAIN) == 0)) {
		return -ENOTSUP;
	}

	/*
	 * Select open drain first, so that we don't glitch the signal
	 * when changing the line to an output.
	 */
	if (flags & GPIO_OPEN_DRAIN)
		*reg_gpotr |= mask;
	else
		*reg_gpotr &= ~mask;

	/* 1.8V or 3.3V */
	reg_1p8v = &IT8XXX2_GPIO_GCRX(
			gpio_1p8v[gpio_config->index][pin].offset);
	mask_1p8v = gpio_1p8v[gpio_config->index][pin].mask_1p8v;
	if (reg_1p8v != &IT8XXX2_GPIO_GCRX(0)) {
		gpio_flags_t volt = flags & IT8XXX2_GPIO_VOLTAGE_MASK;

		if (volt == IT8XXX2_GPIO_VOLTAGE_1P8) {
			__ASSERT(!(flags & GPIO_PULL_UP),
			"Don't enable internal pullup if 1.8V voltage is used");
			*reg_1p8v |= mask_1p8v;
		} else if (volt == IT8XXX2_GPIO_VOLTAGE_3P3 ||
			   volt == IT8XXX2_GPIO_VOLTAGE_DEFAULT) {
			*reg_1p8v &= ~mask_1p8v;
		} else {
			return -EINVAL;
		}
	}

	/* If output, set level before changing type to an output. */
	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_HIGH)
			*reg_gpdr |= mask;
		else if (flags & GPIO_OUTPUT_INIT_LOW)
			*reg_gpdr &= ~mask;
	}

	/* Set input or output. */
	if (flags & GPIO_OUTPUT)
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_OUTPUT) &
				~GPCR_PORT_PIN_MODE_INPUT;
	else
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_INPUT) &
				~GPCR_PORT_PIN_MODE_OUTPUT;

	/* Handle pullup / pulldown */
	if (flags & GPIO_PULL_UP) {
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_PULLUP) &
				~GPCR_PORT_PIN_MODE_PULLDOWN;
	} else if (flags & GPIO_PULL_DOWN) {
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_PULLDOWN) &
				~GPCR_PORT_PIN_MODE_PULLUP;
	} else {
		/* No pull up/down */
		*reg_gpcr &= ~(GPCR_PORT_PIN_MODE_PULLUP |
				GPCR_PORT_PIN_MODE_PULLDOWN);
	}

	return 0;
}

static int gpio_ite_port_get_raw(const struct device *dev,
					gpio_port_value_t *value)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	volatile uint8_t *reg_gpdmr = (uint8_t *)gpio_config->reg_gpdmr;

	/* Get raw bits of GPIO mirror register */
	*value = *reg_gpdmr;

	return 0;
}

static int gpio_ite_port_set_masked_raw(const struct device *dev,
					gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;
	uint8_t out = *reg_gpdr;

	*reg_gpdr = ((out & ~mask) | (value & mask));

	return 0;
}

static int gpio_ite_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;

	/* Set raw bits of GPIO data register */
	*reg_gpdr |= pins;

	return 0;
}

static int gpio_ite_port_clear_bits_raw(const struct device *dev,
						gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;

	/* Clear raw bits of GPIO data register */
	*reg_gpdr &= ~pins;

	return 0;
}

static int gpio_ite_port_toggle_bits(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;

	/* Toggle raw bits of GPIO data register */
	*reg_gpdr ^= pins;

	return 0;
}

static int gpio_ite_manage_callback(const struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_ite_data *data = DEV_GPIO_DATA(dev);

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static void gpio_ite_isr(const void *arg)
{
	uint8_t irq = ite_intc_get_irq_num();
	const struct device *dev = arg;
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	struct gpio_ite_data *data = DEV_GPIO_DATA(dev);
	int ngpios = gpio_config->ngpios;
	uint8_t gpio_pin = 0;

	/* Find out the pin of gpio group */
	for (gpio_pin = 0; gpio_pin <= ngpios; gpio_pin++) {
		if (irq == gpio_config->gpio_irq[gpio_pin]) {
			break;
		}
	}

	/* W/C wakeup interrupt status */
	it8xxx2_wuc_clear_status(gpio_config->wuc_map_list[gpio_pin].wucs,
				 gpio_config->wuc_map_list[gpio_pin].mask);

	gpio_fire_callbacks(&data->callbacks, dev, BIT(gpio_pin));
}

static int gpio_ite_pin_interrupt_configure(const struct device *dev,
						gpio_pin_t pin,
						enum gpio_int_mode mode,
						enum gpio_int_trig trig)
{
	const struct gpio_ite_cfg *gpio_config = DEV_GPIO_CFG(dev);
	uint8_t gpio_irq = gpio_config->gpio_irq[pin];
	const struct device *wuc_wucs = gpio_config->wuc_map_list[pin].wucs;
	uint8_t wuc_mask = gpio_config->wuc_map_list[pin].mask;
	uint32_t flag = 0;

	if (mode == GPIO_INT_MODE_DISABLED) {
		/* Disable GPIO interrupt */
		irq_disable(gpio_irq);
		return 0;
	}

	if (mode == GPIO_INT_MODE_LEVEL) {
		printk("Level trigger mode not supported.\r\n");
		return -ENOTSUP;
	}

	/* Disable irq before configuring it */
	irq_disable(gpio_irq);

	if (trig & GPIO_INT_TRIG_BOTH) {
		if (pin >= gpio_config->ngpios) {
			printk("GPIO port%d pin%d isn't in use.\r\n",
			       gpio_config->index, pin);
			return -EINVAL;
		}

		/* Set wakeup interrupt trigger edge */
		if ((trig & GPIO_INT_TRIG_BOTH) == GPIO_INT_TRIG_BOTH) {
			flag = WUC_TYPE_EDGE_BOTH;
		} else if (trig & GPIO_INT_TRIG_LOW) {
			flag = WUC_TYPE_EDGE_FALLING;
		} else {
			flag = WUC_TYPE_EDGE_RISING;
		}
		it8xxx2_wuc_set_polarity(wuc_wucs, wuc_mask, flag);

		/* W/C wakeup interrupt status */
		it8xxx2_wuc_clear_status(wuc_wucs, wuc_mask);

		/* Enable wakeup interrupt */
		it8xxx2_wuc_enable(wuc_wucs, wuc_mask);
	}

	/* Enable GPIO interrupt */
	irq_connect_dynamic(gpio_irq, 0, gpio_ite_isr, dev, 0);
	irq_enable(gpio_irq);

	return 0;
}

static const struct gpio_driver_api gpio_ite_driver_api = {
	.pin_configure = gpio_ite_configure,
	.port_get_raw = gpio_ite_port_get_raw,
	.port_set_masked_raw = gpio_ite_port_set_masked_raw,
	.port_set_bits_raw = gpio_ite_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ite_port_clear_bits_raw,
	.port_toggle_bits = gpio_ite_port_toggle_bits,
	.pin_interrupt_configure = gpio_ite_pin_interrupt_configure,
	.manage_callback = gpio_ite_manage_callback,
};

static int gpio_ite_init(const struct device *dev)
{
	return 0;
}

#define GPIO_ITE_DEV_CFG_DATA(inst)                                            \
static const struct gpio_wuc_map_cfg                                           \
	gpio_wuc_##inst[IT8XXX2_DT_INST_WUCCTRL_LEN(inst)] =                   \
		IT8XXX2_DT_WUC_ITEMS_LIST(inst);                               \
static struct gpio_ite_data gpio_ite_data_##inst;                              \
static const struct gpio_ite_cfg gpio_ite_cfg_##inst = {                       \
	.common = {                                                            \
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(               \
		DT_INST_PROP(inst, ngpios))                                    \
	},                                                                     \
	.reg_gpdr = DT_INST_REG_ADDR_BY_IDX(inst, 0),                          \
	.reg_gpcr = DT_INST_REG_ADDR_BY_IDX(inst, 1),                          \
	.reg_gpdmr = DT_INST_REG_ADDR_BY_IDX(inst, 2),                         \
	.reg_gpotr = DT_INST_REG_ADDR_BY_IDX(inst, 3),                         \
	.ngpios = DT_INST_PROP(inst, ngpios),                                  \
	.index = (uint8_t)(DT_INST_REG_ADDR(inst) -                            \
			   DT_REG_ADDR(DT_NODELABEL(gpioa))),                  \
	.gpio_irq[0] = DT_INST_IRQ_BY_IDX(inst, 0, irq),                       \
	.gpio_irq[1] = DT_INST_IRQ_BY_IDX(inst, 1, irq),                       \
	.gpio_irq[2] = DT_INST_IRQ_BY_IDX(inst, 2, irq),                       \
	.gpio_irq[3] = DT_INST_IRQ_BY_IDX(inst, 3, irq),                       \
	.gpio_irq[4] = DT_INST_IRQ_BY_IDX(inst, 4, irq),                       \
	.gpio_irq[5] = DT_INST_IRQ_BY_IDX(inst, 5, irq),                       \
	.gpio_irq[6] = DT_INST_IRQ_BY_IDX(inst, 6, irq),                       \
	.gpio_irq[7] = DT_INST_IRQ_BY_IDX(inst, 7, irq),                       \
	.wuc_map_list = gpio_wuc_##inst,                                       \
	};                                                                     \
DEVICE_DT_INST_DEFINE(inst,                                                    \
		gpio_ite_init,                                                 \
		NULL,                                                          \
		&gpio_ite_data_##inst,                                         \
		&gpio_ite_cfg_##inst,                                          \
		PRE_KERNEL_1,                                                  \
		CONFIG_GPIO_INIT_PRIORITY,                                     \
		&gpio_ite_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_ITE_DEV_CFG_DATA)

static int gpio_it8xxx2_init_set(const struct device *arg)
{
	ARG_UNUSED(arg);

	if (IS_ENABLED(CONFIG_SOC_IT8XXX2_GPIO_GROUP_K_L_DEFAULT_PULL_DOWN)) {
		const struct device *gpiok = DEVICE_DT_GET(DT_NODELABEL(gpiok));
		const struct device *gpiol = DEVICE_DT_GET(DT_NODELABEL(gpiol));

		for (int i = 0; i < 8; i++) {
			gpio_pin_configure(gpiok, i, GPIO_INPUT | GPIO_PULL_DOWN);
			gpio_pin_configure(gpiol, i, GPIO_INPUT | GPIO_PULL_DOWN);
		}
	}

	if (IS_ENABLED(CONFIG_SOC_IT8XXX2_GPIO_H7_DEFAULT_OUTPUT_LOW)) {
		const struct device *gpioh = DEVICE_DT_GET(DT_NODELABEL(gpioh));

		gpio_pin_configure(gpioh, 7, GPIO_OUTPUT_LOW);
	}

	return 0;
}
SYS_INIT(gpio_it8xxx2_init_set, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY);
