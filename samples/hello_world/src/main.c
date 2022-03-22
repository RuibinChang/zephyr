/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <soc.h>

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	/* Check ram code start address */
	//printk("SMFI_SCAR0 0x%x %x %x(=00 40 00)\n", SCRA0H, SCRA0M, SCRA0L);

	/* Disable global interrupt for critical section */
	unsigned int key = irq_lock();

	IT8XXX2_GPIO_GPCRA0 = 0x40;

	/* Test for 1 and 2 */
	k_busy_wait(5/*us*/); //first time call
	k_busy_wait(5/*us*/); //!first time call

	/* Test for 3 */
	k_busy_wait(10/*us*/); //!first time call
	k_busy_wait(100/*us*/); //!first time call
	k_busy_wait(200/*us*/); //!first time call
	k_busy_wait(500/*us*/); //!first time call
	k_busy_wait(2000/*us*/); //!first time call

	irq_unlock(key);
}
