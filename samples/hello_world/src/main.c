/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#define BIT(n)                      (1UL << (n))
#define ECREG(x)                    (*((volatile unsigned char *)(x)))

#define IT51XXX_PWM_BASE            0x00F04600
#define IT51XXX_PWM_REG(x)          ECREG(IT51XXX_PWM_BASE + (x))

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	printf("pwm_reg:\n");

	for (int i = 0; i <= 0xFF; i++) {
		if ((i != 0) && ((i % 16) == 0)) {
			printf("\n");
		}
		printf("0x%2x  ", IT51XXX_PWM_REG(i));
	}
	printf("\n");

	return 0;
}
