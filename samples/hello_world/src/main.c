/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#define BIT(n)                      (1UL << (n))
#define ECREG(x)                    (*((volatile unsigned char *)(x)))
#define IT8XXX2_WDT_BASE            0x00F01F00
#define IT8XXX2_WDT_CFG             ECREG(IT8XXX2_WDT_BASE + 0x81)
#define IT8XXX2_WDT_ET1PSR          ECREG(IT8XXX2_WDT_BASE + 0x82)
#define IT8XXX2_WDT_CTRL            ECREG(IT8XXX2_WDT_BASE + 0x85)
#define IT8XXX2_WDT_ET1CNT_L        ECREG(IT8XXX2_WDT_BASE + 0x84)
#define IT8XXX2_WDT_CNT_H           ECREG(IT8XXX2_WDT_BASE + 0x89)
#define IT8XXX2_WDT_CNT_L           ECREG(IT8XXX2_WDT_BASE + 0x86)
#define IT8XXX2_WDT_MAGIC_KEY       ECREG(IT8XXX2_WDT_BASE + 0x87)

#define IT8XXX2_WDT_OBSERV_CNT_H    ECREG(IT8XXX2_WDT_BASE + 0x99)
#define IT8XXX2_WDT_OBSERV_CNT_L    ECREG(IT8XXX2_WDT_BASE + 0x98)

int main(void)
{
	char reg_cfg = IT8XXX2_WDT_CFG;
	char reg_psr = IT8XXX2_WDT_ET1PSR;
	char reg_ctrl = IT8XXX2_WDT_CTRL;
	char reg_cnt_h = IT8XXX2_WDT_CNT_H;
	char reg_cnt_l = IT8XXX2_WDT_CNT_L;

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);


	printf("cfg 0x%x, psr 0x%x, ctrl 0x%x, cnt_h 0x%x, cnt_l 0x%x\n", reg_cfg, reg_psr, reg_ctrl, reg_cnt_h, reg_cnt_l);

	IT8XXX2_WDT_CFG = 0x00;
	IT8XXX2_WDT_ET1PSR = 0x01; /* 1024Hz */
	IT8XXX2_WDT_CFG = 0x30; /* enable magic key, clock source from prescaler */
	//IT8XXX2_WDT_CTRL &= ~ BIT(5); /* bit(5): 0b = don't stop watchdog timer counting */
	IT8XXX2_WDT_ET1CNT_L = 0x00; /* trigger clock */
	IT8XXX2_WDT_CNT_H = 0x01;
	IT8XXX2_WDT_CNT_L = 0x00;
	//IT8XXX2_WDT_CFG = 0x3A; /* lock WDT cnt and prescaler */

	reg_cfg = IT8XXX2_WDT_CFG;
	reg_psr = IT8XXX2_WDT_ET1PSR;
	reg_ctrl = IT8XXX2_WDT_CTRL;
	reg_cnt_h = IT8XXX2_WDT_CNT_H;
	reg_cnt_l = IT8XXX2_WDT_CNT_L;

	printf("cfg 0x%x, psr 0x%x, ctrl 0x%x, cnt_h 0x%x, cnt_l 0x%x\n", reg_cfg, reg_psr, reg_ctrl, reg_cnt_h, reg_cnt_l);


	//IT8XXX2_WDT_MAGIC_KEY = 0xaa; /* magic key not match 0x5c: trigger WDT reset immediately */

	printf("dummy observ_cnt_l 0x%x \n", IT8XXX2_WDT_OBSERV_CNT_L);

	//wait reset
	while(1) {
		printf("observ_cnt_l 0x%x \n", IT8XXX2_WDT_OBSERV_CNT_L);
		printf("observ_cnt_h 0x%x \n", IT8XXX2_WDT_OBSERV_CNT_H);
	}

	return 0;
}
