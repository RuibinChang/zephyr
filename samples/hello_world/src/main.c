/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#define BIT(n)                      (1UL << (n))
#define ECREG(x)                    (*((volatile unsigned char *)(x)))
#define IT51XXX_WDT_BASE            0x00F04780
#define IT51XXX_WDT_CFG             ECREG(IT51XXX_WDT_BASE + 0x01)
#define IT51XXX_WDT_ET1PSR          ECREG(IT51XXX_WDT_BASE + 0x02)
#define IT51XXX_WDT_CTRL            ECREG(IT51XXX_WDT_BASE + 0x05)
#define IT51XXX_WDT_ET1CNT_L        ECREG(IT51XXX_WDT_BASE + 0x04)
#define IT51XXX_WDT_CNT_H           ECREG(IT51XXX_WDT_BASE + 0x09)
#define IT51XXX_WDT_CNT_L           ECREG(IT51XXX_WDT_BASE + 0x06)
#define IT51XXX_WDT_MAGIC_KEY       ECREG(IT51XXX_WDT_BASE + 0x07)
#define IT51XXX_WDT_OBSERV_CNT_H    ECREG(IT51XXX_WDT_BASE + 0x19)
#define IT51XXX_WDT_OBSERV_CNT_L    ECREG(IT51XXX_WDT_BASE + 0x18)

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	char reg_cfg = IT51XXX_WDT_CFG;
	char reg_psr = IT51XXX_WDT_ET1PSR;
	char reg_ctrl = IT51XXX_WDT_CTRL;
	char reg_cnt_h = IT51XXX_WDT_CNT_H;
	char reg_cnt_l = IT51XXX_WDT_CNT_L;

	printf("cfg 0x%x, psr 0x%x, ctrl 0x%x, cnt_h 0x%x, cnt_l 0x%x\n", reg_cfg, reg_psr, reg_ctrl, reg_cnt_h, reg_cnt_l);
	IT51XXX_WDT_CFG = 0x00;
	IT51XXX_WDT_ET1PSR = 0x01; /* 1024Hz */
	IT51XXX_WDT_CFG = 0x30; /* enable magic key, clock source from prescaler */
	//IT51XXX_WDT_CTRL &= ~ BIT(5); /* bit(5): 0b = don't stop watchdog timer counting */
	IT51XXX_WDT_ET1CNT_L = 0x00; /* trigger clock */
	IT51XXX_WDT_CNT_H = 0x01;
	IT51XXX_WDT_CNT_L = 0x00;
	//IT51XXX_WDT_CFG = 0x3A; /* lock WDT cnt and prescaler */
	reg_cfg = IT51XXX_WDT_CFG;
	reg_psr = IT51XXX_WDT_ET1PSR;
	reg_ctrl = IT51XXX_WDT_CTRL;
	reg_cnt_h = IT51XXX_WDT_CNT_H;
	reg_cnt_l = IT51XXX_WDT_CNT_L;
	printf("cfg 0x%x, psr 0x%x, ctrl 0x%x, cnt_h 0x%x, cnt_l 0x%x\n", reg_cfg, reg_psr, reg_ctrl, reg_cnt_h, reg_cnt_l);
	//IT51XXX_WDT_MAGIC_KEY = 0xaa; /* magic key not match 0x5c: trigger WDT reset immediately */
	printf("dummy observ_cnt_l 0x%x \n", IT51XXX_WDT_OBSERV_CNT_L);
	//wait reset
	while(1) {
		printf("observ_cnt_l 0x%x\n", IT51XXX_WDT_OBSERV_CNT_L);
		printf("observ_cnt_h 0x%x\n", IT51XXX_WDT_OBSERV_CNT_H);
	}

	return 0;
}
