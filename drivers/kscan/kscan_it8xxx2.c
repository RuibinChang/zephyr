/*
 * Copyright (c) 2021 ITE Corporation. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_kscan		/*= dtsi compatible*/

#include <errno.h>
#include <device.h>
#include <drivers/kscan.h>
#include <kernel.h>
#include <soc.h>
#include <sys/atomic.h>
#include <logging/log.h>

#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
LOG_MODULE_REGISTER(kscan_it8xxx2);			/*= file name*/

#define MAX_MATRIX_KEY_COLS CONFIG_KSCAN_IT8XXX2_COLUMN_SIZE
#define MAX_MATRIX_KEY_ROWS CONFIG_KSCAN_IT8XXX2_ROW_SIZE
#define KSOH_PIN_MASK (((1 << (MAX_MATRIX_KEY_COLS - 8)) - 1) & 0xff)
#undef CONFIG_KEYBOARD_COL2_INVERTED		/*TODO:move to board level*/

#define KEYBOARD_COLUMN_DRIVE_ALL       -2
#define KEYBOARD_COLUMN_DRIVE_NONE      -1

/* Poll period/debouncing, Timer4 count +1 per 1us, so needn't trnasform operation. */
#define CLOCK_HW_CYCLES_TO_US(X)        (uint64_t)(X)
/* Milliseconds in microseconds */
#define MSEC 1000U
/* Number of tracked scan times */
#define SCAN_OCURRENCES 30U
/* Thread stack size */
#define TASK_STACK_SIZE 1024

#define IT83XX_IRQ_WKINTC  13				/*TODO: move to soc_common.h(soc\riscv\ite...)*/


struct kscan_it8xxx2_data {
	/* variables in usec units */
	uint32_t deb_time_press;
	uint32_t deb_time_rel;
	int64_t poll_timeout;
	uint32_t poll_period;
	uint8_t matrix_stable_state[MAX_MATRIX_KEY_COLS];
	uint8_t matrix_unstable_state[MAX_MATRIX_KEY_COLS];
	uint8_t matrix_previous_state[MAX_MATRIX_KEY_COLS];
	/* Index in to the scan_clock_cycle to indicate start of debouncing */
	uint8_t scan_cycle_idx[MAX_MATRIX_KEY_COLS][MAX_MATRIX_KEY_ROWS];
	/* Track previous "elapsed clock cycles" per matrix scan. This
	 * is used to calculate the debouncing time for every key
	 */
	uint8_t scan_clk_cycle[SCAN_OCURRENCES];
	struct k_sem poll_lock;
	uint8_t scan_cycles_idx;
	kscan_callback_t callback;
	struct k_thread thread;
	atomic_t enable_scan;

	K_KERNEL_STACK_MEMBER(thread_stack, TASK_STACK_SIZE);
};

static struct kscan_it8xxx2_data kbd_data;
extern void ite_intc_isr_clear(unsigned int irq);

DEVICE_DECLARE(kscan_it8xxx2);

/*
 * Drive the specified column low.
 */
static void drive_keyboard_column(int col)
{
	int mask;
	unsigned int key;

	/* Tri-state all outputs */
	if (col == KEYBOARD_COLUMN_DRIVE_NONE)
		mask = 0xffff;
	/* Assert all outputs */
	else if (col == KEYBOARD_COLUMN_DRIVE_ALL)
		mask = 0;
	/* Assert a single output */
	else
		mask = 0xffff ^ BIT(col);
#ifdef CONFIG_KEYBOARD_COL2_INVERTED
	/* KSO[2] is inverted. */
	mask ^= BIT(2);
#endif
	KSOL = mask & 0xff;
	/* critical section with interrupts off */
	key = irq_lock(); //int_mask = read_clear_int_mask();
	/*
	 * Because IT83XX_KBS_KSOH1 register is shared by keyboard scan
	 * out and GPIO output mode, so we don't drive all KSOH pins
	 * here (this depends on how many keyboard matrix output pin
	 * we are using).
	 */
	KSOH1 = (KSOH1 & ~KSOH_PIN_MASK) | ((mask >> 8) & KSOH_PIN_MASK);
	//KSOH2 = 0x00; /*KSO[17:16] output data*/
	/* restore interrupts */
	irq_unlock(key); //set_int_mask(int_mask);

	printk("drive_kb_col : %d (0~15 SingleHigh, -1 ALLHigh, -2 ALLLow)\n", col);

	#if 0 				/*seems not same with ITE*/
	if (data == KEYBOARD_COLUMN_DRIVE_ALL) {
		/* KSO output controlled by the KSO_SELECT field */
		base->KSO_SEL = MCHP_KSCAN_KSO_ALL;
	} else if (data == KEYBOARD_COLUMN_DRIVE_NONE) {
		/* Keyboard scan disabled. All KSO output buffers disabled */
		base->KSO_SEL = MCHP_KSCAN_KSO_EN;
	} else {
		/* It is assumed, KEYBOARD_COLUMN_DRIVE_ALL was
		 * previously set
		 */
		base->KSO_SEL = data;
	}
	#endif
}

/*
 * Read raw row state.
 * Bits are 1 if signal is present, 0 if not present.
 */
static uint8_t read_keyboard_row(void)
{
	/* Bits are active-low, so invert returned levels */
	return KSI ^ 0xff;

	#if 0
	/* In this implementation a 1 means key pressed */
	return ~(base->KSI_IN & 0xFF);
	#endif
}

static bool is_matrix_ghosting(const uint8_t *state)
{
	/* matrix keyboard designs are suceptible to ghosting.
	 * An extra key appears to be pressed when 3 keys
	 * belonging to the same block are pressed.
	 * for example, in the following block
	 *
	 * . . w . q .
	 * . . . . . .
	 * . . . . . .
	 * . . m . a .
	 *
	 * the key m would look as pressed if the user pressed keys
	 * w, q and a simultaneously. A block can also be formed,
	 * with not adjacent columns.
	 */
	for (int c = 0; c <  MAX_MATRIX_KEY_COLS; c++) {
		if (!state[c])
			continue;

		for (int c_n = c + 1; c_n <  MAX_MATRIX_KEY_COLS; c_n++) {
			/* we and the columns to detect a "block".
			 * this is an indication of ghosting, due to current
			 * flowing from a key which was never pressed. in our
			 * case, current flowing is a bit set to 1 as we
			 * flipped the bits when the matrix was scanned.
			 * now we or the colums using z&(z-1) which is
			 * non-zero only if z has more than one bit set.
			 */
			uint8_t common_row_bits = state[c] & state[c_n];

			if (common_row_bits & (common_row_bits - 1))
				return true;
		}
	}

	return false;
}

static bool read_keyboard_matrix(uint8_t *new_state)
{
	uint8_t row;
	uint8_t key_event = 0U;

	for (int col = 0; col < MAX_MATRIX_KEY_COLS; col++) {
		drive_keyboard_column(col); /*chrome:drive all column high first, then drive each column low => derectly XOR oper has the same result*/

		/* Allow the matrix to stabilize before reading it */
		k_busy_wait(50U);
		row = read_keyboard_row();
		new_state[col] = row;

		printk("new_state[%d (col)] = 0x%x (row)\n", col, new_state[col]);
		key_event |= row;
	}

	drive_keyboard_column(KEYBOARD_COLUMN_DRIVE_NONE);

	return key_event != 0U ? true : false;
}

/*
 * Interrupt handler for keyboard matrix scan interrupt.
 */
static void keyboard_raw_interrupt(const void *arg)
{
	ARG_UNUSED(arg);

	printk("KBS Interrupt fired\n");

	//check status
	printk("INT WUESR3 = 0x%x, ISR1 = 0x%x(bit5)\n", WUESR3, ISR1);

	WUESR3 = 0xFF;
	ite_intc_isr_clear(IT83XX_IRQ_WKINTC); //task_clear_pending_irq(IT83XX_IRQ_WKINTC);
	//irq_disable(IT83XX_IRQ_WKINTC/*DT_INST_IRQN(0)*/);		   /*After INT fired, poll task not dis ISR, so MicroChip dsiable ISR do in here. I move to poll task same as chrome*/ 
	k_sem_give(&kbd_data.poll_lock); // = task_set_event(XXX)

	//check status
	printk("INT WUESR3 = 0x%x, ISR1 = 0x%x(bit5)\n", WUESR3, ISR1);

	LOG_DBG(" ");
}

/*
 * Enable or disable keyboard matrix scan interrupts.
 */
void keyboard_raw_enable_interrupt(int enable)
{
	//check status
	printk("en/dis %d, WUESR3 = 0x%x, ISR1 = 0x%x(bit5)\n", enable, WUESR3, ISR1);

	if (enable) {
		WUESR3 = 0xFF;
		ite_intc_isr_clear(IT83XX_IRQ_WKINTC); //task_clear_pending_irq(IT83XX_IRQ_WKINTC);
		irq_enable(IT83XX_IRQ_WKINTC); //task_enable_irq(IT83XX_IRQ_WKINTC);
	} else
		irq_disable(IT83XX_IRQ_WKINTC); //task_disable_irq(IT83XX_IRQ_WKINTC);

	//check status
	printk("en/dis %d, WUESR3 = 0x%x, ISR1 = 0x%x(bit5)\n", enable, WUESR3, ISR1);
}

static bool check_key_events(const struct device *dev)
{
	uint8_t matrix_new_state[MAX_MATRIX_KEY_COLS] = {0U};
	bool key_pressed = false;
	uint32_t cycles_now  = k_cycle_get_32();

	if (++kbd_data.scan_cycles_idx >= SCAN_OCURRENCES)
		kbd_data.scan_cycles_idx = 0U;

	kbd_data.scan_clk_cycle[kbd_data.scan_cycles_idx] = cycles_now;

	/* Scan the matrix */
	key_pressed = read_keyboard_matrix(matrix_new_state);

	/* Abort if ghosting is detected */
	if (is_matrix_ghosting(matrix_new_state)) {
		printk("ghosting is detected\n");
		return false;
	}

	uint8_t row_changed = 0U;
	uint8_t deb_col;

	/* The intent of this loop is to gather information related to key
	 * changes.
	 */
	for (int c = 0; c < MAX_MATRIX_KEY_COLS; c++) {
		/* Check if there was an update from the previous scan */
		row_changed = matrix_new_state[c] ^
					kbd_data.matrix_previous_state[c];

		if (!row_changed)
			continue;

		for (int r = 0; r < MAX_MATRIX_KEY_ROWS; r++) {
			/* Index all they keys that changed for each row
			 * in order to debounce each key in terms of it
			 */
			if (row_changed & BIT(r))
				kbd_data.scan_cycle_idx[c][r] =
					kbd_data.scan_cycles_idx;
		}

		kbd_data.matrix_unstable_state[c] |= row_changed;
		kbd_data.matrix_previous_state[c] = matrix_new_state[c];
	}

	printk("checking1\n");

	for (int c = 0; c < MAX_MATRIX_KEY_COLS; c++) {
		deb_col = kbd_data.matrix_unstable_state[c];

		if (!deb_col)                      /*find the pressed col*/
			continue;

		/* Debouncing for each row key occurs here */
		for (int r = 0; r < MAX_MATRIX_KEY_ROWS; r++) {
			uint8_t mask = BIT(r);
			uint8_t row_bit = matrix_new_state[c] & mask;

			/* Continue if we already debounce a key */
			if (!(deb_col & mask))         /*find the pressed row*/
				continue;

			/* Convert the clock cycle differences to usec */
			uint32_t debt = CLOCK_HW_CYCLES_TO_US(cycles_now -
			kbd_data.scan_clk_cycle[kbd_data.scan_cycle_idx[c][r]]);

			/* Does the key requires more time to be debounced? */
			if (debt < (row_bit ? kbd_data.deb_time_press :
						kbd_data.deb_time_rel)) {
				/* Need more time to debounce */
				continue;
			}

			kbd_data.matrix_unstable_state[c] &= ~row_bit;

			/* Check if there was a change in the stable state */
			if ((kbd_data.matrix_stable_state[c] & mask)
								== row_bit) {
				/* Key state did not change */
				continue;

			}

			/* The current row has been debounced, therefore update
			 * the stable state. Then, proceed to notify the
			 * application about the keys pressed.
			 */
			kbd_data.matrix_stable_state[c] ^= mask;
			if (atomic_get(&kbd_data.enable_scan) == 1U) {
				//kbd_data.callback(dev, r, c,
				//	      row_bit ? true : false); /*excute which function? maybe notify host function*/				
			}
		}
		printk("checking2\n");
	}

	printk("key_pressed = 0x%x\n", key_pressed);
	return key_pressed;
}

static bool poll_expired(uint32_t start_cycles, int64_t *timeout)
{
	uint32_t now_cycles;
	uint32_t microsecs_spent;

	now_cycles = k_cycle_get_32();
	microsecs_spent =  now_cycles - start_cycles;

	/* Update the timeout value */
	*timeout -= microsecs_spent;

	return !(*timeout >= 0);
}

void polling_task(void *dummy1, void *dummy2, void *dummy3)
{
	uint32_t current_cycles;
	uint32_t cycles_diff;
	uint32_t wait_period;
	int64_t local_poll_timeout = kbd_data.poll_timeout;

	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	while (true) {
		//base->KSI_STS = MCHP_KSCAN_KSO_SEL_REG_MASK; //?

		/* Ignore isr when releasing a key as we are polling */
		//MCHP_GIRQ_SRC(MCHP_KSCAN_GIRQ) = BIT(MCHP_KSCAN_GIRQ_POS); //?
		printk("poll task running\n");
		printk("KSOL = 0x%x, KSOH1 = 0x%x\n", KSOL, KSOH1);

		drive_keyboard_column(KEYBOARD_COLUMN_DRIVE_ALL); /*Last KSO status is ALL output high, KSO high to low change somehow don't know will triger KSI INT (measured Low pulse on KSI pin), so do before INT_EN*/
		keyboard_raw_enable_interrupt(1);
		k_sem_take(&kbd_data.poll_lock, K_FOREVER); // = task_wait_event(-1)
		keyboard_raw_enable_interrupt(0);

		uint32_t start_poll_cycles = k_cycle_get_32();

		while (atomic_get(&kbd_data.enable_scan) == 1U) {
			uint32_t start_period_cycles = k_cycle_get_32();
			printk("start_period_cycles : %u\n", start_period_cycles);

			if (check_key_events(DEVICE_GET(kscan_it8xxx2))) {
				local_poll_timeout = kbd_data.poll_timeout;
				start_poll_cycles = k_cycle_get_32();
				printk("start_poll_cycles : %u\n", start_poll_cycles);
			} else if (poll_expired(start_poll_cycles,
					      &local_poll_timeout)) {
				printk("timeout expired\n");
				break;
			}

			printk("check done / timeout not yet\n");

			/* Subtract the time invested from the sleep period
			 * in order to compensate for the time invested
			 * in debouncing a key
			 */
			current_cycles = k_cycle_get_32();
			cycles_diff = current_cycles - start_period_cycles;
			wait_period =  kbd_data.poll_period -
				CLOCK_HW_CYCLES_TO_US(cycles_diff);

			/* Override wait_period in case it is less than 1 ms */
			if (wait_period < MSEC)
				wait_period = MSEC;

			/* wait period results in a larger number when
			 * current cycles counter wrap. In this case, the
			 * whole poll period is used
			 */
			if (wait_period > kbd_data.poll_period) {
				LOG_DBG("wait_period : %u", wait_period);
				printk("sleep wait time : %u\n", wait_period);

				wait_period = kbd_data.poll_period;
			}
			printk("task sleep\n");

			/* Allow other threads to run while we sleep */
			k_usleep(wait_period);
		}
	}
}

/*
 * Initialize the raw keyboard interface.
 */
static int kscan_it8xxx2_init(const struct device *dev)
{
	unsigned int key;
	uint8_t x, y;

	ARG_UNUSED(dev);

	/* Ensure top-level interrupt is disabled */
	keyboard_raw_enable_interrupt(0); /*in irq_en/dis not read and clear, directly en/dis. Nest INT?*/

	/*
	 * bit2, Setting 1 enables the internal pull-up of the KSO[15:0] pins.
	 * To pull up KSO[17:16], set the GPCR registers of their
	 * corresponding GPIO ports.
	 * bit0, Setting 1 enables the open-drain mode of the KSO[17:0] pins.
	 */
	KSOCTRL = 0x05;
	//GPCRC3 = 0x04; /*pull up KSO[16]*/
	//GPCRC5 = 0x04; /*pull up KSO[17]*/
	/* bit2, 1 enables the internal pull-up of the KSI[7:0] pins. */
	KSICTRL = 0x04;
#ifdef CONFIG_KEYBOARD_COL2_INVERTED
	/* KSO[2] is high, others are low. */
	KSOL = BIT(2);
	/* Enable KSO2's push-pull */
	KSOLGCTRL |= BIT(2);
	KSOLGOEN |= BIT(2);
#else
	/* KSO[7:0] pins low. */
	KSOL = 0x00;
#endif
	/* critical section with interrupts off */
	key = irq_lock(); //int_mask = read_clear_int_mask();
	/*
	 * KSO[COLS_MAX:8] pins low.
	 * NOTE: KSO[15:8] pins can part be enabled for keyboard function and
	 *		 rest be configured as GPIO output mode. In this case that we
	 *		 disable the ISR in critical section to avoid race condition.
	 */
	KSOH1 &= ~KSOH_PIN_MASK;
	//KSOH2 = 0x00; /*KSO[17:16] output data*/
	/* restore interrupts */
	irq_unlock(key); //set_int_mask(int_mask);
	/* KSI[0-7] falling-edge triggered is selected */
	WUEMR3 = 0xFF;
	/* W/C */
	WUESR3 = 0xFF;
	ite_intc_isr_clear(IT83XX_IRQ_WKINTC); //task_clear_pending_irq(IT83XX_IRQ_WKINTC);
	/* Enable WUC for KSI[0-7] */
	WUENR3 = 0xFF;

	/* Drive the specified column low */
	//drive_keyboard_column(KEYBOARD_COLUMN_DRIVE_NONE); /*chrome init do, here needn't*/

	#if 0
	/* Enable predrive */
	base->KSO_SEL |= BIT(MCHP_KSCAN_KSO_EN_POS);
	base->EXT_CTRL = MCHP_KSCAN_EXT_CTRL_PREDRV_EN;
	base->KSO_SEL &= ~BIT(MCHP_KSCAN_KSO_EN_POS);
	base->KSI_IEN = MCHP_KSCAN_KSI_IEN_REG_MASK;
	#endif

	/* Time figures (defined in Kconfig.it8xxx2) are transformed from msec to usec */
	kbd_data.deb_time_press = (uint32_t)
		(CONFIG_KSCAN_IT8XXX2_DEBOUNCE_DOWN * MSEC);
	kbd_data.deb_time_rel = (uint32_t)
		(CONFIG_KSCAN_IT8XXX2_DEBOUNCE_UP * MSEC);
	kbd_data.poll_period = (uint32_t)
		(CONFIG_KSCAN_IT8XXX2_POLL_PERIOD * MSEC);
	kbd_data.poll_timeout = 100 * MSEC;

	k_sem_init(&kbd_data.poll_lock, 0, 1); /*create poll lock event*/
	atomic_set(&kbd_data.enable_scan, 1);  /*define CONFIG_ATOMIC_OPERATIONS_BUILTIN? seems needn't*/

	k_thread_create(&kbd_data.thread, kbd_data.thread_stack,
			TASK_STACK_SIZE,
			polling_task, NULL, NULL, NULL,
			K_PRIO_COOP(4), 0, K_NO_WAIT);

	/* Interrupts are enabled in the thread function */
	IRQ_CONNECT(IT83XX_IRQ_WKINTC/*MCHP_KSAN_NVIC*/, 0, keyboard_raw_interrupt/*scan_matrix_xec_isr*/, NULL, 0);

	x = KSOCTRL;
	y = KSICTRL;
	printk("Init KBS KSOCTRL = 0x%x(0x5), KSICTRL = 0x%x(0x4)\n", x, y);

	return 0;
}

static int kscan_it8xxx2_configure(const struct device *dev,
				 kscan_callback_t callback)
{
	/* ? */
	ARG_UNUSED(dev);

	if (!callback) {
		return -EINVAL;
	}

	kbd_data.callback = callback;

	//MCHP_GIRQ_ENSET(MCHP_KSCAN_GIRQ) = BIT(MCHP_KSCAN_GIRQ_POS); //?

	return 0;
}

static int kscan_it8xxx2_inhibit_interface(const struct device *dev)
{
	/* ? */
	ARG_UNUSED(dev);

	atomic_set(&kbd_data.enable_scan, 0);

	return 0;
}

static int kscan_it8xxx2_enable_interface(const struct device *dev)
{
	/* ? */
	ARG_UNUSED(dev);

	atomic_set(&kbd_data.enable_scan, 1);

	return 0;
}

static const struct kscan_driver_api kscan_it8xxx2_driver_api = {
	.config = kscan_it8xxx2_configure,
	.disable_callback = kscan_it8xxx2_inhibit_interface,
	.enable_callback = kscan_it8xxx2_enable_interface,
};

DEVICE_AND_API_INIT(kscan_it8xxx2, DT_PROP(DT_NODELABEL(kscan0), label)/*label is dtsi label="KSCAN"*//*DT_INST_LABEL(0)*/,
		    &kscan_it8xxx2_init,
		    NULL/*load node data from board.dts*/, NULL/*load node config from board.dts*/,
		    POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,
		    &kscan_it8xxx2_driver_api);
