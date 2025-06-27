/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @addtogroup t_i2c_basic
 * @{
 * @defgroup t_i2c_read_write test_i2c_read_write
 * @brief TestPurpose: verify I2C master can read and write
 * @}
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

//define I2C_DEV_NODE	DT_NODELABEL(i2c1) /* get node id by DT_NODELABEL() */

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_0))
#define I2C_DEV_NODE DT_ALIAS(i2c_0)
#elif DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_1))
#define I2C_DEV_NODE DT_ALIAS(i2c_1)      /* get node id by DT_ALIAS() declared in it82xx2_evb.dts */
#elif DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_2))
#define I2C_DEV_NODE DT_ALIAS(i2c_2)
#else
#error "Please set the correct I2C device"
#endif

#define I2C_DEVICE_ADDR 0x40 /* i2c: 7b'0x40 */
#define TCPC_DEVICE_ADDR 0x26 /* TCPC: 7b'0x26 (port0 and port1) */
#define TCPC_port0_vendor_cmd 0xB0
#define TCPC_port1_vendor_cmd 0xB1
#define TIMEOUT_MS 2

uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

#define GY271_HMC_ADDR (0x1E)
#define GY271_QMC_ADDR (0x0D)

#if defined(CONFIG_SENSOR_GY271_QMC)
#define GY271_ADDR GY271_QMC_ADDR
#elif defined(CONFIG_SENSOR_GY271_HMC)
#define GY271_ADDR GY271_HMC_ADDR
#else
#error "No sensor type defined"
#endif

static int test_gy271(void)
{
	unsigned char datas[20];
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
	uint32_t i2c_cfg_tmp;

	if (!device_is_ready(i2c_dev)) {
		TC_PRINT("I2C device is not ready\n");
		return TC_FAIL;
	}

	/* 1. Verify i2c_configure() */
	if (i2c_configure(i2c_dev, i2c_cfg)) {
		TC_PRINT("I2C config failed\n");
		return TC_FAIL;
	}

	/* 2. Verify i2c_get_config() */
	if (i2c_get_config(i2c_dev, &i2c_cfg_tmp)) {
		TC_PRINT("I2C get_config failed\n");
		return TC_FAIL;
	}
	if (i2c_cfg != i2c_cfg_tmp) {
		TC_PRINT("I2C get_config returned invalid config\n");
		return TC_FAIL;
	}

#if 0
	/* 3. verify i2c_write() i2c slave: write data to reg addr 0x0000 and 0x0001 */
	datas[0] = 0x00; /*reg ADDR_H*/
	datas[1] = 0x00; /*reg ADDR_L*/
	datas[2] = 0x11; /*Data*/

	if (i2c_write(i2c_dev, datas, 3, I2C_DEVICE_ADDR)) {
		TC_PRINT("Fail to write reg 0x0000\n");
		return TC_FAIL;
	}

	datas[0] = 0x00; /*reg ADDR_H*/
	datas[1] = 0x01; /*reg ADDR_L*/
	datas[2] = 0x22; /*Data*/
	if (i2c_write(i2c_dev, datas, 3, I2C_DEVICE_ADDR)) {
		TC_PRINT("Fail to write reg 0x0001\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(1));

	/* 4. verify i2c_read() i2c slave: read 2byte data from reg addr 0x0000 */
	datas[0] = 0x00; /*reg ADDR_H*/
	datas[1] = 0x00; /*reg ADDR_L*/
	if (i2c_write(i2c_dev, datas, 2, I2C_DEVICE_ADDR)) {
		TC_PRINT("Fail to write reg 0x0000\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 2, I2C_DEVICE_ADDR)) {
		TC_PRINT("Fail to read 0x0000 data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x (0x1122)\n",
				datas[0], datas[1]);

	k_sleep(K_MSEC(1));
#endif

	/* 5. verify i2c_read() i2c slave: read 2byte data from reg addr 0xffc0 */
	datas[0] = 0xff; /*reg ADDR_H*/
	datas[1] = 0xc0; /*reg ADDR_L*/
	if (i2c_write(i2c_dev, datas, 2, I2C_DEVICE_ADDR)) {
		TC_PRINT("Fail to write reg 0xFFC0\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 2, I2C_DEVICE_ADDR)) {
		TC_PRINT("Fail to read 0xFFC0 data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x (0x5272)\n",
				datas[0], datas[1]);

	k_sleep(K_MSEC(1));

#if 0
	/* 6. verify i2c_read() TCPC port0 slave: read portx vendor cmd (it5272 need enable _ENABLE_I2C_VENDOR_CMD_) */
	datas[0] = TCPC_port0_vendor_cmd; /* read portx vendor cmd */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read portx vendor cmd]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read portx vendor cmd data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x (read portx vendor cmd: disconnect 0x18, connect dongle 0x1F)\n",
				datas[0]);
#endif

#if 1
	/* 7. verify i2c_read() TCPC slave: read VERSION cmd */
	datas[0] = 0x99; /* VERSION */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read VERSION cmd]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 4, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read VERSION cmd data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x (read VERSION 0x03 00 03 80)\n",
				datas[0], datas[1], datas[2], datas[3]);
#endif

#if 1
	/* 8. verify i2c_read() TCPC slave: init CTRL cmd (0x01 PPM_Reset) */
	TC_PRINT("write [CTRL] PPM_Reset\n");
	datas[0] = 0x81; /* CTRL */
	datas[1] = 0x08; /* byte count */
	datas[2] = 0x01; /* 0x01 PPM_Reset */
	datas[3] = 0x00; /* data length */
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	if (i2c_write(i2c_dev, datas, 10, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [CTRL]\n");
		return TC_FAIL;
	}

	datas[0] = 0x83; /* MSGOUT */
	datas[1] = 0x11; /* byte count */
	datas[2] = 0x00;
	datas[3] = 0x00;
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	datas[10] = 0x00;
	datas[11] = 0x00;
	datas[12] = 0x00;
	datas[13] = 0x00;
	datas[14] = 0x00;
	datas[15] = 0x00;
	datas[16] = 0x00;
	datas[17] = 0x00;
	datas[18] = 0x00;
	if (i2c_write(i2c_dev, datas, 19, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [MSGOUT]\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(TIMEOUT_MS));

	datas[0] = 0xBD; /* read INT */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read INT]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read INT data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x (read INT)\n",
				datas[0]);

	datas[0] = 0x80; /* read CCI */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read CCI]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 5, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read CCI data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x %x (read CCI 0x04 00 00 00 08)\n",
				datas[0], datas[1], datas[2], datas[3], datas[4]);

	datas[0] = 0xBC; /* clear INT */
	datas[1] = 0x02; /* UCSI event */
	if (i2c_write(i2c_dev, datas, 2, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [clear INT]\n");
		return TC_FAIL;
	}
#endif

#if 1
	/* 9. verify i2c_read() TCPC slave: init CTRL cmd (0x05 Set_NTF_EN) */
	TC_PRINT("write [CTRL] Set_NTF_EN\n");
	datas[0] = 0x81; /* CTRL */
	datas[1] = 0x08; /* byte count */
	datas[2] = 0x05; /* 0x05 Set_NTF_EN */
	datas[3] = 0x00;
	datas[4] = 0x01; /* cmd complete enable */
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	if (i2c_write(i2c_dev, datas, 10, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [CTRL]\n");
		return TC_FAIL;
	}

	datas[0] = 0x83; /* MSGOUT */
	datas[1] = 0x11; /* byte count */
	datas[2] = 0x00;
	datas[3] = 0x00;
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	datas[10] = 0x00;
	datas[11] = 0x00;
	datas[12] = 0x00;
	datas[13] = 0x00;
	datas[14] = 0x00;
	datas[15] = 0x00;
	datas[16] = 0x00;
	datas[17] = 0x00;
	datas[18] = 0x00;
	if (i2c_write(i2c_dev, datas, 19, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [MSGOUT]\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(TIMEOUT_MS));

	datas[0] = 0xBD; /* read INT */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read INT]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read INT data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x (read INT)\n",
				datas[0]);

	datas[0] = 0x80; /* read CCI */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read CCI]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 5, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read CCI data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x %x (read CCI 0x04 00 00 00 80)\n",
				datas[0], datas[1], datas[2], datas[3], datas[4]);

	datas[0] = 0xBC; /* clear INT */
	datas[1] = 0x02; /* UCSI event */
	if (i2c_write(i2c_dev, datas, 2, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [clear INT]\n");
		return TC_FAIL;
	}
#endif

#if 1
	/* 10. verify i2c_read() TCPC slave: init CTRL cmd (0x04 ACK_CC_CI for Set_NTF_EN) */
	TC_PRINT("write [CTRL] ACK_CC_CI\n");
	datas[0] = 0x81; /* CTRL */
	datas[1] = 0x08; /* byte count */
	datas[2] = 0x04; /* 0x04 ACK_CC_CI */
	datas[3] = 0x00;
	datas[4] = 0x02; /* cmd complete */
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	if (i2c_write(i2c_dev, datas, 10, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [CTRL]\n");
		return TC_FAIL;
	}

	datas[0] = 0x83; /* MSGOUT */
	datas[1] = 0x11; /* byte count */
	datas[2] = 0x00;
	datas[3] = 0x00;
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	datas[10] = 0x00;
	datas[11] = 0x00;
	datas[12] = 0x00;
	datas[13] = 0x00;
	datas[14] = 0x00;
	datas[15] = 0x00;
	datas[16] = 0x00;
	datas[17] = 0x00;
	datas[18] = 0x00;
	if (i2c_write(i2c_dev, datas, 19, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [MSGOUT]\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(TIMEOUT_MS));

	datas[0] = 0xBD; /* read INT */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read INT]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read INT data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x (read INT)\n",
				datas[0]);

	datas[0] = 0x80; /* read CCI */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read CCI]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 5, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read CCI data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x %x (read CCI 0x04 00 00 00 20)\n",
				datas[0], datas[1], datas[2], datas[3], datas[4]);

	datas[0] = 0xBC; /* clear INT */
	datas[1] = 0x02; /* UCSI event */
	if (i2c_write(i2c_dev, datas, 2, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [clear INT]\n");
		return TC_FAIL;
	}
#endif

#if 1
	/* 11. verify i2c_read() TCPC slave: init CTRL cmd (0x06 Get_Capability) */
	TC_PRINT("write [CTRL] Get_Capability\n");
	datas[0] = 0x81; /* CTRL */
	datas[1] = 0x08; /* byte count */
	datas[2] = 0x06; /* 0x06 Get_Capability */
	datas[3] = 0x00;
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	if (i2c_write(i2c_dev, datas, 10, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [CTRL]\n");
		return TC_FAIL;
	}

	datas[0] = 0x83; /* MSGOUT */
	datas[1] = 0x11; /* byte count */
	datas[2] = 0x00;
	datas[3] = 0x00;
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	datas[10] = 0x00;
	datas[11] = 0x00;
	datas[12] = 0x00;
	datas[13] = 0x00;
	datas[14] = 0x00;
	datas[15] = 0x00;
	datas[16] = 0x00;
	datas[17] = 0x00;
	datas[18] = 0x00;
	if (i2c_write(i2c_dev, datas, 19, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [MSGOUT]\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(TIMEOUT_MS));

	datas[0] = 0xBD; /* read INT */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read INT]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read INT data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x (read INT)\n",
				datas[0]);

	datas[0] = 0x80; /* read CCI */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read CCI]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 5, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read CCI data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x %x (read CCI 0x04 00 10 00 80)\n",
				datas[0], datas[1], datas[2], datas[3], datas[4]);

	(void)memset(datas, 0, sizeof(datas));

	datas[0] = 0x82; /* read MSG IN */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read MSG IN]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 17, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read MSG IN data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x(read MSG IN 0x10 45 41 00 00 02 BF 4F 00 02 00 00 00 20 03 20 02)\n",
				datas[0], datas[1], datas[2], datas[3], datas[4], datas[5],
				datas[6], datas[7], datas[8], datas[9], datas[10], datas[11],
				datas[12], datas[13], datas[14], datas[15], datas[16]);

	datas[0] = 0xBC; /* clear INT */
	datas[1] = 0x02; /* UCSI event */
	if (i2c_write(i2c_dev, datas, 2, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [clear INT]\n");
		return TC_FAIL;
	}
#endif

#if 1
	/* 12. verify i2c_read() TCPC slave: init CTRL cmd (0x04 ACK_CC_CI for Get_Capability) */
	TC_PRINT("write [CTRL] ACK_CC_CI\n");
	datas[0] = 0x81; /* CTRL */
	datas[1] = 0x08; /* byte count */
	datas[2] = 0x04; /* 0x04 ACK_CC_CI */
	datas[3] = 0x00;
	datas[4] = 0x02; /* cmd complete */
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	if (i2c_write(i2c_dev, datas, 10, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [CTRL]\n");
		return TC_FAIL;
	}

	datas[0] = 0x83; /* MSGOUT */
	datas[1] = 0x11; /* byte count */
	datas[2] = 0x00;
	datas[3] = 0x00;
	datas[4] = 0x00;
	datas[5] = 0x00;
	datas[6] = 0x00;
	datas[7] = 0x00;
	datas[8] = 0x00;
	datas[9] = 0x00;
	datas[10] = 0x00;
	datas[11] = 0x00;
	datas[12] = 0x00;
	datas[13] = 0x00;
	datas[14] = 0x00;
	datas[15] = 0x00;
	datas[16] = 0x00;
	datas[17] = 0x00;
	datas[18] = 0x00;
	if (i2c_write(i2c_dev, datas, 19, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [MSGOUT]\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(TIMEOUT_MS));

	datas[0] = 0xBD; /* read INT */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read INT]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read INT data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x (read INT)\n",
				datas[0]);

	datas[0] = 0x80; /* read CCI */
	if (i2c_write(i2c_dev, datas, 1, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [read CCI]\n");
		return TC_FAIL;
	}

	(void)memset(datas, 0, sizeof(datas));

	if (i2c_read(i2c_dev, datas, 5, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to read CCI data\n");
		return TC_FAIL;
	}

	TC_PRINT("data: 0x%x %x %x %x %x (read CCI 0x04 00 00 00 20)\n",
				datas[0], datas[1], datas[2], datas[3], datas[4]);

	datas[0] = 0xBC; /* clear INT */
	datas[1] = 0x02; /* UCSI event */
	if (i2c_write(i2c_dev, datas, 2, TCPC_DEVICE_ADDR)) {
		TC_PRINT("Fail to write [clear INT]\n");
		return TC_FAIL;
	}
#endif

	return TC_PASS;
}

static int test_burst_gy271(void)
{
	unsigned char datas[6];
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
	uint32_t i2c_cfg_tmp;

	if (!device_is_ready(i2c_dev)) {
		TC_PRINT("I2C device is not ready\n");
		return TC_FAIL;
	}

	/* 1. verify i2c_configure() */
	if (i2c_configure(i2c_dev, i2c_cfg)) {
		TC_PRINT("I2C config failed\n");
		return TC_FAIL;
	}

	/* 2. Verify i2c_get_config() */
	if (i2c_get_config(i2c_dev, &i2c_cfg_tmp)) {
		TC_PRINT("I2C get_config failed\n");
		return TC_FAIL;
	}
	if (i2c_cfg != i2c_cfg_tmp) {
		TC_PRINT("I2C get_config returned invalid config\n");
		return TC_FAIL;
	}

	datas[0] = 0x00; /*reg ADDR_L*/
	datas[1] = 0x20; /*Data 0*/
	datas[2] = 0x02; /*Data 1*/
	datas[3] = 0x00; /*Data 2*/

	/* 3. verify i2c_burst_write() */
	if (i2c_burst_write(i2c_dev, I2C_DEVICE_ADDR, 0x00 /*reg ADDR_H*/, datas, 4)) {
		TC_PRINT("Fail to write to sensor GY271\n");
		return TC_FAIL;
	}

	k_sleep(K_MSEC(1));

	(void)memset(datas, 0, sizeof(datas));

	/* 4. verify i2c_burst_read() */ //reg ADDR only one byte 0x00, so i2c format can't compatible with two byte ADDR_H ADDR_L, skip burst mode
	if (i2c_burst_read(i2c_dev, I2C_DEVICE_ADDR, 0x00, datas, 4)) {
		TC_PRINT("Fail to fetch sample from sensor GY271\n");
		return TC_FAIL;
	}

	TC_PRINT("axis raw data: %d %d %d %d %d %d\n", datas[0], datas[1], datas[2], datas[3],
		 datas[4], datas[5]);

	return TC_PASS;
}

ZTEST(i2c_gy271, test_i2c_gy271)
{
	zassert_true(test_gy271() == TC_PASS);
}

//ZTEST(i2c_gy271, test_i2c_burst_gy271)
//{
//	zassert_true(test_burst_gy271() == TC_PASS);
//}

ZTEST_SUITE(i2c_gy271, NULL, NULL, NULL, NULL, NULL);
