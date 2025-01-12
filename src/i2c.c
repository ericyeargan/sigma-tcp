/**
 * Copyright (C) 2012 Analog Devices, Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 **/

#include <fcntl.h>
#include <stdio.h>
#include <error.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "adau.h"
#include "logging.h"

static int i2c_fd;
static int i2c_dev_addr;

static int i2c_open(int argc, int arg_offset, char *argv[])
{
	int ret;
	char *endp;

	if (argc < arg_offset + 2) {
		fprintf(stderr, "i2c: Usage %s i2c <i2c-dev> <i2c-addr>\n", argv[0]);
		return -1;
	}

	i2c_fd = open(argv[arg_offset], O_RDWR);
	if (i2c_fd < 0) {
		perror("i2c: Failed to open i2c device");
		return -1;
	}

	i2c_dev_addr = strtoul(argv[arg_offset + 1], &endp, 0);
	if (i2c_dev_addr < 0 || i2c_dev_addr > 255 || *endp != '\0') {
		fprintf(stderr, "i2c: Invalid I2C address: \"%s\"\n", argv[3]);
		return -1;
	}

	ret = ioctl(i2c_fd, I2C_SLAVE_FORCE, i2c_dev_addr);
	if (ret < 0) {
		perror("i2c: Failed to set i2c device address");
		return -1;
	}

	printf("i2c: Initalized for device %s-%x\n", argv[arg_offset], i2c_dev_addr);

	return arg_offset + 2;
}

static int i2c_read(unsigned int addr, unsigned int len, uint8_t *data)
{
	int ret;
	uint8_t addr_buf[2];
	struct i2c_msg msg[2];
	struct i2c_rdwr_ioctl_data xfer = {
		.msgs = msg,
		.nmsgs = 2,
	};

	addr_buf[0] = (addr >> 8) & 0xff;
	addr_buf[1] = addr & 0xff;

	msg[0].addr = i2c_dev_addr;
	msg[0].flags = 0;
	msg[0].buf = addr_buf;
	msg[0].len = 2;
	msg[1].addr = i2c_dev_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = len;

	if ((ret = ioctl(i2c_fd, I2C_RDWR, &xfer)) != 0)
	{
		LOG_ERROR("i2c_read failed (%s)", strerror(errno));
	}
	
	return ret;
}

static int i2c_write(unsigned int addr, unsigned int len, const uint8_t *data)
{
	int ret;
	uint8_t msg_buf[2 + len];
	struct i2c_msg msg[1];
	struct i2c_rdwr_ioctl_data xfer = {
		.msgs = msg,
		.nmsgs = 1,
	};

	msg_buf[0] = addr >> 8;
	msg_buf[1] = addr & 0xff;
	memcpy(msg_buf + 2, data, len);

	msg[0].addr = i2c_dev_addr;
	msg[0].flags = 0;
	msg[0].buf = msg_buf;
	msg[0].len = len + 2;

	if ((ret = ioctl(i2c_fd, I2C_RDWR, &xfer)) != 0)
	{
		LOG_ERROR("i2c_read failed (%s)", strerror(errno));
	}

	return ret;
}

const struct backend_ops i2c_backend_ops = {
	.open = i2c_open,
	.read = i2c_read,
	.write = i2c_write,
};
