#include "adau.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>

#define MAX_DEBUG_DATA_BYTES	8

static void print_data(unsigned int len, uint8_t const *data)
{
	int i;
	for (i = 0; i < MAX_DEBUG_DATA_BYTES; i++) 
	{
		if (i < len)
			printf(" 0x%02X", data[i]);
		else
			break;
	}
	
	if (len > MAX_DEBUG_DATA_BYTES)
	{
		printf(" ...");
	}
}

int adau_read(const struct backend_ops *backend, unsigned int addr, unsigned int len, uint8_t *data)
{
	int ret;

	printf("adau_read addr: 0x%04X len: 0x%04X ", addr, len);

	if ((ret = backend->read(addr, len, data)) < 0) {
		printf(" failed (%s)\n", strerror(errno));
	}
	else {
		printf(" data:");
		print_data(len, data);
		printf("\n");
	}
	return ret;
}

int adau_write(const struct backend_ops *backend, unsigned int addr, unsigned int len, const uint8_t *data)
{
	int ret;

	printf("adau_write addr: 0x%04X len: 0x%04X data:", addr, len);
	print_data(len, data);

	if ((ret = backend->write(addr, len, data)) < 0)
		printf(" failed (%s)\n", strerror(errno));
	else
		printf("\n");

	return ret;
}

int adau_write_float(const struct backend_ops *backend, unsigned int addr, float value)
{
	return 0;
}
