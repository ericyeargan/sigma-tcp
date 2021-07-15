#include "adau.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>

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

#define ADI_5_23_MAX_VALUE 16
#define ADI_5_23_MIN_VALUE -16

#define ADI_5_23_FULL_SCALE_INT	8388608

static int32_t float_to_5_23(float value)
{
	if (value > ADI_5_23_MAX_VALUE)
		value = ADI_5_23_MAX_VALUE;
	else if (value < ADI_5_23_MIN_VALUE)
		value = ADI_5_23_MIN_VALUE;

	return (int32_t)(value * ADI_5_23_FULL_SCALE_INT);
}

static float adi_5_23_to_float(int32_t value)
{
	return (float)value / ADI_5_23_FULL_SCALE_INT;
}

#define PARAMETER_RAM_END_ADDR 	1024

int adau_read_float(const struct backend_ops *backend, unsigned int addr, float *value)
{
	int ret;
	uint8_t data[4];
	if ((ret = backend->read(addr, sizeof(data), data)) < 0)
		return ret;
	int32_t int_value = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	*value = adi_5_23_to_float(int_value);
	return 0;
}

int adau_write_float(const struct backend_ops *backend, unsigned int addr, float value)
{
	uint8_t data[4];
	int32_t value_5_23 = float_to_5_23(value);
	data[0] = (value_5_23 >> 24) & 0xFF;
	data[1] = (value_5_23 >> 16) & 0xFF;
	data[2] = (value_5_23 >> 8) & 0xFF;
	data[3] = value_5_23 & 0xFF;
	return backend->write(addr, sizeof(data), data);
}
