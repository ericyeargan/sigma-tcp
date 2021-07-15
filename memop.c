#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "adau.h"

extern const struct backend_ops i2c_backend_ops;

int main(int argc, char *argv[])
{
	int arg_offset = 1;	

	const struct backend_ops *backend_ops = &i2c_backend_ops;

	if (backend_ops->open) {
		arg_offset = backend_ops->open(argc, arg_offset, argv);
		if (arg_offset < 0)
			exit(1);
	}

	if (argc < arg_offset + 1)
	{
		printf("Usage: %s <i2c params> [op]\n", argv[0]);
		exit(1);
	}

	char* op = argv[arg_offset];

	if (strcmp(op, "write_float") == 0) {
		if (argc < arg_offset + 2)
		{
			printf("Usage: %s <i2c params> write_float [addr] [value]\n", argv[0]);
			exit(1);
		}

		char* addr_str = argv[arg_offset + 1];
		char* value_str = argv[arg_offset + 2];

		int addr = (int)strtol(addr_str, NULL, 0);
		float value = strtof(value_str, NULL);

		printf("writing %f to 0x%04X\n", value, addr);

		adau_write_float(backend_ops, addr, value);		
	} else if (strcmp(op, "read_float") == 0) {
		float value;

		if (argc < arg_offset + 2)
		{
			printf("Usage: %s <i2c params> read_float [addr] [value]\n", argv[0]);
			exit(1);
		}

		char* addr_str = argv[arg_offset + 1];
		int addr = (int)strtol(addr_str, NULL, 0);

		printf("0x%04X: ", addr);

		if (adau_read_float(backend_ops, addr, &value) < 0)
		{
			printf("error (%s)\n", strerror(errno));
			return -1;
		}

		printf("%f\n", value);
	}

    return 0;
}
