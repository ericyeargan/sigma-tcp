#ifndef __ADAU_H__
#define __ADAU_H__

#include <stdint.h>

struct backend_ops {
	int (*open)(int argc, int arg_offset, char *argv[]);
	int (*read)(unsigned int addr, unsigned int len, uint8_t *data);
	int (*write)(unsigned int addr, unsigned int len, const uint8_t *data);
};

int adau_read(const struct backend_ops *backend, unsigned int addr, unsigned int len, uint8_t *data);
int adau_write(const struct backend_ops *backend, unsigned int addr, unsigned int len, const uint8_t *data);

int adau_read_float(const struct backend_ops *backend, unsigned int addr, float *value);
int adau_write_float(const struct backend_ops *backend, unsigned int addr, float value);

#endif
