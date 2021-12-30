#ifndef __ADAU_H__
#define __ADAU_H__

#include <stdint.h>

struct backend_ops {
	int (*read)(unsigned int addr, unsigned int len, uint8_t *data);
	int (*write)(unsigned int addr, unsigned int len, const uint8_t *data);
};

int adau_read(const struct backend_ops *backend, unsigned int addr, unsigned int len, uint8_t *data);
int adau_write(const struct backend_ops *backend, unsigned int addr, unsigned int len, const uint8_t *data);
int adau_safeload(const struct backend_ops *backend, unsigned int addr, unsigned int len, const uint8_t *data);

int adau_read_float(const struct backend_ops *backend, unsigned int addr, float *value);
int adau_read_int(const struct backend_ops *backend, unsigned int addr, int *value);

int adau_safeload_int(const struct backend_ops *backend, unsigned int addr, int value);
int adau_safeload_float(const struct backend_ops *backend, unsigned int addr, float value);

int adau_readback_int(const struct backend_ops *backend, unsigned int capture_addr, unsigned int capture_addr_val, int *value);
int adau_readback_float(const struct backend_ops *backend, unsigned int capture_addr, unsigned int capture_addr_val, float *value);

#ifdef SIGMA_TCP_EEPROM_PROGRAM
int adau_write_to_eeprom();
#endif

#endif
