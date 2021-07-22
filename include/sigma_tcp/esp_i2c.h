#pragma once

#include "esp_err.h"

#include "sigma_tcp/adau.h"

esp_err_t esp_i2c_open(int gpio_num_scl, int gpio_num_sda);

int esp_i2c_backend_read(unsigned int addr, unsigned int len, uint8_t *data);

int esp_i2c_backend_write(unsigned int addr, unsigned int len, const uint8_t *data);

int esp_i2c_eeprom_write(unsigned int addr, unsigned int len, uint8_t *data);

extern const struct backend_ops esp_i2c_backend_ops;
