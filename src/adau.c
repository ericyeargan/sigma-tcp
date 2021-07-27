#include "sigma_tcp/adau.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "logging.h"

#include "sigma_tcp/esp_i2c.h"

#define MAX_DEBUG_DATA_BYTES	8

static int format_data(unsigned int len, uint8_t const *data, char* dest_buffer)
{
	int i;
	char* dest = dest_buffer;
	
	for (i = 0; i < MAX_DEBUG_DATA_BYTES; i++) 
	{
		if (i < len)
		{
			dest += sprintf(dest, " 0x%02X", data[i]);
		}			
		else
			break;
	}
	
	if (len > MAX_DEBUG_DATA_BYTES)
	{
		dest += sprintf(dest, " ...");
	}

	return dest - dest_buffer;	
}

int adau_read(const struct backend_ops *backend, unsigned int addr, unsigned int len, uint8_t *data)
{
	int ret;
	char log_buffer[256];
	char *log = log_buffer;

	log += sprintf(log, "adau_read addr: 0x%04X len: 0x%04X ", addr, len);

	if ((ret = backend->read(addr, len, data)) < 0) {
		log += sprintf(log, " failed");
		LOG_ERROR("%s", log_buffer);
	}
	else {
		log += sprintf(log, " data:");
		log += format_data(len, data, log);
		LOG_INFO("%s", log_buffer);
	}
	
	return ret;
}

#define PROGRAM_BASE_ADDRESS 0x0400
#define PROGRAM_WORDS 1024
#define PROGRAM_TOP_ADDRESS (PROGRAM_BASE_ADDRESS + PROGRAM_WORDS)
#define PROGRAM_WORD_LENGTH	5
#define PROGRAM_BYTES (PROGRAM_WORDS * PROGRAM_WORD_LENGTH)

#define PARAMETER_BASE_ADDRESS 0x0000
#define PARAMETER_WORDS 1024
#define PARAMETER_TOP_ADDRESS (PARAMETER_BASE_ADDRESS + PARAMETER_WORDS)
#define PARAMETER_WORD_LENGTH 4
#define PARAMETER_BYTES (PARAMETER_WORDS * PARAMETER_WORD_LENGTH)

#define CONTROL_REG_BASE_ADDRESS 0x081C
#define CONTROL_REG_BYTES 24

static uint8_t program_buffer[PROGRAM_BYTES];
static uint16_t program_length;

static uint8_t parameter_buffer[PARAMETER_BYTES];
static uint16_t parameter_length;

static uint8_t control_reg_buffer[CONTROL_REG_BYTES];
static uint8_t control_reg_length;

static size_t program_buffer_offset(unsigned int addr)
{
	assert(addr >= PROGRAM_BASE_ADDRESS);
	unsigned int program_offset = addr - PROGRAM_BASE_ADDRESS;
	return program_offset * PROGRAM_WORD_LENGTH;
}

static size_t parameter_buffer_offset(unsigned int addr)
{
	size_t parameter_offset = addr - PARAMETER_BASE_ADDRESS;
	return parameter_offset * PARAMETER_WORD_LENGTH;
}

int adau_write(const struct backend_ops *backend, unsigned int addr, unsigned int len, const uint8_t *data)
{
	int ret;
	char log_buffer[256];
	char *log = log_buffer;

	log += sprintf(log, "adau_write addr: 0x%04X len: 0x%04X data:", addr, len);
	log += format_data(len, data, log);

	if (addr < PARAMETER_TOP_ADDRESS)
	{
		size_t parameter_data_len = len;
		size_t buffer_offset = parameter_buffer_offset(addr);
		LOG_INFO("copying parameter data to parameter buffer ofset 0x%04X", buffer_offset);
		if (buffer_offset + parameter_data_len > PARAMETER_BYTES)
		{
			parameter_data_len = PARAMETER_BYTES - buffer_offset;
			LOG_ERROR("data exceed parameter buffer - resizing to %i bytes", parameter_data_len);
		}
		memcpy(parameter_buffer + buffer_offset, data, parameter_data_len);
		parameter_length = buffer_offset + parameter_data_len;
	}
	else if (addr >= PROGRAM_BASE_ADDRESS && addr < PROGRAM_TOP_ADDRESS)
	{
		size_t program_data_len = len;
		size_t buffer_offset = program_buffer_offset(addr);  
		LOG_INFO("copying program data to program buffer ofset 0x%04X", buffer_offset);
		if (buffer_offset + program_data_len > PROGRAM_BYTES)
		{
			program_data_len = PROGRAM_BYTES - buffer_offset;
			LOG_ERROR("data exceeds program buffer - resizing to %i bytes", program_data_len);
		}
		memcpy(program_buffer + buffer_offset, data, program_data_len);
		program_length = buffer_offset + program_data_len;
	}
	else if (addr == CONTROL_REG_BASE_ADDRESS)
	{
		if (len == CONTROL_REG_BYTES)
		{
			LOG_INFO("copying control register data to buffer");
			memcpy(control_reg_buffer, data, len);
			control_reg_length = len;
		}
		else if (len != 2)
		{
			LOG_ERROR("unexpected control register data length: %i", len);
		}
	}
	else
	{
		LOG_ERROR("unexpected data");
	}

	if ((ret = backend->write(addr, len, data)) < 0)
	{
		log += sprintf(log, " failed");
		LOG_ERROR("%s", log_buffer);
	}
	else {
		LOG_INFO("%s", log_buffer);
	}

	return ret;
}

#define ADI_5_23_MAX_VALUE 16
#define ADI_5_23_MIN_VALUE -16

#define ADI_5_23_FULL_SCALE_INT	0x800000
#define ADI_5_19_FULL_SCALE_INT 0x080000 // 524288

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

static float adi_5_19_to_float(int32_t value)
{
	return (float)value / ADI_5_19_FULL_SCALE_INT;
}

#define PARAMETER_RAM_END_ADDR 	1024

int adau_read_float(const struct backend_ops *backend, unsigned int addr, float *value)
{
	int ret;
	uint8_t data[4];
	if ((ret = adau_read(backend, addr, sizeof(data), data)) < 0)
		return ret;
	int32_t int_value = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	*value = adi_5_23_to_float(int_value);
	return 0;
}

int adau_readback_float(const struct backend_ops *backend, unsigned int capture_addr, unsigned int capture_addr_val, float *value)
{
	int ret;
	uint8_t write_buf[2] = {capture_addr_val >> 8, capture_addr_val};
	uint8_t data[3];
	int32_t int_value;

	if ((ret = adau_write(backend, capture_addr, sizeof(write_buf), write_buf)) < 0)
		return ret;
	if ((ret = adau_read(backend, capture_addr, sizeof(data), data)))
		return ret;

	int_value = data[0] << 16 | data[1] << 8 | data[2];
	*value = adi_5_19_to_float(int_value);

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
	return adau_write(backend, addr, sizeof(data), data);
}

#define EEPROM_PAGE_SIZE	32
#define EEPROM_BYTES		0x2000

#define EEPROM_CMD_END		0x00
#define EEPROM_CMD_LOAD		0x01
#define EEPROM_CMD_NOOP		0x03

static uint8_t eeprom_page_buffer[EEPROM_PAGE_SIZE];

static void eeprom_clear_page(uint8_t val)
{
	memset(eeprom_page_buffer, val, EEPROM_PAGE_SIZE);
}

static int eeprom_write_page(uint16_t page_index)
{
	uint16_t eeprom_addr = page_index * EEPROM_PAGE_SIZE;
	LOG_INFO("writing EEPROM page %i addr 0x%04X", page_index, eeprom_addr);
	if (eeprom_addr >= EEPROM_BYTES)
	{
		LOG_ERROR("invalid write address");
		return -1;
	}
	return esp_i2c_eeprom_write(eeprom_addr, EEPROM_PAGE_SIZE, eeprom_page_buffer);
}

/* Return number of pages written or < 0 on error */
static int eeprom_write_load_command(uint16_t const eeprom_page_index, uint16_t load_addr, uint8_t const *data, size_t len)
{
	int ret;
	size_t page_off = 0;
	size_t page_count = 0;
	const size_t command_len = len + 3;

	LOG_INFO("writing load command: page = %i load addr = 0x%04X len = %i", eeprom_page_index, load_addr, len);

	eeprom_clear_page(EEPROM_CMD_NOOP);

	eeprom_page_buffer[page_off++] = EEPROM_CMD_LOAD;
	eeprom_page_buffer[page_off++] = command_len >> 8;
	eeprom_page_buffer[page_off++] = command_len & 0xFF;
	eeprom_page_buffer[page_off++] = 0; // chip ID
	eeprom_page_buffer[page_off++] = load_addr >> 8;
	eeprom_page_buffer[page_off++] = load_addr & 0xFF;

	for (size_t i = 0; i < len; i++)
	{
		eeprom_page_buffer[page_off++] = data[i];
		if (page_off == EEPROM_PAGE_SIZE)
		{
			if ((ret = eeprom_write_page(eeprom_page_index + page_count)) < 0)
			{
				return ret;
			}
			page_count++;
			eeprom_clear_page(EEPROM_CMD_NOOP);
			page_off = 0;
		}
	}

	if (page_off > 0)
	{
		if ((ret = eeprom_write_page(eeprom_page_index + page_count)) < 0)
		{
			return ret;
		}
		page_count++;
	}

	return page_count;
}

int adau_eeprom_test_pattern()
{
	int ret;
	size_t page_count = 256;
	for (size_t page_index = 0; page_index < page_count; page_index++)
	{
		uint16_t page_address = page_index * EEPROM_PAGE_SIZE;
		eeprom_page_buffer[0] = page_address >> 8;
		eeprom_page_buffer[1] = page_address & 0xFF;
		for (size_t page_offset = 2; page_offset < EEPROM_PAGE_SIZE; page_offset++)
		{
			eeprom_page_buffer[page_offset] = page_offset;
		}
		if ((ret = eeprom_write_page(page_index)) < 0)
		{
			return ret;
		}
	}
	return 0;
}

#define CONTROL_REG_ADDR 0x081C
#define EEPROM_MAX_PARAMETER_COUNT	32	/* we don't have enough space for all parameters */

int adau_write_to_eeprom()
{
	static uint8_t CONTROL_REG_MUTE[] = {0x00, 0x18};
	static uint8_t CONTROL_REG_INIT[] = {0x00, 0x1C};

	int ret;
	uint16_t page_index = 0;
	
	if ((ret = eeprom_write_load_command(page_index, CONTROL_REG_ADDR, CONTROL_REG_MUTE, sizeof(CONTROL_REG_MUTE))) < 0)
		return ret;
	page_index += ret;

	if (parameter_length > EEPROM_MAX_PARAMETER_COUNT)
		parameter_length = EEPROM_MAX_PARAMETER_COUNT;

	LOG_INFO("writing %i bytes of parameter data", parameter_length);
	if ((ret = eeprom_write_load_command(page_index, PARAMETER_BASE_ADDRESS, parameter_buffer, parameter_length)) < 0)
		return ret;
	page_index += ret;

	LOG_INFO("writing %i bytes of program data", program_length);
	if ((ret = eeprom_write_load_command(page_index, PROGRAM_BASE_ADDRESS, program_buffer, program_length)) < 0)
		return ret;
	page_index += ret;

	LOG_INFO("writing %i bytes of register data", control_reg_length);
	if ((ret = eeprom_write_load_command(page_index, CONTROL_REG_BASE_ADDRESS, control_reg_buffer, control_reg_length)) < 0)
		return ret;
	page_index += ret;

	if ((ret = eeprom_write_load_command(page_index, CONTROL_REG_ADDR, CONTROL_REG_INIT, sizeof(CONTROL_REG_INIT))) < 0)
		return ret;
	page_index += ret;	

	eeprom_clear_page(EEPROM_CMD_END);
	if ((ret = eeprom_write_page(page_index)) < 0)
		return ret;

	return 0;
}
