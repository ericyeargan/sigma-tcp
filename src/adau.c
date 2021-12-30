#include "sigma_tcp/adau.h"

#include <errno.h>
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
		LOG_DEBUG("%s", log_buffer);
	}
	
	return ret;
}

#define PROGRAM_BASE_ADDRESS 0x0400
#define PROGRAM_WORDS 1024
#define PROGRAM_TOP_ADDRESS (PROGRAM_BASE_ADDRESS + PROGRAM_WORDS)
#define PROGRAM_WORD_LENGTH	5
#define PROGRAM_BYTES (PROGRAM_WORDS * PROGRAM_WORD_LENGTH)

#ifdef SIGMA_TCP_EEPROM_PROGRAM

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

#endif /* SIGMA_TCP_EEPROM_PROGRAM */

int adau_write(const struct backend_ops *backend, unsigned int addr, unsigned int len, const uint8_t *data)
{
	int ret;
	char log_buffer[256];
	char *log = log_buffer;
	int program_data = 0;

	log += sprintf(log, "adau_write ");

	if (addr >= PROGRAM_BASE_ADDRESS && addr < PROGRAM_TOP_ADDRESS)
	{
		program_data = 1;
		log += sprintf(log, "program ");
	}

	log += sprintf(log, "addr: 0x%04X len: 0x%04X data:", addr, len);
	log += format_data(len, data, log);

#ifdef SIGMA_TCP_EEPROM_PROGRAM
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
	else if (program_data)
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
#endif /* SIGMA_TCP_EEPROM_PROGRAM */

	if ((ret = backend->write(addr, len, data)) < 0)
	{
		log += sprintf(log, " failed");
		LOG_ERROR("%s", log_buffer);
	}
	else {
		if (program_data || addr != 2074)
		{
			LOG_INFO("%s", log_buffer);
		}
		else
		{
            LOG_DEBUG("%s", log_buffer);
		}
	}

	return ret;
}

static int adau_read_ushort(const struct backend_ops *backend, unsigned int addr, uint16_t *value)
{
	int ret;
	uint8_t data[2];
	if ((ret = adau_read(backend, addr, sizeof(data), data)) < 0)
		return ret;
	*value = data[0] << 8 | data[1];
	return 0;
}

static int adau_write_ushort(const struct backend_ops *backend, unsigned int addr, uint16_t value)
{
	uint8_t data[2];
	data[0] = (value >> 8) & 0xFF;
	data[1] = value & 0xFF;
	return adau_write(backend, addr, sizeof(data), data);
}

#define SAFELOAD_REG_COUNT 5
#define SAFELOAD_ADDR_REGISTER_BASE_ADDR 2069 // 0x815
#define SAFELOAD_ADDR_MASK 0x03FF // the safeload address registers seem to allow program RAM addresses (up to 0xFFF) but we restrict to parameter RAM
#define SAFELOAD_DATA_REGISTER_BASE_ADDR 2064 // 0x810
#define SAFELOAD_DATA_LEN 4

#define CORE_REG_ADDR 0x81C
#define CORE_REG_IST_MASK 0x20

/* mute:
I (68772) sigma_tcp: processing write command (0x09) safeload: 1, IC: 1 packet_len: 14 received 14
I (68782) sigma_tcp: adau_write addr: 0x000B len: 0x0004 data: 0x00 0x00 0x00 0x00
I (68812) sigma_tcp: processing write command (0x09) safeload: 1, IC: 1 packet_len: 14 received 14
I (68822) sigma_tcp: adau_write addr: 0x000C len: 0x0004 data: 0x00 0x00 0x20 0x00
*/

/* level:
I (154882) sigma_tcp: processing write command (0x09) safeload: 1, IC: 1 packet_len: 18 received 18
I (154882) sigma_tcp: adau_write addr: 0x0000 len: 0x0008 data: 0x00 0x00 0x18 0xDB 0x00 0x00 0x08 0x00
*/

/* biquad coeff (20 bytes - b0 coeff at addr):
I (228582) sigma_tcp: processing write command (0x09) safeload: 1, IC: 1 packet_len: 30 received 30
I (228592) sigma_tcp: adau_write addr: 0x0011 len: 0x0014 data: 0x00 0x7F 0xA1 0x79 0xFF 0x01 0x45 0x69 ...
*/

int adau_safeload(const struct backend_ops *backend, unsigned int addr, const unsigned int len, const uint8_t *data)
{
    // https://ez.analog.com/dsp/sigmadsp/w/documents/5182/implementing-safeload-writes-on-the-adau1701

	int ret;
	uint8_t addr_buf[2];
    uint8_t data_buf[5];

    char log_buffer[256];
    char *log = log_buffer;

    if (len > SAFELOAD_REG_COUNT * SAFELOAD_DATA_LEN)
    {
        log += sprintf(log, " failed - length exceeds safeload capacity");
        ret = EINVAL;
        goto error;
    }

	if (len % SAFELOAD_DATA_LEN)
    {
        log += sprintf(log, " failed - length must be a multiple of 4");
        ret = EINVAL;
        goto error;
    }

	uint8_t load_count = len / SAFELOAD_DATA_LEN;

    log += sprintf(log, "adau_safeload addr: 0x%04X len: 0x%04X count: %d data:", addr, len, load_count);
    log += format_data(len, data, log);

    unsigned int data_reg_address = SAFELOAD_DATA_REGISTER_BASE_ADDR;
    unsigned int addr_reg_address = SAFELOAD_ADDR_REGISTER_BASE_ADDR;

    for (uint8_t i = 0; i < load_count; i++)
	{
		if ((addr & SAFELOAD_ADDR_MASK) != addr)
        {
            log += sprintf(log, " failed - invalid addr");
            ret = EINVAL;
            goto error;
        }

        data_buf[0] = 0;
        memcpy(data_buf + 1, data, SAFELOAD_DATA_LEN);

        if ((ret = adau_write(backend, data_reg_address, sizeof(data_buf), data_buf)) != 0)
        {
            log += sprintf(log, " failed - data write");
            goto error;
        }

        data += SAFELOAD_DATA_LEN;
        data_reg_address++;

		addr_buf[0] = addr >> 8;
		addr_buf[1] = addr;

        if ((ret = adau_write(backend, addr_reg_address, 2, addr_buf)) != 0)
        {
            log += sprintf(log, " failed - address write");
            goto error;
        }

        addr++;
        addr_reg_address++;
	}

	uint16_t core_reg_val;
	if ((ret = adau_read_ushort(backend, CORE_REG_ADDR, &core_reg_val)) != 0)
    {
        log += sprintf(log, " failed - core reg get");
        goto error;
    }

    if (core_reg_val & CORE_REG_IST_MASK)
    {
        log += sprintf(log, " failed - IST already set");
        goto error;
    }

	core_reg_val |= CORE_REG_IST_MASK;
	
	if ((ret = adau_write_ushort(backend, CORE_REG_ADDR, core_reg_val)) != 0)
    {
        log += sprintf(log, " failed - IST set");
        goto error;
    }

    LOG_INFO("%s", log_buffer);
    return 0;

error:
    (void)log;
    LOG_ERROR("%s", log_buffer);
    return ret;
}

#define ADI_5_23_MAX_VALUE (16)
#define ADI_5_23_MIN_VALUE (-16)

#define ADI_5_23_FULL_SCALE_INT	0x800000 // 8388608
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

int adau_read_int(const struct backend_ops *backend, unsigned int addr, int *value)
{
	int ret;
	uint8_t data[4];
	uint32_t uValue;
	if ((ret = adau_read(backend, addr, sizeof(data), data)) < 0)
		return ret;
	uValue = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	uValue &= 0xFFFFFFF; // 28-bits
	if (uValue & 0x8000000)
	{
		// the MSB is set indicating a negative value - sign-extend the 28-bit value to 32-bits, take the two's complement and negate
		*value = (~(uValue | 0xF0000000) + 1) * -1;
	}
	else
	{
		*value = uValue;
	}
	return 0;
}

int adau_safeload_int(const struct backend_ops *backend, unsigned int addr, int value)
{
	uint8_t data[4];
	data[0] = (value >> 24) & 0xFF;
	data[1] = (value >> 16) & 0xFF;
	data[2] = (value >> 8) & 0xFF;
	data[3] = value & 0xFF;
	return adau_safeload(backend, addr, sizeof(data), data);
}

int adau_read_float(const struct backend_ops *backend, unsigned int addr, float *value)
{
	int ret;
	int32_t int_value;
	if ((ret = adau_read_int(backend, addr, &int_value)) < 0)
		return ret;
	*value = adi_5_23_to_float(int_value);
	return 0;
}

int adau_readback_int(const struct backend_ops *backend, unsigned int capture_addr, unsigned int capture_addr_val, int *value)
{
	int ret;
	uint8_t write_buf[2] = {capture_addr_val >> 8, capture_addr_val};
	uint8_t data[3];

	if ((ret = adau_write(backend, capture_addr, sizeof(write_buf), write_buf)) < 0)
		return ret;
	if ((ret = adau_read(backend, capture_addr, sizeof(data), data)))
		return ret;

	*value = data[0] << 16 | data[1] << 8 | data[2];

	return 0;
}

int adau_readback_float(const struct backend_ops *backend, unsigned int capture_addr, unsigned int capture_addr_val, float *value)
{
	int ret;
	int32_t int_value;

	if ((ret = adau_readback_int(backend, capture_addr, capture_addr_val, &int_value)) < 0)
		return ret;

	*value = adi_5_19_to_float(int_value);

	return 0;
}

int adau_safeload_float(const struct backend_ops *backend, unsigned int addr, float value)
{
	int32_t value_5_23 = float_to_5_23(value);
	return adau_safeload_int(backend, addr, value_5_23);
}
