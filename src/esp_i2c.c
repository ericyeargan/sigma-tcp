#include "sigma_tcp/adau.h"

#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0                              /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000                             /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define TAG "esp_i2c"

esp_err_t esp_i2c_open(int gpio_num_scl, int gpio_num_sda)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "esp_i2c_open scl: %i, sda: %i", gpio_num_scl, gpio_num_sda);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = gpio_num_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = gpio_num_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    
    if ((ret = i2c_param_config(I2C_MASTER_NUM, &conf)) != ESP_OK) {
        return ret;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

#define ADAU_I2C_ADDR   0x34

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */

int esp_i2c_backend_read(unsigned int addr, unsigned int len, uint8_t *data)
{
    int ret;

    if ((ret = i2c_set_timeout(I2C_MASTER_NUM, 400000)) != ESP_OK) {
        return -1;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ADAU_I2C_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr >> 8, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr & 0xFF, ACK_CHECK_EN));

    ESP_ERROR_CHECK(i2c_master_start(cmd));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ADAU_I2C_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, len, I2C_MASTER_ACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "payload read returned 0x%X", ret);
        return -1;
    }
    return 0;
}

int esp_i2c_backend_write(unsigned int addr, unsigned int len, const uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, ADAU_I2C_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr >> 8, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, addr & 0xFF, ACK_CHECK_DIS);
    i2c_master_write(cmd, data, len, ACK_CHECK_EN);
    
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret != ESP_OK ? -1 : 0;
}

const struct backend_ops esp_i2c_backend_ops = {
	.read = esp_i2c_backend_read,
	.write = esp_i2c_backend_write,
};

#define EEPROM_I2C_ADDR 0x50

int esp_i2c_eeprom_write(unsigned int addr, unsigned int len, uint8_t *data)
{
    int ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, EEPROM_I2C_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr >> 8, ACK_CHECK_DIS);
    i2c_master_write_byte(cmd, addr & 0xFF, ACK_CHECK_DIS);
    i2c_master_write(cmd, data, len, ACK_CHECK_EN);
    
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "EEPROM I2C write returned 0x%X", ret);
        return -1;
    }

    vTaskDelay(20 / portTICK_RATE_MS);

    return 0;
}
