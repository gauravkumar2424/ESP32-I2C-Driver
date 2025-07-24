#include "i2c_low_level.h"
#include <esp_log.h>

static const char *TAG = "MAIN";

void app_main(void) {
    i2c_low_level_config_t config = {
        .sda_pin = 9,
        .scl_pin = 8,
        .clock_speed_hz = 100000,
        .one_byte_addr = true, // For AT24C02 (1-byte address)
        .page_size = 8,        // AT24C02: 8-byte page size
        .write_delay_ms = 5    // AT24C02: 5ms write cycle
    };
    esp_err_t ret = i2c_low_level_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return;
    }

    // I2C scanner
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy = 0;
        if (i2c_low_level_write(addr, &dummy, 0) == ESP_OK) {
            ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
        }
    }

    uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
    ret = i2c_eeprom_page_write(0x50, 0x00, data, sizeof(data));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "EEPROM write successful: %02x %02x %02x %02x",
                 data[0], data[1], data[2], data[3]);
    } else {
        ESP_LOGE(TAG, "EEPROM write failed: %s", esp_err_to_name(ret));
    }

    uint8_t read_data[4] = {0};
    ret = i2c_eeprom_read(0x50, 0x00, read_data, sizeof(read_data));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "EEPROM read successful: %02x %02x %02x %02x",
                 read_data[0], read_data[1], read_data[2], read_data[3]);
    } else {
        ESP_LOGE(TAG, "EEPROM read failed: %s", esp_err_to_name(ret));
    }

    ESP_ERROR_CHECK(i2c_low_level_deinit());
}
