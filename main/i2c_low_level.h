#ifndef I2C_LOW_LEVEL_H
#define I2C_LOW_LEVEL_H

#include <esp_err.h>
#include <stdbool.h>

typedef struct {
    int sda_pin;
    int scl_pin;
    uint32_t clock_speed_hz;
    bool one_byte_addr; // true for 1-byte address (e.g., AT24C02)
    uint8_t page_size;  // e.g., 8 for AT24C02, 64 for AT24C256
    uint8_t write_delay_ms; // Write cycle time
} i2c_low_level_config_t;

esp_err_t i2c_low_level_init(i2c_low_level_config_t *config);
esp_err_t i2c_low_level_deinit(void);
esp_err_t i2c_low_level_write(uint8_t device_addr, uint8_t *data, size_t length);
esp_err_t i2c_low_level_read(uint8_t device_addr, uint8_t *data, size_t length);
esp_err_t i2c_eeprom_write(uint8_t device_addr, uint16_t mem_addr, uint8_t *data, size_t length);
esp_err_t i2c_eeprom_read(uint8_t device_addr, uint16_t mem_addr, uint8_t *data, size_t length);
esp_err_t i2c_eeprom_page_write(uint8_t device_addr, uint16_t mem_addr, uint8_t *data, size_t length);
esp_err_t i2c_reset_bus(void);

#endif // I2C_LOW_LEVEL_H
