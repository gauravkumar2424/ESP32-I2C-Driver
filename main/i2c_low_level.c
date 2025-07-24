#include "i2c_low_level.h"
#include <soc/soc.h>
#include <soc/i2c_struct.h>
#include <soc/i2c_reg.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/system_reg.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <string.h>
#include <soc/interrupts.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static const char *TAG = "I2C_LOW_LEVEL";

#define I2C0_BASE (0x3F413000) // ESP32-S3 I2C0 base address
#define I2C_TRANS_COMPLETE_INT (1 << 4) // Transaction complete interrupt
#define I2C_NACK_INT (1 << 8) // NACK interrupt
#define I2C_TIMEOUT_MS 100 // Transaction timeout in milliseconds
#define I2C_MAX_RETRIES 3 // Number of retries for failed transactions

// Command register bit fields
#define I2C_COMMAND_OP_M        0x7    // Operation code mask (bits 0-2)
#define I2C_COMMAND_OP_S        0      // Operation code shift
#define I2C_COMMAND_BYTE_NUM_M  0xFF   // Byte number mask (bits 5-12)
#define I2C_COMMAND_BYTE_NUM_S  5      // Byte number shift
#define I2C_COMMAND_ACK_EN      (1 << 13) // ACK enable (bit 13)
#define I2C_COMMAND_DONE        (1 << 15) // Command done (bit 15)

// Command opcodes
#define I2C_OP_START 0 // Start condition
#define I2C_OP_WRITE 2 // Write data
#define I2C_OP_READ  3 // Read data
#define I2C_OP_STOP  1 // Stop condition

static volatile i2c_dev_t *i2c = (volatile i2c_dev_t *)I2C0_BASE;
static i2c_low_level_config_t *i2c_config = NULL;
static SemaphoreHandle_t i2c_semaphore = NULL;
static SemaphoreHandle_t i2c_mutex = NULL;
static intr_handle_t i2c_intr_handle = NULL;

static void IRAM_ATTR i2c_isr_handler(void *arg) {
    uint32_t int_status = i2c->int_status.val;
    if (int_status & I2C_TRANS_COMPLETE_INT) {
        i2c->int_clr.val = I2C_TRANS_COMPLETE_INT;
        BaseType_t higher_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(i2c_semaphore, &higher_task_woken);
        if (higher_task_woken) {
            portYIELD_FROM_ISR();
        }
    }
    if (int_status & I2C_NACK_INT) {
        i2c->int_clr.nack_int_clr = 1;
    }
}

static void gpio_set_function(int pin, uint32_t signal_idx, bool is_input) {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf);

    esp_rom_gpio_connect_out_signal(pin, signal_idx, false, false);
    if (is_input) {
        esp_rom_gpio_connect_in_signal(pin, signal_idx, false);
    }
}

esp_err_t i2c_low_level_init(i2c_low_level_config_t *config) {
    if (!config || config->clock_speed_hz == 0 ||
        config->sda_pin < 0 || config->scl_pin < 0) {
        ESP_LOGE(TAG, "Invalid config");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_semaphore = xSemaphoreCreateBinary();
    i2c_mutex = xSemaphoreCreateMutex();
    if (!i2c_semaphore || !i2c_mutex) {
        if (i2c_semaphore) vSemaphoreDelete(i2c_semaphore);
        if (i2c_mutex) vSemaphoreDelete(i2c_mutex);
        return ESP_ERR_NO_MEM;
    }

    i2c_config = config;

    SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT0_CLK_EN);
    CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT0_RST);

    gpio_set_function(config->sda_pin, I2CEXT0_SDA_OUT_IDX, true);
    gpio_set_function(config->scl_pin, I2CEXT0_SCL_OUT_IDX, true);

    i2c->ctr.val = 0;
    i2c->ctr.ms_mode = 1;
    i2c->ctr.sda_force_out = 1;
    i2c->ctr.scl_force_out = 1;
    i2c->ctr.clk_en = 1;

    uint32_t apb_clk = 80 * 1000 * 1000;
    uint32_t clk_div = apb_clk / config->clock_speed_hz / 2;
    if (clk_div < 1) clk_div = 1;

    i2c->clk_conf.sclk_div_num = clk_div - 1;
    i2c->clk_conf.sclk_active = 1;

    i2c->scl_low_period.val = clk_div / 2;
    i2c->scl_high_period.val = clk_div / 2;
    i2c->scl_start_hold.val = clk_div / 4;
    i2c->scl_rstart_setup.val = clk_div / 4;
    i2c->scl_stop_hold.val = clk_div / 4;
    i2c->scl_stop_setup.val = clk_div / 4;
    i2c->sda_hold.val = clk_div / 8;
    i2c->sda_sample.val = clk_div / 8;

    i2c->int_ena.val = I2C_TRANS_COMPLETE_INT | I2C_NACK_INT;

    esp_err_t ret = esp_intr_alloc(ETS_I2C_EXT0_INTR_SOURCE, ESP_INTR_FLAG_IRAM,
                                   i2c_isr_handler, NULL, &i2c_intr_handle);
    if (ret != ESP_OK) {
        vSemaphoreDelete(i2c_semaphore);
        vSemaphoreDelete(i2c_mutex);
        i2c_semaphore = NULL;
        i2c_mutex = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "I2C0 initialized: SDA=%d, SCL=%d, Freq=%d Hz",
             config->sda_pin, config->scl_pin, config->clock_speed_hz);
    return ESP_OK;
}

esp_err_t i2c_reset_bus(void) {
    i2c->ctr.val = 0;
    i2c->ctr.ms_mode = 1;
    i2c->ctr.sda_force_out = 1;
    i2c->ctr.scl_force_out = 1;
    i2c->ctr.clk_en = 1;

    for (int i = 0; i < 10; i++) {
        gpio_set_level(i2c_config->scl_pin, 1);
        esp_rom_delay_us(5);
        gpio_set_level(i2c_config->scl_pin, 0);
        esp_rom_delay_us(5);
    }

    return ESP_OK;
}

esp_err_t i2c_low_level_write(uint8_t device_addr, uint8_t *data, size_t length) {
    if (!data || length == 0 || length > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    int retries = I2C_MAX_RETRIES;
    esp_err_t ret = ESP_ERR_TIMEOUT;
    while (retries-- > 0) {
        i2c->fifo_conf.tx_fifo_rst = 1;
        i2c->fifo_conf.tx_fifo_rst = 0;

        i2c->data.val = (device_addr << 1) | 0;
        for (size_t i = 0; i < length; i++) {
            i2c->data.val = data[i];
        }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverflow"
        i2c->comd[0].val = 0;
        i2c->comd[0].command = (I2C_OP_START << I2C_COMMAND_OP_S) |
                               (0 << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_DONE;

        i2c->comd[1].val = 0;
        i2c->comd[1].command = (I2C_OP_WRITE << I2C_COMMAND_OP_S) |
                               ((1 + length) << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_ACK_EN |
                               I2C_COMMAND_DONE;

        i2c->comd[2].val = 0;
        i2c->comd[2].command = (I2C_OP_STOP << I2C_COMMAND_OP_S) |
                               (0 << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_DONE;
#pragma GCC diagnostic pop

        i2c->ctr.trans_start = 1;

        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
            if (i2c->int_status.nack_int_st) {
                ESP_LOGE(TAG, "I2C NACK error");
                i2c->int_clr.nack_int_clr = 1;
                ret = ESP_ERR_INVALID_RESPONSE;
            } else {
                ret = ESP_OK;
                break;
            }
        }
        ESP_LOGW(TAG, "I2C write failed, retrying (%d left)", retries);
        ESP_ERROR_CHECK(i2c_reset_bus());
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    xSemaphoreGive(i2c_mutex);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed after %d retries", I2C_MAX_RETRIES);
    }
    return ret;
}

esp_err_t i2c_low_level_read(uint8_t device_addr, uint8_t *data, size_t length) {
    if (!data || length == 0 || length > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    int retries = I2C_MAX_RETRIES;
    esp_err_t ret = ESP_ERR_TIMEOUT;
    while (retries-- > 0) {
        i2c->fifo_conf.rx_fifo_rst = 1;
        i2c->fifo_conf.rx_fifo_rst = 0;

        i2c->data.val = (device_addr << 1) | 1;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverflow"
        i2c->comd[0].val = 0;
        i2c->comd[0].command = (I2C_OP_START << I2C_COMMAND_OP_S) |
                               (0 << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_DONE;

        i2c->comd[1].val = 0;
        i2c->comd[1].command = (I2C_OP_WRITE << I2C_COMMAND_OP_S) |
                               (1 << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_ACK_EN |
                               I2C_COMMAND_DONE;

        i2c->comd[2].val = 0;
        i2c->comd[2].command = (I2C_OP_READ << I2C_COMMAND_OP_S) |
                               (length << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_ACK_EN |
                               I2C_COMMAND_DONE;

        i2c->comd[3].val = 0;
        i2c->comd[3].command = (I2C_OP_STOP << I2C_COMMAND_OP_S) |
                               (0 << I2C_COMMAND_BYTE_NUM_S) |
                               I2C_COMMAND_DONE;
#pragma GCC diagnostic pop

        i2c->ctr.trans_start = 1;

        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
            if (i2c->int_status.nack_int_st) {
                ESP_LOGE(TAG, "I2C NACK error");
                i2c->int_clr.nack_int_clr = 1;
                ret = ESP_ERR_INVALID_RESPONSE;
            } else {
                for (size_t i = 0; i < length; i++) {
                    data[i] = i2c->data.val;
                }
                ret = ESP_OK;
                break;
            }
        }
        ESP_LOGW(TAG, "I2C read failed, retrying (%d left)", retries);
        ESP_ERROR_CHECK(i2c_reset_bus());
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    xSemaphoreGive(i2c_mutex);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed after %d retries", I2C_MAX_RETRIES);
    }
    return ret;
}

esp_err_t i2c_eeprom_write(uint8_t device_addr, uint16_t mem_addr, uint8_t *data, size_t length) {
    if (!data || length == 0 || length > i2c_config->page_size) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[2 + length];
    size_t addr_len = i2c_config->one_byte_addr ? 1 : 2;
    if (addr_len == 2) {
        buffer[0] = (mem_addr >> 8) & 0xFF;
        buffer[1] = mem_addr & 0xFF;
        memcpy(&buffer[2], data, length);
    } else {
        buffer[0] = mem_addr & 0xFF;
        memcpy(&buffer[1], data, length);
    }

    esp_err_t ret = i2c_low_level_write(device_addr, buffer, addr_len + length);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(i2c_config->write_delay_ms));
    }
    return ret;
}

esp_err_t i2c_eeprom_page_write(uint8_t device_addr, uint16_t mem_addr, uint8_t *data, size_t length) {
    if (!data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t page_size = i2c_config->page_size;
    size_t offset = 0;
    while (offset < length) {
        size_t chunk = MIN(length - offset, page_size - (mem_addr % page_size));
        esp_err_t ret = i2c_eeprom_write(device_addr, mem_addr + offset, data + offset, chunk);
        if (ret != ESP_OK) return ret;
        offset += chunk;
    }
    return ESP_OK;
}

esp_err_t i2c_eeprom_read(uint8_t device_addr, uint16_t mem_addr, uint8_t *data, size_t length) {
    if (!data || length == 0 || length > i2c_config->page_size) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t addr_buffer[2] = {(mem_addr >> 8) & 0xFF, mem_addr & 0xFF};
    size_t addr_len = i2c_config->one_byte_addr ? 1 : 2;
    esp_err_t ret = i2c_low_level_write(device_addr, addr_buffer, addr_len);
    if (ret != ESP_OK) {
        return ret;
    }

    return i2c_low_level_read(device_addr, data, length);
}

esp_err_t i2c_low_level_deinit(void) {
    if (i2c_intr_handle) {
        esp_intr_free(i2c_intr_handle);
        i2c_intr_handle = NULL;
    }
    if (i2c_semaphore) {
        vSemaphoreDelete(i2c_semaphore);
        i2c_semaphore = NULL;
    }
    if (i2c_mutex) {
        vSemaphoreDelete(i2c_mutex);
        i2c_mutex = NULL;
    }

    CLEAR_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT0_CLK_EN);
    SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT0_RST);
    i2c->ctr.clk_en = 0;

    gpio_set_direction(i2c_config->sda_pin, GPIO_MODE_INPUT);
    gpio_set_direction(i2c_config->scl_pin, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "I2C0 deinitialized");
    return ESP_OK;
}
