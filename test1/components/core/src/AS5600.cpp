#include "AS5600.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 定义类静态常量
const float AS5600::_RAD_PER_STEP = (2 * M_PI) / 4096.0f;
const float AS5600::_FULL_CIRCLE = 2 * M_PI;

AS5600::AS5600(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    _i2c_num = i2c_num;
    _sda_pin = sda_pin;
    _scl_pin = scl_pin;
    _full_rotations = 0;
    _angle_prev = 0.0f;
    _bus_handle = NULL;
    _device_handle = NULL;
}

AS5600::~AS5600() {
    if (_device_handle) {
        i2c_master_bus_rm_device(_device_handle);
    }
    if (_bus_handle) {
        i2c_del_master_bus(_bus_handle);
    }
}

esp_err_t AS5600::begin() {
    // 创建I2C主机总线配置
    i2c_master_bus_config_t bus_config = {
        .i2c_port = _i2c_num,
        .sda_io_num = _sda_pin,
        .scl_io_num = _scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false
        }
    };

    // 初始化I2C总线
    esp_err_t err = i2c_new_master_bus(&bus_config, &_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Create I2C bus failed: 0x%02X", err);
        return err;
    }

    // 配置I2C设备
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _AMS5600_ADDRESS,
        .scl_speed_hz = 100000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false
        }
    };

    err = i2c_master_bus_add_device(_bus_handle, &dev_config, &_device_handle);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Add I2C device failed: 0x%02X", err);
        i2c_del_master_bus(_bus_handle);
        _bus_handle = NULL;
        return err;
    }

    // 给设备一点启动时间
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 读取设备ID
    uint16_t device_id;
    err = _read_two_bytes(0x0F, 0x0E, &device_id);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Read device ID failed: 0x%02X", err);
        return err;
    }

    ESP_LOGI("AS5600", "Device ID: 0x%04X", device_id);

    // 宽松的设备ID检查
    if ((device_id & 0xFFF0) != 0x3630) {
        ESP_LOGW("AS5600", "Device ID abnormal, may not be AS5600 or communication issue");
        
    }

    i2c_device_config_t speed_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _AMS5600_ADDRESS,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false
        }
    };
    
    i2c_master_bus_rm_device(_device_handle);
    err = i2c_master_bus_add_device(_bus_handle, &speed_config, &_device_handle);
    if (err != ESP_OK) {
        ESP_LOGW("AS5600", "Cannot set 400kHz, keep 100kHz");
        err = i2c_master_bus_add_device(_bus_handle, &dev_config, &_device_handle);
        if (err != ESP_OK) {
            ESP_LOGE("AS5600", "Re-add device failed");
            return err;
        }
    }

    ESP_LOGI("AS5600", "Init success");
    return ESP_OK;
}
esp_err_t AS5600::read_registers(uint8_t hi_addr, uint8_t lo_addr, uint16_t* data) {
    return _read_two_bytes(hi_addr, lo_addr, data);
}
esp_err_t AS5600::_read_two_bytes(uint8_t hi_addr, uint8_t lo_addr, uint16_t* data) {
    uint8_t lo_byte, hi_byte;
    esp_err_t err;

    // 读取低位字节
    err = _read_register(lo_addr, &lo_byte);
    if (err != ESP_OK) return err;

    // 读取高位字节
    err = _read_register(hi_addr, &hi_byte);
    if (err != ESP_OK) return err;

    *data = (hi_byte << 8) | lo_byte;
    return ESP_OK;
}

esp_err_t AS5600::_read_register(uint8_t reg_addr, uint8_t* data) {
    return i2c_master_transmit_receive(_device_handle, &reg_addr, 1, data, 1, -1);
}

esp_err_t AS5600::get_raw_angle(uint16_t* raw_angle) {
    return _read_two_bytes(_RAW_ANG_HI, _RAW_ANG_LO, raw_angle);
}

esp_err_t AS5600::get_angle_without_track(float* angle) {
    uint16_t raw_angle;
    esp_err_t err = get_raw_angle(&raw_angle);
    if (err != ESP_OK) return err;
    *angle = raw_angle * _RAD_PER_STEP;
    return ESP_OK;
}

esp_err_t AS5600::get_angle(float* angle) {
    float current_angle;
    esp_err_t err = get_angle_without_track(&current_angle);
    if (err != ESP_OK) return err;

    float angle_diff = current_angle - _angle_prev;

    // 检测圈数变化
    if (fabs(angle_diff) > 0.8f * _FULL_CIRCLE) {
        _full_rotations += (angle_diff < 0) ? 1 : -1;
    }

    _angle_prev = current_angle;
    *angle = (float)_full_rotations * _FULL_CIRCLE + current_angle;
    return ESP_OK;
}

void AS5600::debug_registers() {
    uint8_t registers[] = {0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12};
    const char* names[] = {"ZMCO", "ZPOS_H", "ZPOS_L", "MPOS_H", "MPOS_L", 
                          "MANG_H", "MANG_L", "CONF_H"};
    
    ESP_LOGI("AS5600", "=== Register Debug ===");
    for (int i = 0; i < sizeof(registers); i++) {
        uint8_t value;
        esp_err_t err = _read_register(registers[i], &value);
        if (err == ESP_OK) {
            ESP_LOGI("AS5600", "Register 0x%02X (%s): 0x%02X", registers[i], names[i], value);
        } else {
            ESP_LOGE("AS5600", "Read register 0x%02X failed: 0x%02X", registers[i], err);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

esp_err_t AS5600::scan_i2c_bus() {
    ESP_LOGI("AS5600", "Start I2C bus scan...");
    bool found = false;
    
    for (uint8_t address = 0x08; address < 0x78; address++) {
        i2c_device_config_t scan_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 100000,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = false
            }
        };
        
        i2c_master_dev_handle_t scan_handle;
        esp_err_t err = i2c_master_bus_add_device(_bus_handle, &scan_config, &scan_handle);
        if (err == ESP_OK) {
            ESP_LOGI("AS5600", "Found device: 0x%02X", address);
            i2c_master_bus_rm_device(scan_handle);
            found = true;
            
            // 如果是AS5600地址，尝试读取ID
            if (address == _AMS5600_ADDRESS) {
                uint16_t device_id;
                if (_read_two_bytes(0x0F, 0x0E, &device_id) == ESP_OK) {
                    ESP_LOGI("AS5600", "Device ID at 0x%02X: 0x%04X", address, device_id);
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    if (!found) {
        ESP_LOGE("AS5600", "No I2C devices found");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}