
#ifndef AS5600_H
#define AS5600_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "math.h"

class AS5600 {
public:
    AS5600(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin);
    ~AS5600();
    
    esp_err_t begin();
    esp_err_t get_raw_angle(uint16_t* raw_angle);
    esp_err_t get_angle_without_track(float* angle);
    esp_err_t get_angle(float* angle);
    void debug_registers();
    esp_err_t scan_i2c_bus();
    esp_err_t read_registers(uint8_t hi_addr, uint8_t lo_addr, uint16_t* data);
  
private:
    static const uint8_t _AMS5600_ADDRESS = 0x36;
    static const uint8_t _RAW_ANG_HI = 0x0C;
    static const uint8_t _RAW_ANG_LO = 0x0D;
    static const float _RAD_PER_STEP;
    static const float _FULL_CIRCLE;
    
    i2c_port_t _i2c_num;
    gpio_num_t _sda_pin;
    gpio_num_t _scl_pin;
    i2c_master_bus_handle_t _bus_handle;
    i2c_master_dev_handle_t _device_handle;
    
    int32_t _full_rotations;
    float _angle_prev;
    
    esp_err_t _read_two_bytes(uint8_t hi_addr, uint8_t lo_addr, uint16_t* data);
    esp_err_t _read_register(uint8_t reg_addr, uint8_t* data);
};

#endif // AS5600_H