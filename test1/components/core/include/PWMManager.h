#pragma once
#include <driver/ledc.h>
#include <math.h>

class PWMManager {
public:
    PWMManager(gpio_num_t pwm_a, gpio_num_t pwm_b, gpio_num_t pwm_c,
               ledc_channel_t ch_a, ledc_channel_t ch_b, ledc_channel_t ch_c);
    
    esp_err_t init(uint32_t freq_hz = 20000, ledc_timer_bit_t resolution = LEDC_TIMER_10_BIT);
    
    // SVPWM生成
    void setSVPWM(float u_alpha, float u_beta, float u_dc);
    
    // 设置占空比
    void setDutyCycles(float duty_a, float duty_b, float duty_c);
    
    // 获取当前扇区
    uint8_t getSector(float u_alpha, float u_beta);
    
    // 获取占空比
    float getDutyA() const { return duty_a; }
    float getDutyB() const { return duty_b; }
    float getDutyC() const { return duty_c; }
    
private:
    gpio_num_t pwm_a, pwm_b, pwm_c;
    ledc_channel_t ch_a, ch_b, ch_c;
    ledc_timer_t timer_num;
    uint32_t pwm_period;
    float duty_a, duty_b, duty_c;
    
    void setPWMDuty(ledc_channel_t channel, float duty);
    void calculateSVPWM(float u_alpha, float u_beta, float u_dc, 
                       float* t_a, float* t_b, float* t_c);
};