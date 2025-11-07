#include "PWMManager.h"
#include "esp_log.h"

static const char *TAG = "PWM_MANAGER";

PWMManager::PWMManager(gpio_num_t pwm_a, gpio_num_t pwm_b, gpio_num_t pwm_c,
                       ledc_channel_t ch_a, ledc_channel_t ch_b, ledc_channel_t ch_c) {
    this->pwm_a = pwm_a;
    this->pwm_b = pwm_b;
    this->pwm_c = pwm_c;
    this->ch_a = ch_a;
    this->ch_b = ch_b;
    this->ch_c = ch_c;
    this->timer_num = LEDC_TIMER_0;
    this->pwm_period = 0;
    this->duty_a = 0.0f;
    this->duty_b = 0.0f;
    this->duty_c = 0.0f;
}

// 修复PWM通道配置
esp_err_t PWMManager::init(uint32_t freq_hz, ledc_timer_bit_t resolution) {
    // 配置LEDC定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = resolution,
        .timer_num = timer_num,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWM定时器配置失败: 0x%x", err);
        return err;
    }
    
    // 简化的通道配置，移除不支持的字段
    ledc_channel_config_t ch_conf_a = {
        .gpio_num = pwm_a,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch_a,
        .timer_sel = timer_num,
        .duty = 0
    };
    
    ledc_channel_config_t ch_conf_b = {
        .gpio_num = pwm_b,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch_b,
        .timer_sel = timer_num,
        .duty = 0
    };
    
    ledc_channel_config_t ch_conf_c = {
        .gpio_num = pwm_c,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch_c,
        .timer_sel = timer_num,
        .duty = 0
    };
    
    if (ledc_channel_config(&ch_conf_a) != ESP_OK ||
        ledc_channel_config(&ch_conf_b) != ESP_OK ||
        ledc_channel_config(&ch_conf_c) != ESP_OK) {
        ESP_LOGE(TAG, "PWM通道配置失败");
        return ESP_FAIL;
    }
    
    pwm_period = (1 << resolution) - 1;
    ESP_LOGI(TAG, "PWM管理器初始化完成，频率: %lu Hz", freq_hz);
    
    return ESP_OK;
}

uint8_t PWMManager::getSector(float u_alpha, float u_beta) {
    float angle = atan2f(u_beta, u_alpha);
    if (angle < 0) angle += 2 * M_PI;
    
    if (angle < M_PI/3) return 1;
    else if (angle < 2*M_PI/3) return 2;
    else if (angle < M_PI) return 3;
    else if (angle < 4*M_PI/3) return 4;
    else if (angle < 5*M_PI/3) return 5;
    else return 6;
}

void PWMManager::calculateSVPWM(float u_alpha, float u_beta, float u_dc, 
                               float* t_a, float* t_b, float* t_c) {
    // 更精确的SVPWM算法
    float v_ref = sqrtf(u_alpha * u_alpha + u_beta * u_beta);
    
    // 限制电压幅值
    if (v_ref > u_dc / sqrtf(3)) {
        v_ref = u_dc / sqrtf(3);
    }
    
    // 根据扇区计算占空比
    // 这里需要实现完整的SVPWM算法
    // 简化实现：
    *t_a = 0.5f + u_alpha / u_dc;
    *t_b = 0.5f + (-0.5f * u_alpha + 0.866f * u_beta) / u_dc;
    *t_c = 0.5f + (-0.5f * u_alpha - 0.866f * u_beta) / u_dc;
    
    // 限制占空比在安全范围内
    *t_a = fmaxf(0.05f, fminf(0.95f, *t_a));
    *t_b = fmaxf(0.05f, fminf(0.95f, *t_b));
    *t_c = fmaxf(0.05f, fminf(0.95f, *t_c));
}

void PWMManager::setPWMDuty(ledc_channel_t channel, float duty) {
    duty = fmaxf(0.0f, fminf(duty, 0.95f));  // 限制占空比
    uint32_t duty_value = (uint32_t)(duty * pwm_period);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void PWMManager::setDutyCycles(float duty_a, float duty_b, float duty_c) {
    this->duty_a = duty_a;
    this->duty_b = duty_b;
    this->duty_c = duty_c;
    
    setPWMDuty(ch_a, duty_a);
    setPWMDuty(ch_b, duty_b);
    setPWMDuty(ch_c, duty_c);
}

void PWMManager::setSVPWM(float u_alpha, float u_beta, float u_dc) {
    float t_a, t_b, t_c;
    calculateSVPWM(u_alpha, u_beta, u_dc, &t_a, &t_b, &t_c);
    setDutyCycles(t_a, t_b, t_c);
}