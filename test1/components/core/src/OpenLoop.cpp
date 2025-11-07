#include "OpenLoop.h"

static float shaft_angle = 0.0f;          
static int64_t open_loop_timestamp = 0;   
static float zero_electric_angle = 0.0f;  
static float Ualpha = 0.0f, Ubeta = 0.0f;
static float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
static float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;


/**
 * @brief 数值限制函数（将值约束在 [low, high] 范围内）
 */
static float _constrain(float amt, float low, float high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

/**
 * @brief 机械角度转电角度（电角度 = 机械角度 × 极对数 × 2π/2π = 机械角度 × 极对数）
 * @param shaft_angle 机械角度（rad）
 * @param pole_pairs 电机极对数
 * @return 电角度（rad）
 */
static float _mech2ElecAngle(float shaft_angle, int pole_pairs) {
    return shaft_angle * pole_pairs;
}

/**
 * @brief 角度归一化（将角度约束在 [0, 2π] 范围内）
 * @param angle 原始角度（rad）
 * @return 归一化后的角度（rad）
 */
static float _normalizeAngle(float angle) {
    float normalized = fmod(angle, 2 * PI);
    return (normalized >= 0) ? normalized : (normalized + 2 * PI);
}

/**
 * @brief 设置三相PWM占空比（内部调用，不对外暴露）
 * @param Ua/A相电压 Ub/B相电压 Uc/C相电压（V）
 */
static void _setPWM(float Ua, float Ub, float Uc) {
    // 电压转占空比（占空比 = 相电压 / 电源电压，约束在 0~1）
    dc_a = _constrain(Ua / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    dc_b = _constrain(Ub / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    dc_c = _constrain(Uc / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);

    // 设置LEDC占空比（8位分辨率：0~255）
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, (uint8_t)(dc_a * 255));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, (uint8_t)(dc_b * 255));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, (uint8_t)(dc_c * 255));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
}

/**
 * @brief 电压变换（帕克逆变换 + 克拉克逆变换，内部调用）
 * @param Uq Q轴电压（转矩分量） Ud D轴电压（磁通分量，开环时为0）
 * @param angle_el 电角度（rad）
 */
static void _setPhaseVoltage(float Uq, float Ud, float angle_el) {
    // 电角度叠加零点偏移并归一化
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);
    
    // 帕克逆变换（将DQ轴电压转为αβ轴电压，开环时Ud=0）
    Ualpha = -Uq * sin(angle_el);
    Ubeta = Uq * cos(angle_el);

    // 克拉克逆变换（将αβ轴电压转为三相电压，叠加中点电压抵消负电压）
    Ua = Ualpha + VOLTAGE_POWER_SUPPLY / 2;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + VOLTAGE_POWER_SUPPLY / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + VOLTAGE_POWER_SUPPLY / 2;

    // 输出PWM
    _setPWM(Ua, Ub, Uc);
}


void OpenLoop_LEDCPWM_Init(void) {
    // 1. 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_chan_a = {
        .gpio_num = LEDC_OUTPUT_IO_0,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,  // 初始占空比0
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_chan_a));

    ledc_channel_config_t ledc_chan_b = {
        .gpio_num = LEDC_OUTPUT_IO_1,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_chan_b));

    ledc_channel_config_t ledc_chan_c = {
        .gpio_num = LEDC_OUTPUT_IO_2,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_chan_c));

    // 初始化时间戳（首次运行用当前时间）
    open_loop_timestamp = esp_timer_get_time();
}

void OpenLoop_DriverEnable_Init(void) {
    // 配置驱动器使能引脚为输出
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_DRIVER_ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(GPIO_DRIVER_ENABLE_PIN, 1);
}

float OpenLoop_VelocityControl(float target_velocity) {
    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;

    if (Ts <= 0 || Ts > 0.5f) {
        Ts = 1e-3f;  
    }

    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

    float Uq = VOLTAGE_POWER_SUPPLY / 3.0f;

    float elec_angle = _mech2ElecAngle(shaft_angle, MOTOR_POLE_PAIRS);
    _setPhaseVoltage(Uq, 0.0f, elec_angle);

    open_loop_timestamp = now_us;

    return Uq;  
}