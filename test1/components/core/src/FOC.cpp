/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "FOC.h"
#include "AS5600.h"
#include "lowpassfilter.h"
#include "pid.h"
#include "SingleShuntSampler.h"
#include "PWMManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

// 速度环PID参数
#define VELOCITY_KP 1.0f
#define VELOCITY_KI 0.05f
#define VELOCITY_KD 0.001f
#define VELOCITY_RAMP 10.0f
#define VELOCITY_LIMIT 5.0f

// 电流环PID参数
#define CURRENT_Q_KP 3.0f
#define CURRENT_Q_KI 0.5f
#define CURRENT_Q_KD 0.0f
#define CURRENT_Q_RAMP 1000.0f
#define CURRENT_Q_LIMIT _voltage_power_supply

#define CURRENT_D_KP 3.0f
#define CURRENT_D_KI 0.5f
#define CURRENT_D_KD 0.0f
#define CURRENT_D_RAMP 1000.0f
#define CURRENT_D_LIMIT _voltage_power_supply

// 速度滤波器时间常数（秒）
#define VELOCITY_FILTER_TF 0.01f

FOC::FOC(gpio_num_t pwmA, gpio_num_t pwmB, gpio_num_t pwmC,
         ledc_channel_t chA, ledc_channel_t chB, ledc_channel_t chC,
         int polePairs, int direction) :
    _pwmA(pwmA), _pwmB(pwmB), _pwmC(pwmC),
    _channelA(chA), _channelB(chB), _channelC(chC),
    _PP(polePairs), _DIR(direction),
    _sensor(I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_5),//根据实际修改
    _velocity_filter(VELOCITY_FILTER_TF),
    _velocity_pid(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, VELOCITY_RAMP, VELOCITY_LIMIT),
    _current_sense(ADC_CHANNEL_0),
    _current_q_pid(CURRENT_Q_KP, CURRENT_Q_KI, CURRENT_Q_KD, CURRENT_Q_RAMP, CURRENT_Q_LIMIT),
    _current_d_pid(CURRENT_D_KP, CURRENT_D_KI, CURRENT_D_KD, CURRENT_D_RAMP, CURRENT_D_LIMIT) {
    
    // 初始化成员变量默认值
    _voltage_power_supply = 0.0f;
    _Ualpha = _Ubeta = _Ua = _Ub = _Uc = 0.0f;
    _zero_electric_angle = 0.0f;
    _motor_target = 0.0f;
    _Ud = 0.0f;
    _receive_index = 0;
    _commaPosition = -1;
    memset(_received_chars, 0, MAX_RECEIVE_LEN);
    
    // 初始化速度控制变量
    _target_velocity = 0.0f;
    _current_velocity = 0.0f;
    _angle_prev = 0.0f;
    _velocity_timestamp_prev = esp_timer_get_time();
    
    // 初始化电流控制变量
    _Iq = 0.0f;
    _Id = 0.0f;
    _Iq_target = 0.0f;
    _Id_target = 0.0f;
    
    // 初始化电流采样变量
    _current_sampling_enabled = false;
    _pwm_center_triggered = false;
    _current_sector = 0;
}

// PWM中心对齐中断回调
void IRAM_ATTR FOC::pwmCenterCallback(void* arg) {
    FOC* foc = (FOC*)arg;
    if (foc->_current_sampling_enabled) {
        foc->_pwm_center_triggered = true;
    }
}

// 电流采样回调
void FOC::sampleCurrents() {
    if (_pwm_center_triggered) {
        _readCurrents();
        _pwm_center_triggered = false;
    }
}

// 启用/禁用电流采样
void FOC::enableCurrentSampling(bool enable) {
    _current_sampling_enabled = enable;
}

// 初始化函数：配置硬件和外设
esp_err_t FOC::begin(float power_supply) {
    _voltage_power_supply = power_supply;
    
    // 初始化电流传感器
    _current_sense.init();
    
    // 配置LEDC定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 30000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    
    if (ledc_timer_config(&timer_conf) != ESP_OK) {
        return ESP_FAIL;
    }

    // 配置PWM通道
    ledc_channel_config_t channel_conf[3] = {
        {
            .gpio_num = _pwmA,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = _channelA,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = 0
        },
        {
            .gpio_num = _pwmB,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = _channelB,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = 0
        },
        {
            .gpio_num = _pwmC,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = _channelC,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = 0
        }
    };

    for (int i = 0; i < 3; i++) {
        if (ledc_channel_config(&channel_conf[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    const esp_timer_create_args_t timer_args = {
        .callback = pwmCenterCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_center_timer",
        .skip_unhandled_events = false
    };
    
    esp_timer_handle_t pwm_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pwm_timer));
    
    const uint64_t half_period_us = 1000000 / (2 * 30000); 
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_timer, half_period_us));

    // 初始化传感器
    BeginSensor();

    // 初始化UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 启用电流采样
    enableCurrentSampling(true);

    return ESP_OK;
}

// 角度归一化到 [0, 2PI]
float FOC::_normalizeAngle(float angle) {
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

// 数值约束
float FOC::_constrain(float value, float min, float max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// 设置PWM占空比
void FOC::_setPwm(float Ua, float Ub, float Uc) {
    // 限制电压范围
    Ua = _constrain(Ua, 0.0f, _voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, _voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, _voltage_power_supply);
    
    // 计算占空比（8位分辨率）
    float dc_a = _constrain(Ua / _voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / _voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / _voltage_power_supply, 0.0f, 1.0f);

    // 输出PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, (uint32_t)(dc_a * 255));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, (uint32_t)(dc_b * 255));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelC, (uint32_t)(dc_c * 255));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelC);
}

// 设置扭矩
void FOC::setTorque(float Uq, float angle_el) {
    Uq = _constrain(Uq, -_voltage_power_supply / 2, _voltage_power_supply / 2);
    _Ud = 0; 
    angle_el = _normalizeAngle(angle_el);
    
    // 帕克逆变换（将dq轴电压转换为αβ轴）
    _Ualpha = -Uq * sin(angle_el);
    _Ubeta = Uq * cos(angle_el);

    // 克拉克逆变换（将αβ轴电压转换为三相电压）
    _Ua = _Ualpha + _voltage_power_supply / 2;
    _Ub = (_2_SQRT3 * _Ubeta - _Ualpha) / 2 + _voltage_power_supply / 2;
    _Uc = (-_Ualpha - _2_SQRT3 * _Ubeta) / 2 + _voltage_power_supply / 2;
    
    _setPwm(_Ua, _Ub, _Uc);
}

// 传感器初始化
void FOC::BeginSensor() {
    if (_sensor.begin() != ESP_OK) {
        const char* err_msg = "AS5600初始化失败";
        uart_write_bytes(UART_NUM_0, err_msg, strlen(err_msg));
        uart_write_bytes(UART_NUM_0, "\r\n", 2);
    }
}

// 获取无圈数跟踪的角度
float FOC::getAngle_Without_track() {
    float angle = 0.0f;
    _sensor.get_angle_without_track(&angle);
    return angle;
}

// 获取带圈数的累计角度
float FOC::getAngle() {
    float angle = 0.0f;
    _sensor.get_angle(&angle);
    return _DIR * angle;
}

// 计算电角度
float FOC::_electricalAngle() {
    float mechanical_angle = getAngle_Without_track();
    return _normalizeAngle((float)(_DIR * _PP) * mechanical_angle - _zero_electric_angle);
}

// 获取电角度（对外接口）
float FOC::getElectricalAngle() {
    return _electricalAngle();
}

// 传感器对齐
void FOC::alignSensor() {
    enableCurrentSampling(false);
    
    // 施加固定电压使电机定位到电角度3PI/2处
    setTorque(3.0f, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 记录当前机械角度对应的零电角度
    _zero_electric_angle = _electricalAngle();
    
    // 停止输出
    setTorque(0.0f, _3PI_2);
    
    // 启用电流采样
    enableCurrentSampling(true);
    
    // 打印校准结果
    char msg[64];
    sprintf(msg, "零电角度校准完成: %f", _zero_electric_angle);
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
    uart_write_bytes(UART_NUM_0, "\r\n", 2);
}

// 读取电流值
// 读取电流值
void FOC::_readCurrents() {
    float angle_el = getElectricalAngle();
    _current_sense.getPhaseCurrents(_current_sector, 
                                   &_current_sense.current_a, 
                                   &_current_sense.current_b, 
                                   &_current_sense.current_c);
}
// 电流环控制
void FOC::_currentControl(float angle_el) {
    // 检查是否有新的电流采样数据
    sampleCurrents();
    
    // Clark变换（将三相电流转换为αβ坐标系）
    float I_alpha = _current_sense.current_a;
    float I_beta = _1_SQRT3 * _current_sense.current_a + _2_SQRT3 * _current_sense.current_b;
    
    // Park变换（将αβ坐标系电流转换为dq坐标系）
    float cos_angle = cos(angle_el);
    float sin_angle = sin(angle_el);
    _Id = I_alpha * cos_angle + I_beta * sin_angle;
    _Iq = I_beta * cos_angle - I_alpha * sin_angle;
    
    // 电流环PID控制
    float Ud = _current_d_pid(_Id_target - _Id);
    float Uq = _current_q_pid(_Iq_target - _Iq);
    
    // 限制输出电压
    Ud = _constrain(Ud, -_voltage_power_supply/2, _voltage_power_supply/2);
    Uq = _constrain(Uq, -_voltage_power_supply/2, _voltage_power_supply/2);
    
    // 帕克逆变换（将dq轴电压转换为αβ轴）
    _Ualpha = Ud * cos_angle - Uq * sin_angle;
    _Ubeta = Ud * sin_angle + Uq * cos_angle;
    
    // 克拉克逆变换（将αβ轴电压转换为三相电压）
    _Ua = _Ualpha + _voltage_power_supply / 2;
    _Ub = (-_Ualpha + _2_SQRT3 * _Ubeta) / 2 + _voltage_power_supply / 2;
    _Uc = (-_Ualpha - _2_SQRT3 * _Ubeta) / 2 + _voltage_power_supply / 2;
    
    // 设置PWM
    _setPwm(_Ua, _Ub, _Uc);
}

// 设置目标Iq电流
void FOC::setTargetCurrentIq(float iq_target) { 
    _Iq_target = iq_target; 
}

// 设置目标Id电流
void FOC::setTargetCurrentId(float id_target) { 
    _Id_target = id_target; 
}

// 执行电流环控制
void FOC::currentLoop() {
    float angle_el = getElectricalAngle();
    _currentControl(angle_el);
}

// 获取当前Iq值
float FOC::getCurrentIq() const { 
    return _Iq; 
}

// 获取当前Id值
float FOC::getCurrentId() const { 
    return _Id; 
}

// 获取速度（弧度/秒）
float FOC::getVelocity() {
    uint64_t now = esp_timer_get_time();
    float dt = (now - _velocity_timestamp_prev) * 1e-6f;
    
    if (dt <= 0 || dt > 0.5f) {
        dt = 1e-3f;
    }
    
    float current_angle = getAngle();
    float velocity = (current_angle - _angle_prev) / dt;
    
    _angle_prev = current_angle;
    _velocity_timestamp_prev = now;
    
    return _velocity_filter(velocity);
}

// 设置目标速度
void FOC::setVelocity(float target_velocity) {
    _target_velocity = target_velocity;
}

// 速度环控制
void FOC::velocityLoop() {
    _current_velocity = getVelocity();
    
    float velocity_error = _target_velocity - _current_velocity;
    _Iq_target = _velocity_pid(velocity_error);
    
    _Iq_target = _constrain(_Iq_target, -_voltage_power_supply/2, _voltage_power_supply/2);
    
    float angle_el = getElectricalAngle();
    _currentControl(angle_el);
}

// 串口接收用户命令
void FOC::serialReceiveUserCommand() {
    uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
    
    if (len > 0) {
        for (int i = 0; i < len; i++) {
            if (data[i] == '\n' || data[i] == '\r') {
                _received_chars[_receive_index] = '\0';
                _commaPosition = -1;
                
                for (int j = 0; j < _receive_index; j++) {
                    if (_received_chars[j] == ',') {
                        _commaPosition = j;
                        break;
                    }
                }
                
                if (_commaPosition != -1) {
                    char target_str[MAX_RECEIVE_LEN];
                    strncpy(target_str, _received_chars, _commaPosition);
                    target_str[_commaPosition] = '\0';
                    _motor_target = atof(target_str);
                    
                    char msg[64];
                    sprintf(msg, "目标值更新: %f", _motor_target);
                    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
                    uart_write_bytes(UART_NUM_0, "\r\n", 2);
                }
                
                _receive_index = 0;
                memset(_received_chars, 0, MAX_RECEIVE_LEN);
            } else if (_receive_index < MAX_RECEIVE_LEN - 1) {
                _received_chars[_receive_index++] = data[i];
            }
        }
    }
}

// 获取电机目标值
float FOC::getMotorTarget() {
    return _motor_target;
}*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "FOC.h"
#include "AS5600.h"
#include "lowpassfilter.h"
#include "pid.h"
#include "SingleShuntSampler.h"  
#include "PWMManager.h"          
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

// 速度环PID参数
#define VELOCITY_KP 1.0f
#define VELOCITY_KI 0.05f
#define VELOCITY_KD 0.001f
#define VELOCITY_RAMP 10.0f
#define VELOCITY_LIMIT 5.0f

// 电流环PID参数
#define CURRENT_Q_KP 3.0f
#define CURRENT_Q_KI 0.5f
#define CURRENT_Q_KD 0.0f
#define CURRENT_Q_RAMP 1000.0f

#define CURRENT_D_KP 3.0f
#define CURRENT_D_KI 0.5f
#define CURRENT_D_KD 0.0f
#define CURRENT_D_RAMP 1000.0f

// 速度滤波器时间常数（秒）
#define VELOCITY_FILTER_TF 0.01f

FOC::FOC(gpio_num_t pwmA, gpio_num_t pwmB, gpio_num_t pwmC,
         ledc_channel_t chA, ledc_channel_t chB, ledc_channel_t chC,
         int polePairs, int direction) :
    _pwmA(pwmA), _pwmB(pwmB), _pwmC(pwmC),
    _channelA(chA), _channelB(chB), _channelC(chC),
    _PP(polePairs), _DIR(direction),
    _sensor(I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_5),
    _velocity_filter(VELOCITY_FILTER_TF),
    _velocity_pid(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, VELOCITY_RAMP, VELOCITY_LIMIT),
    _current_q_pid(CURRENT_Q_KP, CURRENT_Q_KI, CURRENT_Q_KD, CURRENT_Q_RAMP, 0),
    _current_d_pid(CURRENT_D_KP, CURRENT_D_KI, CURRENT_D_KD, CURRENT_D_RAMP, 0)
{
    // 初始化成员变量默认值
    _voltage_power_supply = 0.0f;
    _Ualpha = _Ubeta = _Ua = _Ub = _Uc = 0.0f;
    _zero_electric_angle = 0.0f;
    _motor_target = 0.0f;
    _Ud = 0.0f;
    _receive_index = 0;
    _commaPosition = -1;
    memset(_received_chars, 0, MAX_RECEIVE_LEN);
    
    // 初始化速度控制变量
    _target_velocity = 0.0f;
    _current_velocity = 0.0f;
    _angle_prev = 0.0f;
    _velocity_timestamp_prev = esp_timer_get_time();
    
    // 初始化电流控制变量
    _Iq = 0.0f;
    _Id = 0.0f;
    _Iq_target = 0.0f;
    _Id_target = 0.0f;
    
    // 初始化电流数据
    _phase_currents.a = 0.0f;
    _phase_currents.b = 0.0f;
    _phase_currents.c = 0.0f;
    
    // 初始化电流采样和PWM管理器指针
    _current_sampler = nullptr;
    _pwm_manager = nullptr;
    
    // 初始化电流采样变量
    _current_sampling_enabled = false;
    _pwm_center_triggered = false;
    _current_sector = 0;
    _sampling_phase = 0; // 0: 等待第一次采样, 1: 第一次采样完成, 2: 第二次采样完成
}

FOC::~FOC() {
    // 清理动态分配的对象
    if (_current_sampler != nullptr) {
        delete _current_sampler;
    }
    if (_pwm_manager != nullptr) {
        delete _pwm_manager;
    }
}

void FOC::setVoltageDQ(float Uq, float Ud) {
    float angle_el = getElectricalAngle();
    
    // 限制电压范围
    Uq = _constrain(Uq, -_voltage_power_supply/2, _voltage_power_supply/2);
    Ud = _constrain(Ud, -_voltage_power_supply/2, _voltage_power_supply/2);
    
    // 帕克逆变换
    float cos_angle = cos(angle_el);
    float sin_angle = sin(angle_el);
    float Ualpha = Ud * cos_angle - Uq * sin_angle;
    float Ubeta = Ud * sin_angle + Uq * cos_angle;
    
    // 设置SVPWM输出
    setSVPWM(Ualpha, Ubeta);
}

bool FOC::isCurrentSamplingReady() const {
    if (_current_sampler == nullptr) return false;
    return _current_sampler->areSamplesReady();
}

void FOC::triggerCurrentSampling() {
    if (!_current_sampling_enabled || _current_sampler == nullptr) return;
    
    static bool first_sample = true;
    float angle_el = getElectricalAngle();
    uint8_t sector = _getSectorFromAngle(angle_el);
    
    if (first_sample) {
        // 第一次采样（PWM周期开始）
        _current_sampler->triggerFirstSample(sector);
        first_sample = false;
    } else {
        // 第二次采样（PWM周期中间）
        _current_sampler->triggerSecondSample();
        first_sample = true;
        
        // 采样完成后读取电流
        if (_current_sampler->areSamplesReady()) {
            _readCurrents();
        }
    }
}
// PWM中心对齐中断回调
void IRAM_ATTR FOC::pwmCenterCallback(void* arg) {
    FOC* foc = (FOC*)arg;
    if (foc->_current_sampling_enabled) {
        foc->triggerCurrentSampling();
    }
}

// 电流采样回调
void FOC::sampleCurrents() {
    if (_current_sampler != nullptr && _current_sampler->areSamplesReady()) {
        _readCurrents();
        _sampling_phase = 0; // 重置采样阶段
        ESP_LOGD("FOC", "采样完成，准备下一次采样");
    }
}

// 启用/禁用电流采样
void FOC::enableCurrentSampling(bool enable) {
    _current_sampling_enabled = enable;
    if (!enable) {
        _sampling_phase = 0; // 禁用时重置采样阶段
    }
}

// 初始化函数：配置硬件和外设
esp_err_t FOC::begin(float power_supply) {
    _voltage_power_supply = power_supply;
    
    // 设置电流环限制
    _current_q_pid = PIDController(CURRENT_Q_KP, CURRENT_Q_KI, CURRENT_Q_KD, 
                                  CURRENT_Q_RAMP, _voltage_power_supply);
    _current_d_pid = PIDController(CURRENT_D_KP, CURRENT_D_KI, CURRENT_D_KD, 
                                  CURRENT_D_RAMP, _voltage_power_supply);
    
    // 创建单电阻采样器
    _current_sampler = new SingleShuntSampler(ADC_CHANNEL_1, 0.01f, 20.0f);  // GPIO36
    if (_current_sampler->init() != ESP_OK) {
        ESP_LOGE("FOC", "单电阻采样器初始化失败");
        return ESP_FAIL;
    }
    
    // 创建PWM管理器
    _pwm_manager = new PWMManager(_pwmA, _pwmB, _pwmC, _channelA, _channelB, _channelC);
    if (_pwm_manager->init(30000, LEDC_TIMER_8_BIT) != ESP_OK) {
        ESP_LOGE("FOC", "PWM管理器初始化失败");
        return ESP_FAIL;
    }

    // 创建PWM中心对齐定时器
    const esp_timer_create_args_t timer_args = {
        .callback = pwmCenterCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_center_timer",
        .skip_unhandled_events = false
    };
    
    esp_timer_handle_t pwm_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pwm_timer));
    
    const uint64_t half_period_us = 1000000 / (2 * 30000); 
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_timer, half_period_us));

    // 初始化传感器
    BeginSensor();

    // 初始化UART
 // 在 FOC::begin() 函数中修复 UART 配置
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT,
    .flags = {0}  // 添加缺失的flags字段
};

   uart_param_config(UART_NUM_0, &uart_config);
   uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
   uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 启用电流采样
    enableCurrentSampling(true);

    ESP_LOGI("FOC", "FOC初始化完成，电源电压: %.1fV", power_supply);
    return ESP_OK;
}

// 角度归一化到 [0, 2PI]
float FOC::_normalizeAngle(float angle) {
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

// 数值约束
float FOC::_constrain(float value, float min, float max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// 设置PWM占空比 - 使用PWMManager
void FOC::_setPwm(float Ua, float Ub, float Uc) {
    if (_pwm_manager == nullptr) return;
    
    // 限制电压范围
    Ua = _constrain(Ua, 0.0f, _voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, _voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, _voltage_power_supply);
    
    // 计算占空比
    float dc_a = _constrain(Ua / _voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / _voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / _voltage_power_supply, 0.0f, 1.0f);

    // 使用PWM管理器设置占空比
    _pwm_manager->setDutyCycles(dc_a, dc_b, dc_c);
}

// 设置SVPWM
void FOC::setSVPWM(float u_alpha, float u_beta) {
    if (_pwm_manager == nullptr) return;
    
    _pwm_manager->setSVPWM(u_alpha, u_beta, _voltage_power_supply);
    _current_sector = _pwm_manager->getSector(u_alpha, u_beta);
}

// 设置扭矩
void FOC::setTorque(float Uq, float angle_el) {
    Uq = _constrain(Uq, -_voltage_power_supply / 2, _voltage_power_supply / 2);
    _Ud = 0; 
    angle_el = _normalizeAngle(angle_el);
    
    // 帕克逆变换（将dq轴电压转换为αβ轴）
    _Ualpha = -Uq * sin(angle_el);
    _Ubeta = Uq * cos(angle_el);

    // 设置SVPWM并更新扇区
    setSVPWM(_Ualpha, _Ubeta);
    
    ESP_LOGD("FOC", "设置扭矩: Uq=%.2f, 角度=%.2f", Uq, angle_el);
}

// 传感器初始化
void FOC::BeginSensor() {
    if (_sensor.begin() != ESP_OK) {
        const char* err_msg = "AS5600初始化失败";
        uart_write_bytes(UART_NUM_0, err_msg, strlen(err_msg));
        uart_write_bytes(UART_NUM_0, "\r\n", 2);
        ESP_LOGE("FOC", "AS5600传感器初始化失败");
    } else {
        ESP_LOGI("FOC", "AS5600传感器初始化成功");
    }
}

// 获取无圈数跟踪的角度
float FOC::getAngle_Without_track() {
    float angle = 0.0f;
    _sensor.get_angle_without_track(&angle);
    return angle;
}

// 获取带圈数的累计角度
float FOC::getAngle() {
    float angle = 0.0f;
    _sensor.get_angle(&angle);
    return _DIR * angle;
}

// 计算电角度
float FOC::_electricalAngle() {
    float mechanical_angle = getAngle_Without_track();
    return _normalizeAngle((float)(_DIR * _PP) * mechanical_angle - _zero_electric_angle);
}

// 获取电角度（对外接口）
float FOC::getElectricalAngle() {
    return _electricalAngle();
}

// 传感器对齐
void FOC::alignSensor() {
    enableCurrentSampling(false);
    
    ESP_LOGI("FOC", "开始传感器对齐...");
    
    // 施加固定电压使电机定位到电角度3PI/2处
    setTorque(3.0f, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 记录当前机械角度对应的零电角度
    _zero_electric_angle = _electricalAngle();
    
    // 停止输出
    setTorque(0.0f, _3PI_2);
    
    // 启用电流采样
    enableCurrentSampling(true);
    
    // 打印校准结果
    char msg[64];
    sprintf(msg, "零电角度校准完成: %f", _zero_electric_angle);
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
    uart_write_bytes(UART_NUM_0, "\r\n", 2);
    
    ESP_LOGI("FOC", "传感器对齐完成，零电角度: %.2f", _zero_electric_angle);
}

// 根据角度获取扇区
uint8_t FOC::_getSectorFromAngle(float angle_el) {
    angle_el = _normalizeAngle(angle_el);
    return (uint8_t)(angle_el / (PI / 3)) + 1;
}

// 根据电压获取扇区
uint8_t FOC::_getSectorFromVoltage(float u_alpha, float u_beta) {
    if (_pwm_manager == nullptr) return 1;
    return _pwm_manager->getSector(u_alpha, u_beta);
}

// 读取电流值 - 使用单电阻采样器
void FOC::_readCurrents() {
    if (_current_sampler == nullptr) return;
    
    float angle_el = getElectricalAngle();
    
    // 获取当前扇区
    _current_sector = _getSectorFromAngle(angle_el);
    if (_current_sector > 6) _current_sector = 1;
    
    // 使用单电阻采样器采样并重建电流
    float ia, ib, ic;
    if (_current_sampler->sampleAndReconstruct(_current_sector, &ia, &ib, &ic)) {
        _phase_currents.a = ia;
        _phase_currents.b = ib;
        _phase_currents.c = ic;
        
        ESP_LOGD("FOC", "电流读取: Ia=%.3f, Ib=%.3f, Ic=%.3f, 扇区=%d", 
                 ia, ib, ic, _current_sector);
    } else {
        ESP_LOGW("FOC", "电流重建失败");
    }
}

// 电流环控制
void FOC::_currentControl(float angle_el) {
    // 检查是否有新的电流采样数据
    sampleCurrents();
    
    // Clark变换（将三相电流转换为αβ坐标系）
    float I_alpha = _phase_currents.a;
    float I_beta = _1_SQRT3 * _phase_currents.a + _2_SQRT3 * _phase_currents.b;
    
    // Park变换（将αβ坐标系电流转换为dq坐标系）
    float cos_angle = cos(angle_el);
    float sin_angle = sin(angle_el);
    _Id = I_alpha * cos_angle + I_beta * sin_angle;
    _Iq = I_beta * cos_angle - I_alpha * sin_angle;
    
    // 电流环PID控制
    float Ud = _current_d_pid(_Id_target - _Id);
    float Uq = _current_q_pid(_Iq_target - _Iq);
    
    // 限制输出电压
    Ud = _constrain(Ud, -_voltage_power_supply/2, _voltage_power_supply/2);
    Uq = _constrain(Uq, -_voltage_power_supply/2, _voltage_power_supply/2);
    
    // 帕克逆变换（将dq轴电压转换为αβ轴）
    _Ualpha = Ud * cos_angle - Uq * sin_angle;
    _Ubeta = Ud * sin_angle + Uq * cos_angle;
    
    // 设置SVPWM输出
    setSVPWM(_Ualpha, _Ubeta);
    
    ESP_LOGD("FOC", "电流控制: Id=%.3f/%.3f, Iq=%.3f/%.3f, Ud=%.2f, Uq=%.2f", 
             _Id, _Id_target, _Iq, _Iq_target, Ud, Uq);
}

// 设置目标Iq电流
void FOC::setTargetCurrentIq(float iq_target) { 
    _Iq_target = iq_target; 
    ESP_LOGD("FOC", "设置目标Iq: %.3fA", iq_target);
}

// 设置目标Id电流
void FOC::setTargetCurrentId(float id_target) { 
    _Id_target = id_target; 
    ESP_LOGD("FOC", "设置目标Id: %.3fA", id_target);
}

// 执行电流环控制
void FOC::currentLoop() {
    float angle_el = getElectricalAngle();
    _currentControl(angle_el);
}

// 获取当前Iq值
float FOC::getCurrentIq() const { 
    return _Iq; 
}

// 获取当前Id值
float FOC::getCurrentId() const { 
    return _Id; 
}

// 获取速度（弧度/秒）
float FOC::getVelocity() {
    uint64_t now = esp_timer_get_time();
    float dt = (now - _velocity_timestamp_prev) * 1e-6f;
    
    if (dt <= 0 || dt > 0.5f) {
        dt = 1e-3f;
    }
    
    float current_angle = getAngle();
    float velocity = (current_angle - _angle_prev) / dt;
    
    _angle_prev = current_angle;
    _velocity_timestamp_prev = now;
    
    return _velocity_filter(velocity);
}

// 设置目标速度
void FOC::setVelocity(float target_velocity) {
    _target_velocity = target_velocity;
    ESP_LOGD("FOC", "设置目标速度: %.2f rad/s", target_velocity);
}

// 速度环控制
void FOC::velocityLoop() {
    _current_velocity = getVelocity();
    
    float velocity_error = _target_velocity - _current_velocity;
    _Iq_target = _velocity_pid(velocity_error);
    
    _Iq_target = _constrain(_Iq_target, -_voltage_power_supply/2, _voltage_power_supply/2);
    
    float angle_el = getElectricalAngle();
    _currentControl(angle_el);
    
    ESP_LOGD("FOC", "速度控制: 目标=%.2f, 实际=%.2f, Iq_target=%.3f", 
             _target_velocity, _current_velocity, _Iq_target);
}

// 串口接收用户命令
void FOC::serialReceiveUserCommand() {
    uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
    
    if (len > 0) {
        for (int i = 0; i < len; i++) {
            if (data[i] == '\n' || data[i] == '\r') {
                _received_chars[_receive_index] = '\0';
                _commaPosition = -1;
                
                for (int j = 0; j < _receive_index; j++) {
                    if (_received_chars[j] == ',') {
                        _commaPosition = j;
                        break;
                    }
                }
                
                if (_commaPosition != -1) {
                    char target_str[MAX_RECEIVE_LEN];
                    strncpy(target_str, _received_chars, _commaPosition);
                    target_str[_commaPosition] = '\0';
                    _motor_target = atof(target_str);
                    
                    char msg[64];
                    sprintf(msg, "目标值更新: %f", _motor_target);
                    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
                    uart_write_bytes(UART_NUM_0, "\r\n", 2);
                    
                    ESP_LOGI("FOC", "接收到新目标值: %.3f", _motor_target);
                }
                
                _receive_index = 0;
                memset(_received_chars, 0, MAX_RECEIVE_LEN);
            } else if (_receive_index < MAX_RECEIVE_LEN - 1) {
                _received_chars[_receive_index++] = data[i];
            }
        }
    }
}

// 获取电机目标值
float FOC::getMotorTarget() {
    return _motor_target;
}