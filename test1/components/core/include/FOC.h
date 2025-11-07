/*#ifndef FOC_H
#define FOC_H

#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "CurrentSense.h"
#include "lowpassfilter.h"
#include "pid.h"
#include <math.h>
#include <string.h>
#include "esp_intr_alloc.h" 
#include "AS5600.h"

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define BUF_SIZE (1024)
#define PI 3.14159265359f
#define _3PI_2 4.71238898f
#define MAX_RECEIVE_LEN 32

class FOC {
public:
    FOC(gpio_num_t pwmA, gpio_num_t pwmB, gpio_num_t pwmC,
        ledc_channel_t chA, ledc_channel_t chB, ledc_channel_t chC,
        int polePairs, int direction);
    
    esp_err_t begin(float power_supply);
    void setTorque(float Uq, float angle_el);
    void alignSensor();
    float getAngle();
    void serialReceiveUserCommand();
    float getMotorTarget();
    float getElectricalAngle();
    float getVelocity();
    void setVelocity(float target_velocity);
    void velocityLoop();
    void readCurrents() { _readCurrents(); }
    float getCurrentA() const { return _current_sense.current_a; }
    float getCurrentB() const { return _current_sense.current_b; }
    float getCurrentC() const { return _current_sense.current_c; }
    const CurrSense& getCurrentSense() const { return _current_sense; }
    void sampleCurrents();
    void enableCurrentSampling(bool enable);
    void setTargetCurrentIq(float iq_target);
    void setTargetCurrentId(float id_target);
    void currentLoop();
    float getCurrentIq() const;
    float getCurrentId() const;
    float getVoltageSupply() const { return _voltage_power_supply; }
    bool checkObservationRegion(float angle_el, float t1, float t2);
    void handleNonObservationRegion(float* duty_a, float* duty_b, float* duty_c, int sector);

    // 修改为静态辅助函数，可以在const成员函数中调用
    static float constrainValue(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
    
    static void IRAM_ATTR pwmCenterCallback(void* arg);  
     
private:
    gpio_num_t _pwmA, _pwmB, _pwmC;
    ledc_channel_t _channelA, _channelB, _channelC;
    int _PP, _DIR;
    float _voltage_power_supply;  
    float _Ualpha, _Ubeta, _Ua, _Ub, _Uc;
    float _zero_electric_angle;
    float _motor_target;
    float _Ud;
    char _received_chars[MAX_RECEIVE_LEN];
    int _receive_index;
    int _commaPosition;
    float _target_velocity;
    float _current_velocity;
    float _angle_prev;
    uint64_t _velocity_timestamp_prev;
    AS5600 _sensor;   
    LowPassFilter _velocity_filter;
    PIDController _velocity_pid;
    CurrSense _current_sense; 
    float _Iq, _Id;
    float _Iq_target, _Id_target;
    PIDController _current_q_pid;
    PIDController _current_d_pid;
    bool _current_sampling_enabled;
    bool _pwm_center_triggered;
    int _current_sector;
    
  
    float _normalizeAngle(float angle);
    static float _constrain(float value, float min, float max);  
    void _setPwm(float Ua, float Ub, float Uc);
    float _electricalAngle();
    void BeginSensor();
    float getAngle_Without_track();
    void _readCurrents();
    void _currentControl(float angle_el);
};
#endif // FOC_H
#ifndef FOC_H
#define FOC_H*/
// FOC.h
/*#ifndef FOC_H
#define FOC_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include "AS5600.h"
#include "lowpassfilter.h"
#include "pid.h"
#include "SingleShuntSampler.h"
#include "PWMManager.h"

#define PI 3.14159265358979323846f
#define _2PI 6.28318530717958647692f
#define _3PI_2 4.71238898038468985769f
#define _2_SQRT3 1.15470053837925152901f
#define _1_SQRT3 0.57735026918962576450f
#define _SQRT3 1.73205080756887729352f

#define BUF_SIZE 128
#define MAX_RECEIVE_LEN 64

class FOC {
public:
    // 构造函数和析构函数
    FOC(gpio_num_t pwmA, gpio_num_t pwmB, gpio_num_t pwmC,
        ledc_channel_t chA, ledc_channel_t chB, ledc_channel_t chC,
        int polePairs, int direction);
    ~FOC();

    // 初始化函数
    esp_err_t begin(float power_supply);

    // 传感器相关函数
    void BeginSensor();
    float getAngle_Without_track();
    float getAngle();
    float getElectricalAngle();
    void alignSensor();

    // 电流控制函数
    void setTargetCurrentIq(float iq_target);
    void setTargetCurrentId(float id_target);
    float getCurrentIq() const;
    float getCurrentId() const;
    void currentLoop();
    
    // 电压控制函数（用于CurrentLoop直接控制）
    void setVoltageDQ(float Uq, float Ud);
    void setSVPWM(float u_alpha, float u_beta);

    // 速度控制函数
    void setVelocity(float target_velocity);
    float getVelocity();
    void velocityLoop();

    // 基础控制函数
    void setTorque(float Uq, float angle_el);

    // 采样控制函数
    void enableCurrentSampling(bool enable);
    bool isCurrentSamplingReady() const;
    void sampleCurrents();

    // 串口通信函数
    void serialReceiveUserCommand();
    float getMotorTarget();

    // 获取电源电压
    float getVoltageSupply() const { return _voltage_power_supply; }

    // PWM回调函数（静态）
    static void IRAM_ATTR pwmCenterCallback(void* arg);

private:
    // 私有成员变量
    gpio_num_t _pwmA, _pwmB, _pwmC;
    ledc_channel_t _channelA, _channelB, _channelC;
    int _PP;  // 极对数
    int _DIR; // 方向

    AS5600 _sensor;
    float _voltage_power_supply;
    
    // 电压变量
    float _Ualpha, _Ubeta;
    float _Ua, _Ub, _Uc;
    float _Ud;
    
    // 角度和位置变量
    float _zero_electric_angle;
    float _angle_prev;
    
    // 速度控制变量
    LowPassFilter _velocity_filter;
    PIDController _velocity_pid;
    float _target_velocity;
    float _current_velocity;
    uint64_t _velocity_timestamp_prev;
    
    // 电流控制变量
    PIDController _current_q_pid;
    PIDController _current_d_pid;
    float _Iq, _Id;
    float _Iq_target, _Id_target;
    
    // 电流采样相关
    SingleShuntSampler* _current_sampler;
    PWMManager* _pwm_manager;
    bool _current_sampling_enabled;
    bool _pwm_center_triggered;
    uint8_t _current_sector;
    uint8_t _sampling_phase;
    
    struct {
        float a, b, c;
    } _phase_currents;
    
    // 串口通信变量
    char _received_chars[MAX_RECEIVE_LEN];
    int _receive_index;
    int _commaPosition;
    float _motor_target;

    // 私有方法
    float _normalizeAngle(float angle);
    float _constrain(float value, float min, float max);
    void _setPwm(float Ua, float Ub, float Uc);
    float _electricalAngle();
    void _readCurrents();
    void _currentControl(float angle_el);
    uint8_t _getSectorFromAngle(float angle_el);
    uint8_t _getSectorFromVoltage(float u_alpha, float u_beta);
    void triggerCurrentSampling();
};

#endif // FOC_H*/
#ifndef FOC_H
#define FOC_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include "AS5600.h"
#include "lowpassfilter.h"
#include "pid.h"
#include "SingleShuntSampler.h"
#include "PWMManager.h"

#define PI 3.14159265358979323846f
#define _2PI 6.28318530717958647692f
#define _3PI_2 4.71238898038468985769f
#define _2_SQRT3 1.15470053837925152901f
#define _SQRT3 1.73205080756887729352f
#define _1_SQRT3 0.57735026918962576450f

#define MAX_RECEIVE_LEN 32
#define BUF_SIZE 1024

class FOC {
public:
    // 构造函数
    FOC(gpio_num_t pwmA, gpio_num_t pwmB, gpio_num_t pwmC,
        ledc_channel_t chA, ledc_channel_t chB, ledc_channel_t chC,
        int polePairs, int direction);
    
    ~FOC();

    // 初始化函数
    esp_err_t begin(float power_supply);

    // 传感器相关函数
    void BeginSensor();
    float getAngle_Without_track();
    float getAngle();
    float getElectricalAngle();
    void alignSensor();

    // 控制函数
    void setTorque(float Uq, float angle_el);
    void setVoltageDQ(float Uq, float Ud);
    void setSVPWM(float u_alpha, float u_beta);
    void currentLoop();
    void velocityLoop();

    // 电流控制函数
    void setTargetCurrentIq(float iq_target);
    void setTargetCurrentId(float id_target);
    float getCurrentIq() const;
    float getCurrentId() const;

    // 速度控制函数
    float getVelocity();
    void setVelocity(float target_velocity);

    // 串口通信函数
    void serialReceiveUserCommand();
    float getMotorTarget();

    // 状态获取函数
    float getVoltageSupply() const { return _voltage_power_supply; }
    bool isCurrentSamplingReady() const;

    // 电流采样控制
    void enableCurrentSampling(bool enable);
    void sampleCurrents();
    void triggerCurrentSampling();

    // 三相电流获取函数
    float getCurrentA() const { return _phase_currents.a; }
    float getCurrentB() const { return _phase_currents.b; }
    float getCurrentC() const { return _phase_currents.c; }

    // PWM中心对齐回调（静态）
    static void IRAM_ATTR pwmCenterCallback(void* arg);

private:
    // PWM引脚和通道
    gpio_num_t _pwmA, _pwmB, _pwmC;
    ledc_channel_t _channelA, _channelB, _channelC;
    
    // 电机参数
    int _PP;  // 极对数
    int _DIR; // 方向
    
    // 传感器
    AS5600 _sensor;
    
    // 滤波器
    LowPassFilter _velocity_filter;
    
    // PID控制器
    PIDController _velocity_pid;
    PIDController _current_q_pid;
    PIDController _current_d_pid;
    
    // 电源和电压
    float _voltage_power_supply;
    float _Ualpha, _Ubeta;
    float _Ua, _Ub, _Uc;
    float _Ud;
    
    // 角度相关
    float _zero_electric_angle;
    float _angle_prev;
    uint64_t _velocity_timestamp_prev;
    
    // 电流相关
    float _Iq, _Id;
    float _Iq_target, _Id_target;
    
    // 速度控制
    float _target_velocity;
    float _current_velocity;
    
    // 串口通信
    char _received_chars[MAX_RECEIVE_LEN];
    int _receive_index;
    int _commaPosition;
    float _motor_target;
    
    // 电流采样
    struct {
        float a, b, c;
    } _phase_currents;
    
    SingleShuntSampler* _current_sampler;
    PWMManager* _pwm_manager;
    
    bool _current_sampling_enabled;
    bool _pwm_center_triggered;
    uint8_t _current_sector;
    uint8_t _sampling_phase;

    // 私有方法
    float _normalizeAngle(float angle);
    float _constrain(float value, float min, float max);
    void _setPwm(float Ua, float Ub, float Uc);
    void _currentControl(float angle_el);
    void _readCurrents();
    float _electricalAngle();
    uint8_t _getSectorFromAngle(float angle_el);
    uint8_t _getSectorFromVoltage(float u_alpha, float u_beta);
};

#endif // FOC_H