#include "SpeedLoop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdio.h>

// 默认PID参数
#define DEFAULT_KP 0.8f
#define DEFAULT_KI 0.1f
#define DEFAULT_KD 0.02f
#define DEFAULT_RAMP 10.0f
#define DEFAULT_LIMIT 5.0f

SpeedLoop::SpeedLoop(FOC* foc_ptr, float kp, float ki, float kd, float ramp, float limit)
    : _foc(foc_ptr),
      _velocity_pid(kp, ki, kd, ramp, limit),
      _target_velocity(0.0f),
      _current_velocity(0.0f),
      _control_output(0.0f),
      _enabled(false)
{
    // 参数验证
    if (_foc == nullptr) {
        printf("错误: FOC指针不能为null!\n");
    }
}

esp_err_t SpeedLoop::begin()
{
    if (_foc == nullptr) {
        printf("错误: FOC未初始化!\n");
        return ESP_FAIL;
    }
    
    // 重置PID控制器
    _velocity_pid.reset();
    
    _enabled = true;
    printf("速度环初始化完成\n");
    printf("PID参数: P=%.3f, I=%.3f, D=%.3f, Ramp=%.1f, Limit=%.1f\n",
           _velocity_pid.P, _velocity_pid.I, _velocity_pid.D, 
           _velocity_pid.output_ramp, _velocity_pid.limit);
    
    return ESP_OK;
}

void SpeedLoop::setTargetVelocity(float target_velocity)
{
    _target_velocity = target_velocity;
    printf("目标速度设置为: %.2f rad/s\n", target_velocity);
}

float SpeedLoop::getTargetVelocity() const
{
    return _target_velocity;
}

float SpeedLoop::getCurrentVelocity()
{
    _current_velocity = _foc->getVelocity();
    return _current_velocity;
}

float SpeedLoop::getVelocityError()
{
    return _target_velocity - _current_velocity;
}

float SpeedLoop::getControlOutput() const
{
    return _control_output;
}

void SpeedLoop::setPIDParams(float kp, float ki, float kd, float ramp, float limit)
{
    _velocity_pid.P = kp;
    _velocity_pid.I = ki;
    _velocity_pid.D = kd;
    _velocity_pid.output_ramp = ramp;
    _velocity_pid.limit = limit;
    
    printf("PID参数更新: P=%.3f, I=%.3f, D=%.3f, Ramp=%.1f, Limit=%.1f\n",
           kp, ki, kd, ramp, limit);
}

void SpeedLoop::resetPID()
{
    _velocity_pid.reset();
    _control_output = 0.0f;
    printf("PID控制器已重置\n");
}

void SpeedLoop::update()
{
    if (!_enabled || _foc == nullptr) {
        return;
    }
    
    // 获取当前速度
    _current_velocity = _foc->getVelocity();
    
    // 计算速度误差
    float error = _target_velocity - _current_velocity;
    
    // PID控制计算
    _control_output = _velocity_pid(error);
    
    // 获取电角度
    float electrical_angle = _foc->getElectricalAngle();
    
    // 设置扭矩输出
    _foc->setTorque(_control_output, electrical_angle);
}

void SpeedLoop::enable(bool enabled)
{
    _enabled = enabled;
    if (enabled) {
        printf("速度环已启用\n");
    } else {
        printf("速度环已禁用\n");
        // 禁用时停止电机
        _foc->setTorque(0.0f, _foc->getElectricalAngle());
    }
}

bool SpeedLoop::isEnabled() const
{
    return _enabled;
}