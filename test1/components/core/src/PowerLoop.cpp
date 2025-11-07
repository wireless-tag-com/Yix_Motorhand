#include "PowerLoop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

PowerLoop::PowerLoop(FOC* foc_ptr, float kp, float ki, float kd, float ramp, float limit)
    : _foc(foc_ptr),
      _torque_pid(kp, ki, kd, ramp, limit),
      _target_torque(0.0f),
      _current_torque(0.0f) {
}

esp_err_t PowerLoop::begin() {
    if (_foc == nullptr) {
        ESP_LOGE("PowerLoop", "FOC pointer is null");
        return ESP_FAIL;
    }
    
    _torque_pid.reset();
    
    ESP_LOGI("PowerLoop", "Torque loop initialized successfully");
    return ESP_OK;
}

void PowerLoop::setTargetTorque(float target_torque) {
    _target_torque = target_torque;
}

void PowerLoop::torqueLoop() {
    if (_foc == nullptr) return;
    
    float current_iq = _foc->getCurrentIq();
    _current_torque = current_iq;
    
    // 计算力矩误差并使用PID控制器
    float torque_error = _target_torque - _current_torque;
    float Uq = _torque_pid(torque_error);
    
    // 获取电角度
    float electrical_angle = _foc->getElectricalAngle();
    
    // 设置扭矩
    _foc->setTorque(Uq, electrical_angle);
}

float PowerLoop::getCurrentTorque() const {
    return _current_torque;
}

float PowerLoop::getTargetTorque() const {
    return _target_torque;
}

void PowerLoop::updatePIDParams(float kp, float ki, float kd, float ramp, float limit) {
    _torque_pid.P = kp;
    _torque_pid.I = ki;
    _torque_pid.D = kd;
    _torque_pid.output_ramp = ramp;
    _torque_pid.limit = limit;
    
    ESP_LOGI("PowerLoop", "PID parameters updated: KP=%.3f, KI=%.3f, KD=%.3f", kp, ki, kd);
}