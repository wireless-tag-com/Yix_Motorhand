#include "CurrentLoop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "CurrentLoop";

// 移除有问题的构造函数初始化，使用成员初始化列表
CurrentLoop::CurrentLoop(FOC* foc_ptr, 
                         float kp_q, float ki_q, float kd_q,
                         float kp_d, float ki_d, float kd_d)
    : _foc(foc_ptr),
      _current_q_pid(kp_q, ki_q, kd_q, 1000.0f, 0.0f),  // 直接使用数值替代DEFAULT_CURRENT_RAMP
      _current_d_pid(kp_d, ki_d, kd_d, 1000.0f, 0.0f),
      _target_iq(0.0f),
      _target_id(0.0f),
      _current_iq(0.0f),
      _current_id(0.0f),
      _control_output_iq(0.0f),
      _control_output_id(0.0f),
      _enabled(false),
      _update_count(0)
{
    // 参数验证
    if (_foc == nullptr) {
        ESP_LOGE(TAG, "错误: FOC指针不能为null!");
    } else {
        ESP_LOGI(TAG, "电流环控制器创建成功");
    }
}

esp_err_t CurrentLoop::begin()
{
    if (_foc == nullptr) {
        ESP_LOGE(TAG, "错误: FOC未初始化!");
        return ESP_FAIL;
    }
    
    // 设置电流环限制（基于电源电压）
    float voltage_limit = _foc->getVoltageSupply();
    _current_q_pid = PIDController(_current_q_pid.P, _current_q_pid.I, _current_q_pid.D, 
                                  _current_q_pid.output_ramp, voltage_limit);
    _current_d_pid = PIDController(_current_d_pid.P, _current_d_pid.I, _current_d_pid.D, 
                                  _current_d_pid.output_ramp, voltage_limit);
    
    // 重置PID控制器
    _current_q_pid.reset();
    _current_d_pid.reset();
    
    _enabled = true;
    
    ESP_LOGI(TAG, "电流环初始化完成");
    ESP_LOGI(TAG, "Q轴PID参数: P=%.3f, I=%.3f, D=%.3f, Ramp=%.1f, Limit=%.1f",
             _current_q_pid.P, _current_q_pid.I, _current_q_pid.D, 
             _current_q_pid.output_ramp, _current_q_pid.limit);
    ESP_LOGI(TAG, "D轴PID参数: P=%.3f, I=%.3f, D=%.3f, Ramp=%.1f, Limit=%.1f",
             _current_d_pid.P, _current_d_pid.I, _current_d_pid.D, 
             _current_d_pid.output_ramp, _current_d_pid.limit);
    
    return ESP_OK;
}

void CurrentLoop::setTargetCurrentIq(float iq_target)
{
    _target_iq = iq_target;
    ESP_LOGD(TAG, "目标Q轴电流设置为: %.3f A", iq_target);
}

void CurrentLoop::setTargetCurrentId(float id_target)
{
    _target_id = id_target;
    ESP_LOGD(TAG, "目标D轴电流设置为: %.3f A", id_target);
}

void CurrentLoop::setTargetCurrent(float iq_target, float id_target)
{
    _target_iq = iq_target;
    _target_id = id_target;
    ESP_LOGD(TAG, "目标电流设置: Iq=%.3fA, Id=%.3fA", iq_target, id_target);
}

float CurrentLoop::getCurrentIq() const
{
    return _current_iq;
}

float CurrentLoop::getCurrentId() const
{
    return _current_id;
}

float CurrentLoop::getTargetIq() const
{
    return _target_iq;
}

float CurrentLoop::getTargetId() const
{
    return _target_id;
}

float CurrentLoop::getCurrentErrorIq()
{
    return _target_iq - _current_iq;
}

float CurrentLoop::getCurrentErrorId()
{
    return _target_id - _current_id;
}

float CurrentLoop::getControlOutputIq() const
{
    return _control_output_iq;
}

float CurrentLoop::getControlOutputId() const
{
    return _control_output_id;
}

void CurrentLoop::setPIDParamsIq(float kp, float ki, float kd, float ramp, float limit)
{
    _current_q_pid.P = kp;
    _current_q_pid.I = ki;
    _current_q_pid.D = kd;
    _current_q_pid.output_ramp = ramp;
    _current_q_pid.limit = limit;
    
    ESP_LOGI(TAG, "Q轴PID参数更新: P=%.3f, I=%.3f, D=%.3f, Ramp=%.1f, Limit=%.1f",
             kp, ki, kd, ramp, limit);
}

void CurrentLoop::setPIDParamsId(float kp, float ki, float kd, float ramp, float limit)
{
    _current_d_pid.P = kp;
    _current_d_pid.I = ki;
    _current_d_pid.D = kd;
    _current_d_pid.output_ramp = ramp;
    _current_d_pid.limit = limit;
    
    ESP_LOGI(TAG, "D轴PID参数更新: P=%.3f, I=%.3f, D=%.3f, Ramp=%.1f, Limit=%.1f",
             kp, ki, kd, ramp, limit);
}

void CurrentLoop::resetPID()
{
    _current_q_pid.reset();
    _current_d_pid.reset();
    _control_output_iq = 0.0f;
    _control_output_id = 0.0f;
    
    ESP_LOGI(TAG, "电流环PID控制器已重置");
}

void CurrentLoop::update()
{
    if (!_enabled || _foc == nullptr) {
        return;
    }
    
    // 确保电流采样已完成
    if (!_foc->isCurrentSamplingReady()) {
        ESP_LOGD(TAG, "等待电流采样完成...");
        return; // 等待下一次采样
    }
    
    // 读取当前电流值
    _current_iq = _foc->getCurrentIq();
    _current_id = _foc->getCurrentId();
    
    // 计算电流误差
    float error_iq = _target_iq - _current_iq;
    float error_id = _target_id - _current_id;
    
    // PID控制计算，输出为电压值
    _control_output_iq = _current_q_pid(error_iq);
    _control_output_id = _current_d_pid(error_id);
    
    // 直接设置电压到FOC，绕过内部电流环
    _foc->setVoltageDQ(_control_output_iq, _control_output_id);
    
    // 调试信息
    _update_count++;
    if (_update_count % 100 == 0) {
        ESP_LOGI(TAG, 
                "电流环: Iq_ref=%.3fA, Iq=%.3fA, Id_ref=%.3fA, Id=%.3fA, "
                "Uq=%.3fV, Ud=%.3fV",
                _target_iq, _current_iq, _target_id, _current_id,
                _control_output_iq, _control_output_id);
    }
}

void CurrentLoop::enable(bool enabled)
{
    _enabled = enabled;
    if (enabled) {
        ESP_LOGI(TAG, "电流环已启用");
        // 启用时重置PID，避免积分饱和
        resetPID();
    } else {
        ESP_LOGI(TAG, "电流环已禁用");
        // 禁用时停止电流输出
        _foc->setTargetCurrentIq(0.0f);
        _foc->setTargetCurrentId(0.0f);
    }
}

bool CurrentLoop::isEnabled() const
{
    return _enabled;
}

bool CurrentLoop::isCurrentSamplingReady() const
{
    if (_foc == nullptr) return false;
    
    // 这里可以添加电流采样就绪状态的检查
    // 例如检查ADC采样是否完成，电流数据是否有效等
    return true;
}