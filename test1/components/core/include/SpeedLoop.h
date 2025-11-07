#ifndef SPEEDLOOP_H
#define SPEEDLOOP_H

#include "FOC.h"
#include "pid.h"

class SpeedLoop {
public:
    // 构造函数
    SpeedLoop(FOC* foc_ptr, 
              float kp = 0.8f, float ki = 0.1f, float kd = 0.02f,
              float ramp = 10.0f, float limit = 5.0f);
    
    // 初始化速度环
    esp_err_t begin();
    
    // 设置目标速度（弧度/秒）
    void setTargetVelocity(float target_velocity);
    
    // 获取当前目标速度
    float getTargetVelocity() const;
    
    // 获取当前实际速度
    float getCurrentVelocity();
    
    // 获取当前速度误差
    float getVelocityError();
    
    // 获取当前控制输出Uq
    float getControlOutput() const;
    
    // 更新PID参数
    void setPIDParams(float kp, float ki, float kd, float ramp, float limit);
    
    // 重置PID控制器
    void resetPID();
    
    // 执行速度环控制（需要在主循环中定期调用）
    void update();
    
    // 启用/禁用速度环控制
    void enable(bool enabled);
    
    // 检查速度环是否启用
    bool isEnabled() const;

private:
    FOC* _foc;                          // FOC对象指针
    PIDController _velocity_pid;        // 速度PID控制器
    float _target_velocity;             // 目标速度（rad/s）
    float _current_velocity;            // 当前速度（rad/s）
    float _control_output;              // 控制输出Uq
    bool _enabled;                      // 速度环使能标志
};

#endif // SPEEDLOOP_H