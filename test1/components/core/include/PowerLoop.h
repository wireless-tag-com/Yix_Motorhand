#ifndef POWERLOOP_H
#define POWERLOOP_H

#include "FOC.h"
#include "pid.h"
#include <math.h>

class PowerLoop {
public:
    // 构造函数
    PowerLoop(FOC* foc_ptr, 
              float kp = 0.5f, float ki = 0.01f, float kd = 0.05f,
              float ramp = 5.0f, float limit = 3.0f);
    
    // 初始化函数
    esp_err_t begin();
    
    // 设置目标力矩
    void setTargetTorque(float target_torque);
    
    // 执行力矩环控制
    void torqueLoop();
    
    // 获取当前力矩
    float getCurrentTorque() const;
    
    // 获取目标力矩
    float getTargetTorque() const;
    
    // 更新PID参数
    void updatePIDParams(float kp, float ki, float kd, float ramp, float limit);

private:
    FOC* _foc;                  
    PIDController _torque_pid;  
    float _target_torque;       
    float _current_torque;     
};

#endif // POWERLOOP_H