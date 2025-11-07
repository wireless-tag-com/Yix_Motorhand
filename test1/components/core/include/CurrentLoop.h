#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "FOC.h"
#include "pid.h"

// 添加缺失的定义
#define DEFAULT_CURRENT_RAMP 1000.0f

class CurrentLoop {
public:
    CurrentLoop(FOC* foc_ptr, 
                float kp_q, float ki_q, float kd_q,
                float kp_d, float ki_d, float kd_d);
    
    esp_err_t begin();
    void update();
    void setTargetCurrentIq(float iq_target);
    void setTargetCurrentId(float id_target);
    void setTargetCurrent(float iq_target, float id_target);
    float getCurrentIq() const;
    float getCurrentId() const;
    float getTargetIq() const;
    float getTargetId() const;
    float getCurrentErrorIq();
    float getCurrentErrorId();
    float getControlOutputIq() const;
    float getControlOutputId() const;
    void setPIDParamsIq(float kp, float ki, float kd, float ramp, float limit);
    void setPIDParamsId(float kp, float ki, float kd, float ramp, float limit);
    void resetPID();
    void enable(bool enabled);
    bool isEnabled() const;
    bool isCurrentSamplingReady() const;

private:
    FOC* _foc;
    PIDController _current_q_pid;
    PIDController _current_d_pid;
    float _target_iq;
    float _target_id;
    float _current_iq;
    float _current_id;
    float _control_output_iq;
    float _control_output_id;
    bool _enabled;
    uint32_t _update_count;

    // 约束函数
    float _constrain(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
};

#endif // CURRENT_LOOP_H