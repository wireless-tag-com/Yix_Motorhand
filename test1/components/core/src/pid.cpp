#include "pid.h"
#include "esp_timer.h" 

static inline float _constrain(float amt, float low, float high) {
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    
    , limit(limit)         
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = esp_timer_get_time();
}

void PIDController::reset() {
    integral_prev = 0.0f;    
    error_prev = 0.0f;       
    output_prev = 0.0f;      
}

// PID 控制器函数
float PIDController::operator() (float error){
    
    uint64_t timestamp_now = esp_timer_get_time();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    
    if(Ts <= 0 || Ts > 0.5f) {
        Ts = 1e-3f;  // 默认为1ms
    }
    
    float proportional = P * error;

    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
    integral = _constrain(integral, -limit, limit);

    float derivative = D * (error - error_prev) / Ts;

    // 计算输出并限幅
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);

    // 加速度限制
    if(output_ramp > 0){
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp) {
            output = output_prev + output_ramp * Ts;
        } else if (output_rate < -output_ramp) {
            output = output_prev - output_ramp * Ts;
        }
    }

    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    
    return output;
}