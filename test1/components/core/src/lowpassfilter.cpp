#include "lowpassfilter.h"

LowPassFilter::LowPassFilter(float time_constant) 
    : _Tf(time_constant), 
      _y_prev(0.0f)        
{
    _timestamp_prev = esp_timer_get_time();  
}

// 输入原始数据x，返回滤波后的数据
float LowPassFilter::operator()(float x) {
    uint64_t timestamp = esp_timer_get_time();  
    float dt = (timestamp - _timestamp_prev) * 1e-6f;  

    // 处理时间间隔异常情况
    if (dt < 0.0f) {
        dt = 1e-3f;  // 异常时使用1ms默认间隔
    } 
    else if (dt > 0.3f) {
        _y_prev = x;
        _timestamp_prev = timestamp;
        return x;
    }

    float alpha = _Tf / (_Tf + dt);
    float y = alpha * _y_prev + (1.0f - alpha) * x;

    _y_prev = y;
    _timestamp_prev = timestamp;

    return y;
}


void LowPassFilter::reset(float initial_value) {
    _y_prev = initial_value;
    _timestamp_prev = esp_timer_get_time();
}