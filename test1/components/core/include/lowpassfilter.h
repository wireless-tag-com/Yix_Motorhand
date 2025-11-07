#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include "esp_timer.h"  

class LowPassFilter {
private:
    float _Tf;                  
    float _y_prev;            
    uint64_t _timestamp_prev;   

public:
    LowPassFilter(float time_constant);

    float operator()(float x);

    void reset(float initial_value = 0.0f);
};

#endif // LOWPASSFILTER_H