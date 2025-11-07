#ifndef PID_H
#define PID_H

#include "esp_timer.h"  

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;
    void reset();
    float operator() (float error);

    float P;
    float I; 
    float D; 
    float output_ramp; 
    float limit; 
protected:
    float error_prev; 
    float output_prev;  
    float integral_prev; 
    uint64_t timestamp_prev;  
};

#endif