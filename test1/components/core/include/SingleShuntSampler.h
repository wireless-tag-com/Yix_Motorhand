#ifndef SINGLE_SHUNT_SAMPLER_H
#define SINGLE_SHUNT_SAMPLER_H

#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <string.h>

class SingleShuntSampler {
public:
    SingleShuntSampler(adc_channel_t adc_channel, 
                      float shunt_resistance = 0.01f, 
                      float op_amp_gain = 20.0f);
    ~SingleShuntSampler();

    esp_err_t init();
    void calibrateOffset();
    uint32_t readADC();
    float adcToCurrent(uint32_t adc_value);
    float readShuntCurrent();
    
    // 电流重建相关方法
    void triggerFirstSample(uint8_t sector);
    void triggerSecondSample();
    bool areSamplesReady() const;
    void reconstructPhaseCurrents(uint8_t sector, float sample1, float sample2);
    bool sampleAndReconstruct(uint8_t sector, float* ia, float* ib, float* ic);
    bool getPhaseCurrents(uint8_t sector, float* ia, float* ib, float* ic);
    
    // 参数调整方法
    void setCurrentScale(float scale_factor);
    void setShuntParameters(float shunt_resistance, float op_amp_gain);
    
    // 静态方法
    static void delay_us(uint32_t us);
    static void sampleTimerCallback(void* arg);

private:
    adc_channel_t adc_channel;
    float shunt_resistance;
    float op_amp_gain;
    int32_t adc_offset;
    float filter_alpha;
    adc_oneshot_unit_handle_t adc_handle;
    esp_timer_handle_t sample_timer;
    
    // 电流缩放因子
    float _current_scale;
    
    // 采样状态
    bool _first_sample_ready;
    bool _second_sample_ready;
    float _sample1;
    float _sample2;
    uint8_t _current_sector;
    float filtered_currents[3];
    
    // 私有方法
    float _constrain(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
};

#endif // SINGLE_SHUNT_SAMPLER_H