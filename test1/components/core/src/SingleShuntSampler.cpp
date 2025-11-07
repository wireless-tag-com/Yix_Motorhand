#include "SingleShuntSampler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "SINGLE_SHUNT";

// 修复静态成员函数定义（去掉static关键字）
void SingleShuntSampler::delay_us(uint32_t us) {
    uint32_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        // 空循环等待
    }
}

SingleShuntSampler::SingleShuntSampler(adc_channel_t adc_channel, 
                                     float shunt_resistance, 
                                     float op_amp_gain) {
    this->adc_channel = adc_channel;
    this->shunt_resistance = shunt_resistance;
    this->op_amp_gain = op_amp_gain;
    this->adc_offset = 0;
    this->filter_alpha = 0.3f;
    this->adc_handle = NULL;
    this->sample_timer = NULL;
    
    // 初始化采样状态
    _first_sample_ready = false;
    _second_sample_ready = false;
    _sample1 = _sample2 = 0.0f;
    _current_sector = 1;
    
    memset(filtered_currents, 0, sizeof(filtered_currents));
}

SingleShuntSampler::~SingleShuntSampler() {
    if (adc_handle) {
        adc_oneshot_del_unit(adc_handle);
    }
    if (sample_timer) {
        esp_timer_delete(sample_timer);
    }
}

esp_err_t SingleShuntSampler::init() {
    ESP_LOGI(TAG, "初始化单电阻采样器");
    
    // 修复：按照正确的字段顺序初始化
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT, // 必须在ulp_mode之前
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC单元初始化失败: 0x%x", err);
        return err;
    }
    
    // 配置ADC通道
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    
    err = adc_oneshot_config_channel(adc_handle, adc_channel, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC通道配置失败: 0x%x", err);
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return err;
    }
    
    // 创建采样定时器
    esp_timer_create_args_t timer_args = {
        .callback = sampleTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sample_timer",
        .skip_unhandled_events = false
    };
    
    err = esp_timer_create(&timer_args, &sample_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "采样定时器创建失败: 0x%x", err);
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return err;
    }
    
    // 校准ADC偏移
    calibrateOffset();
    
    ESP_LOGI(TAG, "单电阻采样器初始化完成");
    return ESP_OK;
}


void SingleShuntSampler::calibrateOffset() {
    ESP_LOGI(TAG, "开始ADC偏移校准");
    
    const int num_samples = 200;
    int64_t sum = 0;
    
    for (int i = 0; i < num_samples; i++) {
        sum += readADC();
        delay_us(1000);
    }
    
    adc_offset = sum / num_samples;
    ESP_LOGI(TAG, "ADC偏移校准完成: %ld", adc_offset);
}

uint32_t SingleShuntSampler::readADC() {
    const int oversampling = 8;
    uint32_t sum = 0;
    int raw_value = 0;
    
    for (int i = 0; i < oversampling; i++) {
        esp_err_t err = adc_oneshot_read(adc_handle, adc_channel, &raw_value);
        if (err == ESP_OK) {
            sum += raw_value;
        }
        delay_us(1);
    }
    
    return sum / oversampling;
}


float SingleShuntSampler::adcToCurrent(uint32_t adc_value) {
    // 去除偏移
    int32_t corrected_value = (int32_t)adc_value - adc_offset;
    
    // ESP32 ADC参考电压约为1100mV
    float voltage = (corrected_value * 1100.0f) / 4095.0f / 1000.0f;  // 转换为伏特
    
    // 计算电流: I = V_shunt / (R_shunt * G_amp)
    float current = voltage / (shunt_resistance * op_amp_gain);
    
    return current;
}

float SingleShuntSampler::readShuntCurrent() {
    uint32_t adc_value = readADC();
    return adcToCurrent(adc_value);
}

void SingleShuntSampler::sampleTimerCallback(void* arg) {
   
    (void)arg;
    
}

// 触发第一次采样（在PWM周期开始时）
void SingleShuntSampler::triggerFirstSample(uint8_t sector) {
    _current_sector = sector;
    _sample1 = readShuntCurrent();
    _first_sample_ready = true;
    _second_sample_ready = false;
    
    ESP_LOGD(TAG, "第一次采样完成 - 扇区: %d, 电流: %.3fA", sector, _sample1);
}

// 触发第二次采样（在PWM周期中间）
void SingleShuntSampler::triggerSecondSample() {
    if (_first_sample_ready) {
        _sample2 = readShuntCurrent();
        _second_sample_ready = true;
        ESP_LOGD(TAG, "第二次采样完成 - 电流: %.3fA", _sample2);
    } else {
        ESP_LOGW(TAG, "第二次采样被调用，但第一次采样未完成");
    }
}

// 保留第二个areSamplesReady定义，删除第一个定义
bool SingleShuntSampler::areSamplesReady() const {
    if (!_first_sample_ready || !_second_sample_ready) {
        return false;
    }
    
    // 检查采样数据是否过时（超过1个PWM周期）
    static uint32_t last_sample_time = 0;
    uint32_t current_time = esp_timer_get_time();
    if (current_time - last_sample_time > 1000) { // 1ms超时
        return false;
    }
    
    return true;
}

void SingleShuntSampler::reconstructPhaseCurrents(uint8_t sector, float sample1, float sample2) {
    float ia, ib, ic;
    
    // 根据扇区重建三相电流
    switch(sector) {
        case 1: // 扇区1: V100采样Ia, V110采样-Ic
            ia = sample1;           // Ia
            ic = -sample2;          // Ic
            ib = -ia - ic;          // Ib = -Ia - Ic
            break;
            
        case 2: // 扇区2: V110采样Ib, V010采样-Ia
            ib = sample1;           // Ib
            ia = -sample2;          // Ia
            ic = -ia - ib;          // Ic = -Ia - Ib
            break;
            
        case 3: // 扇区3: V010采样-Ic, V011采样Ib
            ic = -sample1;          // Ic
            ib = sample2;           // Ib
            ia = -ib - ic;          // Ia = -Ib - Ic
            break;
            
        case 4: // 扇区4: V011采样Ia, V001采样-Ib
            ia = sample1;           // Ia
            ib = -sample2;          // Ib
            ic = -ia - ib;          // Ic = -Ia - Ib
            break;
            
        case 5: // 扇区5: V001采样-Ic, V101采样Ia
            ic = -sample1;          // Ic
            ia = sample2;           // Ia
            ib = -ia - ic;          // Ib = -Ia - Ic
            break;
            
        case 6: // 扇区6: V101采样Ib, V100采样-Ia
            ib = sample1;           // Ib
            ia = -sample2;          // Ia
            ic = -ia - ib;          // Ic = -Ia - Ib
            break;
            
        default:
            ia = ib = ic = 0.0f;
            ESP_LOGW(TAG, "未知扇区: %d", sector);
            break;
    }
    
    // 应用低通滤波器
    filtered_currents[0] = filter_alpha * ia + (1 - filter_alpha) * filtered_currents[0];
    filtered_currents[1] = filter_alpha * ib + (1 - filter_alpha) * filtered_currents[1];
    filtered_currents[2] = filter_alpha * ic + (1 - filter_alpha) * filtered_currents[2];
    
    ESP_LOGD(TAG, "电流重建 - 扇区%d: Ia=%.3f, Ib=%.3f, Ic=%.3f", 
             sector, filtered_currents[0], filtered_currents[1], filtered_currents[2]);
}

// 改进采样时序控制
// 在 SingleShuntSampler.cpp 中加强电流重建
bool SingleShuntSampler::sampleAndReconstruct(uint8_t sector, float* ia, float* ib, float* ic) {
    if (!areSamplesReady()) {
        return false;
    }
    
    // 立即重建电流，避免数据过时
    reconstructPhaseCurrents(_current_sector, _sample1, _sample2);
    
    // 返回滤波后的电流值
    *ia = filtered_currents[0];
    *ib = filtered_currents[1];
    *ic = filtered_currents[2];
    
    // 立即重置采样状态，准备下一次采样
    _first_sample_ready = false;
    _second_sample_ready = false;
    
    return true;
}


bool SingleShuntSampler::getPhaseCurrents(uint8_t sector, float* ia, float* ib, float* ic) {
    
    
    if (areSamplesReady()) {
        return sampleAndReconstruct(sector, ia, ib, ic);
    } else {
        // 如果没有新的采样数据，返回上次滤波后的值
        *ia = filtered_currents[0];
        *ib = filtered_currents[1];
        *ic = filtered_currents[2];
        return true;
    }
}