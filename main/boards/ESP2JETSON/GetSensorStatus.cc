#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include "config.h"
#include <esp_adc/adc_oneshot.h>
#include "GetSensorStatus.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include <driver/gpio.h>
#include <esp_rom_sys.h>

GetSensorStatus::GetSensorStatus(gpio_num_t hyper_echo_pin,
        gpio_num_t hyper_trig_pin,
        adc_unit_t adc_unit,
        adc_channel_t adc_channel)
    : hyper_echo_pin_(hyper_echo_pin),
    hyper_trig_pin_(hyper_trig_pin),
    adc_unit_(adc_unit),
    adc_channel_(adc_channel),
    adc_handle_(nullptr)  // 初始化ADC句柄
{
        // 分别配置两个GPIO
        gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << hyper_echo_pin_),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&echo_conf);

        gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << hyper_trig_pin_),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&trig_conf);
        gpio_set_level(hyper_trig_pin_, 0);

        // 使用成员变量初始化ADC
        InitializeAdc();
}


GetSensorStatus::~GetSensorStatus() {
    if (timer_handle_) {
        esp_timer_stop(timer_handle_);
        esp_timer_delete(timer_handle_);
    }
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
}


void GetSensorStatus::InitHCSR04(){

    // 
    gpio_set_direction(hyper_trig_pin_, GPIO_MODE_OUTPUT);
    gpio_set_direction(hyper_echo_pin_, GPIO_MODE_INPUT);

    gpio_set_level(hyper_trig_pin_, 0);
    esp_rom_delay_us(2);
    gpio_set_level(hyper_trig_pin_, 1);
    esp_rom_delay_us(10);
    gpio_set_level(hyper_trig_pin_, 0);
}
// void GetSensorStatus::ReadHCSR04Data(){  
//     uint64_t start_time = esp_timer_get_time();
//     while (!gpio_get_level(hyper_echo_pin_)); // Wait for HIGH
//     uint64_t echo_start = esp_timer_get_time();
//     while (gpio_get_level(hyper_echo_pin_)); // Wait for LOW
//     uint64_t echo_end = esp_timer_get_time();
// }
// float GetSensorStatus::CalculateHCSR04Data(uint64_t echo_start, uint64_t echo_end){
//     uint64_t duration = echo_end - echo_start;
//     float distance = (duration * 0.034) / 2;

//     return distance;
// }

// 修复超声波测量函数
float GetSensorStatus::ReadDistance() {
    // 发送触发脉冲
    gpio_set_level(hyper_trig_pin_, 0);
    esp_rom_delay_us(2);
    gpio_set_level(hyper_trig_pin_, 1);
    esp_rom_delay_us(10);
    gpio_set_level(hyper_trig_pin_, 0);

    // 等待回波上升沿（带超时）
    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(hyper_echo_pin_)) {
        if (esp_timer_get_time() - start > 30000) { // 30ms超时
            return -1.0f; // 超时错误
        }
    }
    uint64_t echo_start = esp_timer_get_time();

    // 等待回波下降沿（带超时）
    start = esp_timer_get_time();
    while (gpio_get_level(hyper_echo_pin_)) {
        if (esp_timer_get_time() - start > 30000) {
            return -1.0f;
        }
    }
    uint64_t echo_end = esp_timer_get_time();

    // 计算距离 (cm)
    uint64_t duration = echo_end - echo_start;
    return (duration * 0.0343) / 2.0f;  // 更精确的声速
}

float GetSensorStatus::ReadHeartBeatRate() {
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_channel_, &raw));
    
    // 转换为电压 (假设参考电压 3.3V)
    float voltage = raw * (3.3f / 4095.0f);
    
    // TODO: 实际项目需要校准电压到心率值的转换
    // 简单模拟：电压 1.0-2.0V 对应心率 50-120 BPM
    float heartRate = 50.0f + (voltage - 1.0f) * 70.0f;
    
    // 限制心率范围
    if (heartRate < 40.0f) heartRate = 40.0f;
    if (heartRate > 160.0f) heartRate = 160.0f;
    
    return heartRate;
}

void GetSensorStatus::InitializeAdc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit_,  // 使用成员变量
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, adc_channel_, &chan_config));
}