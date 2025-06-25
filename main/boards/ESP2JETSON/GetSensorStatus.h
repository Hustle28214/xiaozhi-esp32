#ifndef __GETSENSORSTATUS_H__
#define __GETSENSORSTATUS_H__

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include "config.h"
#include <esp_adc/adc_oneshot.h>

#define TAG "GetSensorStatus"


class GetSensorStatus {

    private:
        esp_timer_handle_t timer_handle_ = nullptr;
        gpio_num_t hyper_echo_pin_;
        gpio_num_t hyper_trig_pin_;
        gpio_num_t emg_sig_pin;
        adc_unit_t adc_unit_;
        adc_channel_t adc_channel_;
        adc_oneshot_unit_handle_t adc_handle_;
    // 超声波HC-SR04的读取函数
    public:
        void InitHCSR04();
        // void ReadHCSR04Data();
        // void CalculateHCSR04Data(uint64_t echo_start, uint64_t echo_end);
        float ReadDistance();
        // 心跳传感器KY-039,获取电压
        float ReadHeartBeatRate();
        // TODO: 构造函数该怎么声明？
        GetSensorStatus(gpio_num_t hyper_echo_pin, gpio_num_t hyper_trig_pin,
            adc_unit_t adc_unit, adc_channel_t adc_channel);
        ~GetSensorStatus();
        void InitializeAdc();

        
};

#endif // __GETSENSORSTATUS_H__