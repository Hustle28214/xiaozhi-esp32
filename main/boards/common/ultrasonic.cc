#include <esp_log.h>
#include <cstring>
#include "application.h"
#include "board.h"
#include "ultrasonic.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include <driver/gpio.h>
#include <esp_rom_sys.h>

#define HCSR04_Trig_Pin GPIO_NUM_8
#define HCSR04_Echo_Pin GPIO_NUM_18 

void Ultrasonic::InitHCSR04(){
    // init sensor
    gpio_set_direction(HCSR04_Trig_Pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(HCSR04_Trig_Pin, GPIO_MODE_INPUT);

    gpio_set_level(HCSR04_Trig_Pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(HCSR04_Trig_Pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HCSR04_Trig_Pin, 0);
}


float Ultrasonic::ReadDistance() {
    // 发送触发脉冲
    gpio_set_level(HCSR04_Trig_Pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(HCSR04_Trig_Pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HCSR04_Trig_Pin, 0);

    // 等待回波上升沿（带超时）
    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(HCSR04_Echo_Pin)) {
        if (esp_timer_get_time() - start > 30000) { // 30ms超时
            return -1.0f; // 超时错误
        }
    }
    uint64_t echo_start = esp_timer_get_time();

    // 等待回波下降沿（带超时）
    start = esp_timer_get_time();
    while (gpio_get_level(HCSR04_Echo_Pin)) {
        if (esp_timer_get_time() - start > 30000) {
            return -1.0f;
        }
    }
    uint64_t echo_end = esp_timer_get_time();

    // 计算距离 (cm)
    uint64_t duration = echo_end - echo_start;
    return (duration * 0.0343) / 2.0f;  // 更精确的声速
}
