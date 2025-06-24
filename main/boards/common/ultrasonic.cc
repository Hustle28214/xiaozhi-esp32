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
#include "esp_timer.h"


#define HCSR04_Trig_Pin GPIO_NUM_8
#define HCSR04_Echo_Pin GPIO_NUM_18 

void Ultrasonic::InitHCSR04(){
    // init sensor
    gpio_set_direction(HCSR04_Trig_Pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(HCSR04_Trig_Pin, GPIO_MODE_INPUT);

    // gpio_set_level(HCSR04_HCSR04_Trig_Pin, 0);
    // esp_rom_delay_us(2);
    // gpio_set_level(HCSR04_HCSR04_Trig_Pin, 1);
    // esp_rom_delay_us(10);
    // gpio_set_level(HCSR04_HCSR04_Trig_Pin, 0);
}


int Ultrasonic::ReadDistance() {
    InitHCSR04();
    gpio_set_level(HCSR04_Trig_Pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(HCSR04_Trig_Pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HCSR04_Trig_Pin, 0);

    // Measure echo pulse width
    uint64_t start_time = esp_timer_get_time();
    while (!gpio_get_level(HCSR04_Echo_Pin)); // Wait for HIGH
    uint64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(HCSR04_Echo_Pin)); // Wait for LOW
    uint64_t echo_end = esp_timer_get_time();

    uint64_t duration = echo_end - echo_start;
    int distance = (duration * 0.034) / 2;

    vTaskDelay(pdMS_TO_TICKS(500));
    return distance;
}
