#include <stdio.h>
#include <EMG.h>
#include <driver/gpio.h>
#include <driver/adc.h>



// ADC1_CH2

void EMG::InitEMG(){
    gpio_set_direction(EMG_GPIO,GPIO_MODE_INPUT);
    adc1_config_width(ADC_BITWIDTH_12);
    adc1_config_channel_atten(EMG_ADC_CHANNEL,ADC_ATTEN_DB_11);
}

int EMG::ReadEMGData(){
    InitEMG();
    int analogInput = 0;
    analogInput = adc1_get_raw(EMG_ADC_CHANNEL);
    float voltage = analogInput * (3.3f / 4095.0f);

    return voltage;
}


