#include "driver/adc.h"
#include <driver/gpio.h>

#define EMG_SIG_PIN GPIO_NUM_3
#define EMG_ADC_CHANNEL ADC1_CHANNEL_2

class EMG{
    private:

    public:
        void InitEMG();
        int ReadEMGData();

};