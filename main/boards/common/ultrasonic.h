#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <driver/gpio.h>

class Ultrasonic
{
private:
    /* data */
public:
    // Ultrasonic(/* args */);
    // ~ultrasonic();
    void InitHCSR04();
    int ReadDistance();
};



#endif
