#ifndef _MAX30102_H_
#define _MAX30102_H_

#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#define MAX30102_I2C_ADDRESS 0x57

// 寄存器地址定义
#define REG_INTR_STATUS_1    0x00
#define REG_INTR_STATUS_2    0x01
#define REG_INTR_ENABLE_1    0x02
#define REG_INTR_ENABLE_2    0x03
#define REG_FIFO_WR_PTR      0x04
#define REG_OVF_COUNTER      0x05
#define REG_FIFO_RD_PTR      0x06
#define REG_FIFO_DATA        0x07
#define REG_FIFO_CONFIG      0x08
#define REG_MODE_CONFIG      0x09
#define REG_SPO2_CONFIG      0x0A
#define REG_LED1_PA          0x0C
#define REG_LED2_PA          0x0D
#define REG_PILOT_PA         0x10
#define REG_MULTI_LED_CTRL1  0x11
#define REG_MULTI_LED_CTRL2  0x12
#define REG_TEMP_INTR        0x1F
#define REG_TEMP_FRAC        0x20
#define REG_TEMP_CONFIG      0x21
#define REG_PROX_INT_THRESH  0x30
#define REG_REV_ID           0xFE
#define REG_PART_ID          0xFF


#define GPIO_NUM_MAX GPIO_NUM_13
#define SPO_SDA_PIN GPIO_NUM_14
#define SPO_SCL_PIN GPIO_NUM_21

class MAX30102
{
private:
    i2c_port_t i2c_port;
    uint8_t address;
    gpio_num_t int_pin;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t spo_bus;
    float spo2;
    int heartRate;
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool readFIFO(uint32_t *red, uint32_t *ir);

public:
    MAX30102(i2c_port_t port = (i2c_port_t)1, uint8_t addr = MAX30102_I2C_ADDRESS, gpio_num_t int_pin = GPIO_NUM_MAX);
    ~MAX30102();
    
    bool begin();
    bool reset();
    bool getData( bool *spo2Valid, bool *hrValid);
    bool readFIFOBytes(uint8_t reg, uint8_t *data, size_t len);
    float Read_blood_oxgen();
    int Read_Heartrate();
};

#endif