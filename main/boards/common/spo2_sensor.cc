#include "spo2_sensor.h"
#include "board.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include "i2c_device.h"

static const char *TAG = "MAX30102";



// float MAX30102::Read_blood_oxgen();
// int MAX30102::Read_Heartrate();

// 算法相关常量定义
#define BUFFER_SIZE 100
#define MA4_SIZE 4
static int32_t an_x[BUFFER_SIZE];  // 存储IR数据
static int32_t an_y[BUFFER_SIZE];  // 存储Red数据
#define FS 100  // 采样率100Hz

// SpO2查找表
const uint8_t uch_spo2_table[184] = { 
  95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
  99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
  97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
  90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
  80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
  66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
  49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
  28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
  3, 2, 1 
};

// 算法函数声明
static void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
static void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height);
static void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
static void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
static void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size);

// 核心算法函数实现
static void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, 
    uint32_t *pun_red_buffer, 
    int32_t *pn_spo2, int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate, int8_t *pch_hr_valid) 
{
    uint32_t un_ir_mean = 0;
    int32_t k, n_i_ratio_count;
    int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks;
    int32_t an_ir_valley_locs[15];
    int32_t n_peak_interval_sum;
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx = 0, n_x_dc_max_idx = 0;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;

    // 计算IR信号均值并移除DC分量
    for (k = 0; k < n_ir_buffer_length; k++) 
        un_ir_mean += pun_ir_buffer[k];
    un_ir_mean = un_ir_mean / n_ir_buffer_length;

    // 反转信号用于波谷检测
    for (k = 0; k < n_ir_buffer_length; k++)
        an_x[k] = -1 * ((int32_t)pun_ir_buffer[k] - (int32_t)un_ir_mean);

    // 4点移动平均滤波
    for (k = 0; k < n_ir_buffer_length - MA4_SIZE; k++) {
        an_x[k] = (an_x[k] + an_x[k+1] + an_x[k+2] + an_x[k+3]) / 4;
    }

    // 动态阈值计算
    n_th1 = 0;
    for (k = 0; k < n_ir_buffer_length; k++) {
        n_th1 += an_x[k];
    }
    n_th1 = n_th1 / n_ir_buffer_length;
    if (n_th1 < 30) n_th1 = 30;
    if (n_th1 > 60) n_th1 = 60;

    // 检测波峰（对应原始信号的波谷）
    n_npks = 0;
    for (k = 0; k < 15; k++) an_ir_valley_locs[k] = 0;
    maxim_find_peaks(an_ir_valley_locs, &n_npks, an_x, n_ir_buffer_length, n_th1, 4, 15);

    // 计算心率
    n_peak_interval_sum = 0;
    if (n_npks >= 2) {
        for (k = 1; k < n_npks; k++)
            n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k-1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = (int32_t)((FS * 60) / n_peak_interval_sum);
        *pch_hr_valid = 1;
    } else {
        *pn_heart_rate = -999;
        *pch_hr_valid = 0;
    }

    // 重新加载原始数据
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = (int32_t)pun_ir_buffer[k];
        an_y[k] = (int32_t)pun_red_buffer[k];
    }

    // 计算血氧饱和度
    n_i_ratio_count = 0;
    n_exact_ir_valley_locs_count = n_npks;
    for (k = 0; k < 5; k++) an_ratio[k] = 0;

    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;
        if (an_ir_valley_locs[k+1] - an_ir_valley_locs[k] > 3) {
            for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k+1]; i++) {
                if (an_x[i] > n_x_dc_max) {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max) {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }
            
            n_y_ac = (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k]]) * 
                     (n_y_dc_max_idx - an_ir_valley_locs[k]);
            n_y_ac = an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]);
            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;
            
            n_x_ac = (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k]]) * 
                     (n_x_dc_max_idx - an_ir_valley_locs[k]);
            n_x_ac = an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]);
            n_x_ac = an_x[n_x_dc_max_idx] - n_x_ac;
            
            n_nume = (n_y_ac * n_x_dc_max) >> 7;
            n_denom = (n_x_ac * n_y_dc_max) >> 7;
            
            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0) {
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom;
                n_i_ratio_count++;
            }
        }
    }

    // 取中值作为最终比值
    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;
    
    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx-1] + an_ratio[n_middle_idx]) / 2;
    else
        n_ratio_average = an_ratio[n_middle_idx];

    // 通过查找表计算SpO2
    if (n_ratio_average > 2 && n_ratio_average < 184) {
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *pn_spo2 = n_spo2_calc;
        *pch_spo2_valid = 1;
    } else {
        *pn_spo2 = -999;
        *pch_spo2_valid = 0;
    }
}

// 峰值检测辅助函数
static void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, 
                            int32_t n_size, int32_t n_min_height, 
                            int32_t n_min_distance, int32_t n_max_num) 
{
    maxim_peaks_above_min_height(pn_locs, n_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, n_npks, pn_x, n_min_distance);
    *n_npks = std::min(*n_npks, n_max_num);
}

static void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks, 
                                        int32_t *pn_x, int32_t n_size, 
                                        int32_t n_min_height) 
{
    int32_t i = 1, riseFound = 0, holdOff1 = 0, holdOff2 = 0, holdOffThresh = 4;
    *n_npks = 0;

    while (i < n_size - 1) {
        if (holdOff2 == 0) {
            if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]) {
                riseFound = 1;
            }
            if (riseFound == 1) {
                if ((pn_x[i] < n_min_height) && (holdOff1 < holdOffThresh)) {
                    riseFound = 0;
                    holdOff1 = 0;
                } else {
                    if (holdOff1 == holdOffThresh) {
                        if ((pn_x[i] < n_min_height) && (pn_x[i-1] >= n_min_height)) {
                            if ((*n_npks) < 15) {
                                pn_locs[(*n_npks)++] = i;
                            }
                            holdOff1 = 0;
                            riseFound = 0;
                            holdOff2 = 8;
                        }
                    } else {
                        holdOff1++;
                    }
                }
            }
        } else {
            holdOff2--;
        }
        i++;
    }
}

static void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, 
                                    int32_t *pn_x, int32_t n_min_distance) 
{
    int32_t i, j, n_old_npks, n_dist;
    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

    n_old_npks = *pn_npks;
    *pn_npks = 0;
    
    for (i = 0; i < n_old_npks; i++) {
        bool keep = true;
        for (j = 0; j < *pn_npks; j++) {
            n_dist = pn_locs[i] - pn_locs[j];
            if (abs(n_dist) < n_min_distance) {
                keep = false;
                break;
            }
        }
        if (keep) {
            pn_locs[(*pn_npks)++] = pn_locs[i];
        }
    }
    maxim_sort_ascend(pn_locs, *pn_npks);
}

static void maxim_sort_ascend(int32_t *pn_x, int32_t n_size) {
    for (int32_t i = 1; i < n_size; i++) {
        int32_t temp = pn_x[i];
        int32_t j = i;
        while (j > 0 && pn_x[j-1] > temp) {
            pn_x[j] = pn_x[j-1];
            j--;
        }
        pn_x[j] = temp;
    }
}

static void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size) {
    for (int32_t i = 1; i < n_size; i++) {
        int32_t temp = pn_indx[i];
        int32_t j = i;
        while (j > 0 && pn_x[pn_indx[j-1]] < pn_x[temp]) {
            pn_indx[j] = pn_indx[j-1];
            j--;
        }
        pn_indx[j] = temp;
    }
}

MAX30102 max30102_sensor_(I2C_NUM_1, 0x57, GPIO_NUM_MAX);

MAX30102::MAX30102(i2c_port_t port, uint8_t addr, gpio_num_t int_pin) 
    : i2c_port(port), address(addr), int_pin(int_pin), dev_handle(nullptr)
{

    i2c_master_bus_config_t bus_config = {
                .i2c_port = (i2c_port_t)1,
                .sda_io_num = SPO_SDA_PIN,
                .scl_io_num = SPO_SCL_PIN,
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt = 7,
                .intr_priority = 0,
                .trans_queue_depth = 0,
                .flags = {
                    .enable_internal_pullup = 1,
                },
            };


    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &spo_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    
    if (spo_bus != nullptr) {
        ESP_ERROR_CHECK(i2c_master_bus_add_device(spo_bus, &dev_cfg, &dev_handle));
    } else {
        ESP_LOGE(TAG, "I2C bus handle is null!");
    }
}

MAX30102::~MAX30102() 
{
    i2c_master_bus_rm_device(dev_handle);
    // i2c_del_master_bus(bus_handle);
}

bool MAX30102::writeRegister(uint8_t reg, uint8_t value)
{
    // uint8_t write_buf[2] = {reg, value};
    
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_write(cmd, write_buf, sizeof(write_buf), true);
    // i2c_master_stop(cmd);
    
    // esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    // i2c_cmd_link_delete(cmd);// TODO: CHeck
    
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "I2C write error: %d", ret);
    //     return false;
    // }
    // return true;
    uint8_t write_buf[2] = {reg, value};
    
    esp_err_t ret = i2c_master_transmit(
        dev_handle,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(1000)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write error: %d", ret);
        return false;
    }
    return true;
}

bool MAX30102::readRegister(uint8_t reg, uint8_t *value)
{
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd, reg, true);
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    // i2c_master_read_byte(cmd, value, I2C_MASTER_LAST_NACK);
    // i2c_master_stop(cmd);
    
    // esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    // i2c_cmd_link_delete(cmd); // TODO: Check
    
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "I2C read error: %d", ret);
    //     return false;
    // }
    // return true;
    esp_err_t ret = i2c_master_transmit_receive(
        dev_handle,
        &reg,
        1,
        value,
        1,
        pdMS_TO_TICKS(1000)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error: %d", ret);
        return false;
    }
    return true;
}

bool MAX30102::readFIFOBytes(uint8_t reg, uint8_t *data, size_t len)
{
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd, reg, true);
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    // if (len > 1) {
    //     i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    // }
    // i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_LAST_NACK);
    // i2c_master_stop(cmd);
    
    // esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    // i2c_cmd_link_delete(cmd); // 
    
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "FIFO read error: %d", ret);
    //     return false;
    // }
    // return true;
    esp_err_t ret = i2c_master_transmit_receive(
        dev_handle,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(1000)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FIFO read error: %d", ret);
        return false;
    }
    return true;
}

bool MAX30102::begin()
{
    // Initialize interrupt pin
    if (int_pin != GPIO_NUM_MAX) {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << int_pin);
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
    }
    
    // Reset device
    if (!reset()) {
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Verify device ID
    uint8_t id;
    if (!readRegister(REG_PART_ID, &id)) {
        return false;
    }
    
    if (id != 0x15) { // MAX30102 PART ID is 0x15
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X", id);
        return false;
    }
    
    return true;
}

bool MAX30102::reset()
{
    return writeRegister(REG_MODE_CONFIG, 0x40); // Set reset bit
}



bool MAX30102::getData( bool *spo2Valid, bool *hrValid)
{
    const int buffer_length = 100; // 5秒的数据，100Hz采样率
    uint32_t red_buffer[buffer_length];
    uint32_t ir_buffer[buffer_length];

    // 重置配置
    if (!reset()) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 配置传感器
    if (!writeRegister(REG_INTR_ENABLE_1, 0xC0)) return false; // INTR setting
    if (!writeRegister(REG_INTR_ENABLE_2, 0x00)) return false;
    if (!writeRegister(REG_FIFO_WR_PTR, 0x00)) return false;  // FIFO_WR_PTR[4:0]
    if (!writeRegister(REG_OVF_COUNTER, 0x00)) return false;   // OVF_COUNTER[4:0]
    if (!writeRegister(REG_FIFO_RD_PTR, 0x00)) return false;   // FIFO_RD_PTR[4:0]
    if (!writeRegister(REG_FIFO_CONFIG, 0x4F)) return false;   // sample avg = 1, fifo rollover=false, 
    if (!writeRegister(REG_MODE_CONFIG, 0x03)) return false;   // SpO2 mode
    if (!writeRegister(REG_SPO2_CONFIG, 0x27)) return false;   // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    if (!writeRegister(REG_LED1_PA, 0xFF)) return false;       // ~10mA for LED1
    if (!writeRegister(REG_LED2_PA, 0xFF)) return false;       // ~10mA for LED2      // ~7mA for LED2
    if (!writeRegister(REG_PILOT_PA, 0x7F)) return false;      // ~25mA for Pilot LED
    
    // 收集数据
    //uint32_t un_min = 0x3FFFF;
    //uint32_t un_max = 0;

    // 等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 清除中断状态
    uint8_t int_status;
    readRegister(REG_INTR_STATUS_1, &int_status);
    readRegister(REG_INTR_STATUS_2, &int_status);
    
   for (int i = 0; i < buffer_length; i++) {
        if (int_pin != GPIO_NUM_MAX) {
            while (gpio_get_level(int_pin) == 1) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        uint8_t temp[6];
        if (!readFIFOBytes(REG_FIFO_DATA, temp, sizeof(temp))) {
            return false;
        }
        
        red_buffer[i] = ((uint32_t)(temp[0] & 0x03) << 16) | 
                        ((uint32_t)temp[1] << 8) | temp[2];
        ir_buffer[i] = ((uint32_t)(temp[3] & 0x03) << 16) | 
                       ((uint32_t)temp[4] << 8) | temp[5];
                              
        // 清除中断
        readRegister(REG_INTR_STATUS_1, &int_status);
    }
    
    // 使用专业算法计算心率和血氧
    int32_t spo2_val;
    int32_t heart_rate_val;
    int8_t spo2_valid_flag, hr_valid_flag;
    
    maxim_heart_rate_and_oxygen_saturation(
        ir_buffer, buffer_length, 
        red_buffer, 
        &spo2_val, &spo2_valid_flag, 
        &heart_rate_val, &hr_valid_flag
    );
    
    // 更新结果
    this->heartRate = heart_rate_val;
    this->spo2 = static_cast<float>(spo2_val);
    *hrValid = (hr_valid_flag == 1);
    *spo2Valid = (spo2_valid_flag == 1);
    
    return true;
}



float MAX30102::Read_blood_oxgen(){
    return spo2;
}

int MAX30102::Read_Heartrate(){
    return heartRate;
}

// MAX30102* Board::GetMAX30102() {
//     return &max30102_sensor_;
// }
//   // 返回实际的传感器对象