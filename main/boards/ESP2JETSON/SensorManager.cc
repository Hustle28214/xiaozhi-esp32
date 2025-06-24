#include "GetSensorStatus.h"
#include <esp_log.h>
#include <cstring>
#include "application.h"
#include "board.h"
#include "config.h"
#include "iot/thing.h"
#include "sdkconfig.h"

// 使用唯一TAG避免重定义
static const char* SENSOR_TAG = "SensorSystem";

namespace iot {
    class Sensor : public Thing {
    private:
        GetSensorStatus* sensor_reader_;  // 添加传感器读取器
        TaskHandle_t task_handle_ = nullptr;
        QueueHandle_t action_queue_;

    public:
        Sensor(GetSensorStatus* sensor) : Thing("Sensor", "传感器数据"), 
                  sensor_reader_(sensor)  // 初始化传感器
        {
            methods_.AddMethod("speak", "超声波检测", 
                ParameterList({Parameter("text", "播报内容", kValueTypeString, true)}),
                [this](const ParameterList& params) {
                    float distance = sensor_reader_->ReadDistance();
                    
                    if (distance < 30.0f) {
                        std::string greeting = "Hello, welcome!";
                        ESP_LOGI(SENSOR_TAG, "Person detected (%.1fcm), play: %s", 
                                distance, greeting.c_str());
                    } else {
                        ESP_LOGI(SENSOR_TAG, "No person (%.1fcm > 30cm)", distance);
                    }
                });

            methods_.AddMethod("heartbeat", "心跳检测", 
                ParameterList({Parameter("text", "播报内容", kValueTypeString, true)}),
                [this](const ParameterList& params) {
                    float heartRate = sensor_reader_->ReadHeartBeatRate();
                    std::string feedback;
                    
                    if (heartRate < 60.0f) {
                        feedback = "Heart rate too low, please rest";
                    } else if (heartRate > 100.0f) {
                        feedback = "Heart rate too high, please relax";
                    } else {
                        feedback = "Heart rate normal";
                    }
                    
                    ESP_LOGI(SENSOR_TAG, "Heart: %.1f BPM, feedback: %s", 
                            heartRate, feedback.c_str());
                });

            methods_.AddMethod("SetVolume", "设置音量", 
                ParameterList({Parameter("volume", "0-100", kValueTypeNumber, true)}),
                [this](const ParameterList& params) {
                    auto codec = Board::GetInstance().GetAudioCodec();
                    int volume = params["volume"].number();
                    volume = (volume > 100) ? 100 : volume;
                    codec->SetOutputVolume(static_cast<uint8_t>(volume));
                });
        }

        ~Sensor() {
            delete sensor_reader_;  // 释放资源
            if (task_handle_) {
                vTaskDelete(task_handle_);
            }
            if (action_queue_) {
                vQueueDelete(action_queue_);
            }
        }
    };
}

// 删除重复的 Sensor 定义
static iot::Sensor* g_sensor = nullptr;

// 修改初始化函数，接受传感器参数
void InitializeSensor(GetSensorStatus* sensor) {
    if (!g_sensor) {
        g_sensor = new iot::Sensor(sensor);
        ESP_LOGI(SENSOR_TAG, "Sensor initialized");
    }
}