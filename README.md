# 基于小智ai和MCP协议的常见传感器状态反馈

板子：ESP32S3
烧录方式：串口

此仓库是基于bread-compact-wifi的引脚排布改进而来，加入了HC-SR04等常见传感器的状态反馈功能，已经烧录验证。

请打开Menuconfig-Xiaozhi Assistant选择烧录ESP2JETSON，其他地方保持默认。


# 主要改动

> main/boards/ESP2JETSON下：
ESP2JETSON.cc: 初始化I2C、外设以及IOT设置
GetSensorStatus.cc：初始化传感器的GPIO和中断设置
SensorManager.cc：已经弃用（不再使用旧的IOT方法，已经迁移到MCP协议实现），目前只调取了设置音量的iot方法

> main/mcp_server.cc:
使用MCP协议获取超声波传感器数据：
```cpp
AddTool("self.get_ultrasonic_sensor_status",
        "提供了实时的超声波传感器数据，返回值为人到机械臂的距离，单位为毫米\n"
        "使用这个功能给下面的条件: \n"
        "1. Answering questions about current condition (e.g. what is the current volume of the audio speaker?)\n"
        "2. As the first step to control the device (e.g. turn up / down the volume of the audio speaker, etc.)",
        PropertyList(),
        [&board](const PropertyList& properties) -> ReturnValue {
            auto ultra =  board.GetUltrasonic() -> ReadDistance();
            return ultra;
        });
```

> main/boards/common/ultrasonic.cc:
添加了超声波传感器引脚状态初始化和距离计算方法。

# 未来计划

添加MAX30102心率血氧传感器(I2C,0x59)和sEMG肌肉电传感器(模拟输入)的状态获取工具。

# 接线（正在更新）

[接线图](main\boards\ESP2JETSON\接线.png)