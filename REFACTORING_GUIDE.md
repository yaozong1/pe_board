# PE Board 项目重构说明

## 项目结构重构

原始的 `pe.c` 文件（1823行）已经被重构为多个模块化的文件，提高了代码的可读性和可维护性。

## 新的文件结构

### 头文件 (.h)
- `hardware_config.h` - 硬件配置和引脚定义
- `gnss_module.h` - GNSS模块接口
- `mqtt_module.h` - MQTT模块接口  
- `battery_module.h` - 电池/ADC模块接口
- `gpio_module.h` - GPIO控制模块接口
- `timer_module.h` - 定时器模块接口

### 源文件 (.c)
- `main.c` - 主程序入口，包含app_main()
- `gnss_module.c` - GNSS功能实现（NMEA解析，低功耗管理，TTFF测量）
- `mqtt_module.c` - MQTT功能实现（AT命令，网络连接，数据发布）
- `battery_module.c` - 电池监控功能实现（ADC读取，电压转换）
- `gpio_module.c` - GPIO控制功能实现（LED，按键，电源控制）
- `timer_module.c` - 定时器功能实现（100ms定时器，按键检测）

### 配置文件
- `mqtt_config.h` - MQTT服务器配置（保持不变）

### 备份文件
- `pe_original.c` - 原始pe.c文件的备份

## 模块功能分解

### 1. Hardware Config (hardware_config.h)
- 所有硬件引脚定义
- UART配置参数
- ADC配置参数
- 功能开关宏定义
- 调试日志控制宏

### 2. GNSS Module (gnss_module.c/.h)
- NMEA句子解析 (RMC, GGA, GSA)
- UBX二进制协议支持
- Quectel专用命令支持
- 低功耗管理 (Standby, Backup模式)
- TTFF (Time To First Fix) 测量
- 位置数据结构管理

### 3. MQTT Module (mqtt_module.c/.h)
- AT命令处理
- 4G网络初始化
- MQTT连接管理
- 数据发布功能
- 电源管理（EG915U控制）
- GPS+MQTT周期任务

### 4. Battery Module (battery_module.c/.h)
- ADC初始化和读取
- 电池电压计算
- SOC (State of Charge) 估算
- 多次采样平均

### 5. GPIO Module (gpio_module.c/.h)
- GPIO初始化
- LED控制函数
- 按键状态检测
- 电源控制引脚管理

### 6. Timer Module (timer_module.c/.h)
- 100ms定时器配置
- 按键检测处理
- 周期性任务触发
- MQTT发布定时控制

## 主要优势

1. **模块化设计** - 每个功能模块独立，便于维护和测试
2. **清晰的接口** - 通过头文件提供清晰的API接口
3. **代码复用** - 模块可以在其他项目中复用
4. **易于调试** - 每个模块可以独立测试和调试
5. **可扩展性** - 新功能可以作为新模块添加
6. **可读性** - 代码结构清晰，功能分组明确

## 编译配置

`CMakeLists.txt` 已更新为包含所有新的源文件：
```cmake
idf_component_register(SRCS "main.c"
                           "gnss_module.c"
                           "mqtt_module.c" 
                           "battery_module.c"
                           "gpio_module.c"
                           "timer_module.c"
                    INCLUDE_DIRS "")
```

## 使用说明

### 初始化顺序
1. GPIO初始化
2. GNSS模块初始化 
3. MQTT模块初始化
4. 电池监控初始化
5. 定时器初始化并启动
6. 创建GNSS任务
7. 创建GPS+MQTT周期任务

### 主要任务
- `gnss_task` - 处理GNSS数据接收和解析
- `gps_mqtt_cycle_task` - 执行定位→MQTT上报→睡眠的周期

### 功能特性
- 按键触发的调制解调器重启
- 定时的AT命令发送
- 周期性的MQTT数据发布
- 电池状态监控
- GNSS低功耗管理

这个重构提供了一个更加模块化、可维护的代码结构，同时保持了原有的所有功能。