# MQTT 功能使用说明

## 概述
此代码为ESP32+EG915U模组添加了完整的MQTT客户端功能，支持通过4G网络连接到MQTT服务器并发送数据。

## 配置文件说明

### 修改 `mqtt_config.h` 文件
在使用前，您需要修改 `main/mqtt_config.h` 文件中的配置信息：

```c
// 1. MQTT服务器信息
#define MQTT_BROKER_HOST    "your_mqtt_broker.com"      // 替换为您的MQTT服务器地址
#define MQTT_BROKER_PORT    1883                        // MQTT端口号

// 2. 认证信息
#define MQTT_CLIENT_ID      "esp32_pe_tracker_001"      // 客户端ID，确保唯一
#define MQTT_USERNAME       "your_username"             // MQTT用户名
#define MQTT_PASSWORD       "your_password"             // MQTT密码

// 3. 网络APN（根据您的运营商设置）
#define APN_NAME            "your_apn"                  // 如: "cmnet", "3gnet", "ctnet"
```

## 功能特性

### 1. 自动网络初始化
- 自动设置4G网络参数
- 配置APN并激活PDP上下文
- 获取IP地址

### 2. MQTT连接管理
- 自动连接MQTT服务器
- 支持用户名密码认证
- 自动重连机制
- 连接状态监控

### 3. 数据发布
- **状态主题**: `tracker/status` - 设备上线/离线状态
- **传感器主题**: `tracker/sensor` - ADC数据等传感器信息
- **GPS主题**: `tracker/gps` - GPS位置信息（预留）
- **数据主题**: `tracker/data` - 其他数据

### 4. 触发方式
- **按键触发**: 按下IO0按键启动MQTT连接
- **定时发送**: 每10秒自动发送一次传感器数据
- **状态监控**: 每5秒检查AT指令状态

## 使用方法

### 1. 配置步骤
1. 编辑 `main/mqtt_config.h` 文件
2. 填入正确的MQTT服务器信息
3. 设置正确的APN名称
4. 编译并烧录程序

### 2. 运行测试
1. 程序启动后会自动初始化UART和定时器
2. 按下IO0按键触发MQTT连接
3. 观察串口日志，查看连接状态
4. 成功连接后会自动发送状态消息和传感器数据

### 3. 日志监控
```
I (xxx) MQTT: Initializing 4G network...
I (xxx) MQTT: 4G network initialized successfully
I (xxx) MQTT: Connecting to MQTT broker...
I (xxx) MQTT: Connected to MQTT broker successfully
I (xxx) MQTT: Publishing to topic: tracker/status
I (xxx) MQTT: Message published successfully
```

## MQTT消息格式

### 状态消息 (tracker/status)
```json
{
    "status": "online",
    "device": "pe_tracker"
}
```

### 传感器数据 (tracker/sensor)
```json
{
    "adc": 2.35,
    "timestamp": 12345678
}
```

## 故障排除

### 1. 网络连接失败
- 检查APN设置是否正确
- 确认SIM卡已激活数据服务
- 查看信号强度 (AT+CSQ)

### 2. MQTT连接失败
- 确认服务器地址和端口正确
- 检查用户名密码
- 确认客户端ID唯一性

### 3. 发布消息失败
- 检查主题名称格式
- 确认网络连接稳定
- 查看QoS设置

## 自定义扩展

### 添加新的主题
在 `mqtt_config.h` 中添加：
```c
#define MQTT_TOPIC_CUSTOM   MQTT_TOPIC_PREFIX "/custom"
```

### 修改发布内容
在 `mqtt_task` 函数中自定义消息格式：
```c
char custom_data[256];
snprintf(custom_data, sizeof(custom_data), 
        "{\"temperature\":%.1f,\"humidity\":%.1f}", 
        temperature, humidity);
mqtt_publish_message(MQTT_TOPIC_CUSTOM, custom_data);
```

### 调整发布频率
修改定时器回调中的计数器：
```c
if (counter % 300 == 0)  // 每30秒发送一次 (300 * 100ms = 30s)
{
    mqtt_publish_flag = true;
}
```

## 注意事项

1. **内存使用**: MQTT任务使用4KB栈空间，如需更多功能可适当增加
2. **网络稳定性**: 4G网络可能不稳定，代码已包含重连机制
3. **功耗考虑**: 持续的4G连接会增加功耗，可考虑添加休眠机制
4. **安全性**: 建议使用SSL/TLS加密连接（端口8883）
5. **QoS级别**: 当前使用QoS 1，确保消息至少送达一次

## 扩展建议

1. 添加GPS数据获取和发布
2. 实现MQTT订阅功能，接收远程命令
3. 添加数据缓存机制，网络断开时本地存储
4. 实现OTA固件更新功能
5. 添加设备配置远程管理