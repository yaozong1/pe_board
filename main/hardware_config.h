#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// 硬件引脚定义
#ifndef LED_PIN_EN_GNSS
#define LED_PIN_EN_GNSS 46  // IO46（同时作为 GNSS 使能脚）
#endif
#ifndef LED_PIN_RST_GNSS
#define LED_PIN_RST_GNSS 47  // IO47 -> 通过三极管反相驱动 RESET_N
#endif
#ifndef QT_POWER_PIN
#define QT_POWER_PIN 18      // EG915U PWRKEY 控制脚
#endif

#ifndef LED_PIN_1
// 注意：GPIO1 被用于 ADC1_CHANNEL_0（电池采样），避免冲突，这里改用 GPIO2 作为 LED1
#define LED_PIN_1 2
#endif
#ifndef LED_PIN_3
#define LED_PIN_3 3
#endif
#ifndef BUTTON_PIN
#define BUTTON_PIN 0
#endif
#ifndef POWER_EN_4V
#define POWER_EN_4V 16
#endif
#ifndef LED_PIN_EN_GPS
#define LED_PIN_EN_GPS 5
#endif
#ifndef LED_PIN_EN_3V3_LDO
#define LED_PIN_EN_3V3_LDO 6
#endif
#ifndef RS_EN
#define RS_EN 7
#endif

// 运动传感器引脚配置 (LIS2DH12TR)
#ifndef MOTION_SENSOR_SDA_PIN
#define MOTION_SENSOR_SDA_PIN 14    // I2C SDA
#endif
#ifndef MOTION_SENSOR_SCL_PIN
#define MOTION_SENSOR_SCL_PIN 15    // I2C SCL
#endif

// UART 配置
#ifndef EX_UART_NUM
#define EX_UART_NUM UART_NUM_2   // GNSS 使用 UART2
#endif
#ifndef BUF_SIZE
#define BUF_SIZE 1024
#endif

#ifndef UART_EG915U_NUM
#define UART_EG915U_NUM UART_NUM_1 // EG915U模组 UART (IO11, IO12)
#endif
#ifndef EG915U_TX_PIN
#define EG915U_TX_PIN 11  // ESP32的TX连接到EG915U
#endif
#ifndef EG915U_RX_PIN
#define EG915U_RX_PIN 12  // ESP32的RX连接到EG915U
#endif
#ifndef EG915U_POWER
#define EG915U_POWER 48   // IO48 读取 EG915U 上电状态
#endif

// GNSS UART 配置
#define GNSS_UART_NUM EX_UART_NUM
#define GNSS_BAUD 115200
#define GNSS_UART_TX 42  // ESP32 TX 输出到 GNSS RXD (IO42)
#define GNSS_UART_RX 45  // ESP32 RX 接 GNSS TXD (IO45)
#define GNSS_EN_PIN LED_PIN_EN_GNSS
#define GNSS_RST_PIN LED_PIN_RST_GNSS

// ADC 配置
#define ADC1_CHANNEL ADC1_CHANNEL_0  // IO1 对应 ADC1_CHANNEL_0
#define NO_OF_SAMPLES 64  // 采样数

// 功能开关
#ifndef ENABLE_MQTT
#define ENABLE_MQTT 1
#endif

#ifndef ENABLE_GNSS_LP
#define ENABLE_GNSS_LP 0              // 关闭旧的低功耗循环，改用周期上报任务
#endif

// EG915 AT 自测开关：置为 1 时仅运行握手自测任务，便于独立验证串口与模组联通
#ifndef ENABLE_EG915_AT_SELFTEST
#define ENABLE_EG915_AT_SELFTEST 1
#endif

// 通用 UNUSED_ATTR，避免静态函数/变量未用告警
#ifndef UNUSED_ATTR
#define UNUSED_ATTR __attribute__((unused, used))
#endif

// GNSS 调试配置
#ifndef GNSS_RAW_LOG_ENABLED
#define GNSS_RAW_LOG_ENABLED 0
#endif

#ifndef GNSS_UBX_LOG_ENABLED
#define GNSS_UBX_LOG_ENABLED 0
#endif

// GNSS 解析日志控制 (控制RMC/GGA/GSA解析结果输出)
#ifndef GNSS_PARSE_LOG_ENABLED
#define GNSS_PARSE_LOG_ENABLED 0  // 默认关闭，减少日志输出
#endif

#if GNSS_RAW_LOG_ENABLED
#define GNSS_RAW_LOG(fmt, ...) ESP_LOGI("GNSS_RAW", fmt, ##__VA_ARGS__)
#else
#define GNSS_RAW_LOG(...) do { } while (0)
#endif

#if GNSS_UBX_LOG_ENABLED
#define GNSS_UBX_LOG(fmt, ...) ESP_LOGI("GNSS_UBX", fmt, ##__VA_ARGS__)
#else
#define GNSS_UBX_LOG(...) do { } while (0)
#endif

#if GNSS_PARSE_LOG_ENABLED
#define GNSS_PARSE_LOG(fmt, ...) ESP_LOGI("GNSS", fmt, ##__VA_ARGS__)
#else
#define GNSS_PARSE_LOG(...) do { } while (0)
#endif

// 低功耗测试参数
#ifndef GNSS_LP_OFF_SEC
#define GNSS_LP_OFF_SEC 60            // 关断时长（秒）
#endif
#ifndef GNSS_TTFF_TIMEOUT_SEC
#define GNSS_TTFF_TIMEOUT_SEC 120     // TTFF 超时（秒）
#endif
#ifndef GNSS_LP_ON_HOLD_SEC
#define GNSS_LP_ON_HOLD_SEC 15        // Fix 后继续保持运行时间（秒）
#endif
#ifndef GNSS_LP_METHOD
#define GNSS_LP_METHOD 2
#endif

// GPS 上报周期参数
#ifndef GPS_CYCLE_SLEEP_MIN
#define GPS_CYCLE_SLEEP_MIN 6              // 睡眠 6 分钟
#endif
#ifndef GPS_FIX_TIMEOUT_SEC
#define GPS_FIX_TIMEOUT_SEC 300            // 5 分钟定位超时
#endif
#ifndef MQTT_SEND_MAX_RETRY
#define MQTT_SEND_MAX_RETRY 3              // MQTT 发布最多重试 3 次
#endif

// MQTT 主题定义
#ifndef MQTT_TOPIC_GPS
#define MQTT_TOPIC_GPS "fleet/PE-001/gps"
#endif
#ifndef MQTT_TOPIC_BATTERY
#define MQTT_TOPIC_BATTERY "fleet/PE-001/battery"
#endif

#ifndef RS485_UART_NUM
#define RS485_UART_NUM UART_NUM_0    // 默认使用 UART0（与 USB-Serial-JTAG 分离）
#endif
#ifndef RS485_TX_PIN
#define RS485_TX_PIN 10              // RS485 DI <- ESP32 TX (IO10)
#endif
#ifndef RS485_RX_PIN
#define RS485_RX_PIN 9               // RS485 RO -> ESP32 RX (IO9)
#endif
// 可选：若有方向控制脚，解除注释并设置引脚号
// #define RS485_DE_RE_PIN  -1        // 高电平发送，低电平接收

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 40               // ESP32 TX -> TJA1051 TXD
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 39               // ESP32 RX <- TJA1051 RXD
#endif
#ifndef CAN_EN_PIN
#define CAN_EN_PIN 41               // TJA1051 Silent/Standby 控制脚（低电平启用）
#endif

#endif // HARDWARE_CONFIG_H