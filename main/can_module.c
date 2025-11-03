#include "sdkconfig.h"
#include "can_module.h"
#include "hardware_config.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "CAN";

// 兼容性处理：某些环境下 portTICK_PERIOD_MS 依赖 CONFIG_FREERTOS_HZ，若未定义则桥接到 FreeRTOS 配置
#ifndef CONFIG_FREERTOS_HZ
#include "freertos/FreeRTOSConfig.h"
#define CONFIG_FREERTOS_HZ configTICK_RATE_HZ
#endif

// CAN模块状态
static bool can_initialized = false;
static bool can_running = false;
static bool transceiver_enabled = false;

// 统计信息
static can_stats_t can_statistics = {0};

// 回调函数
static can_rx_callback_t rx_callback = NULL;
static can_error_callback_t error_callback = NULL;

// 内部函数声明
static bool configure_gpio_pins(void);
static void handle_can_alerts(uint32_t alerts);

bool can_transceiver_enable(bool enable)
{
    // TJA1051T/3 Silent(S)/STB pin: LOW = Normal (TX enabled), HIGH = Silent/Standby
    int level = enable ? 0 : 1; // active-low enable
    esp_err_t ret = gpio_set_level(CAN_EN_PIN, level);
    if (ret == ESP_OK) {
        transceiver_enabled = enable;
        ESP_LOGI(TAG, "CAN transceiver %s (IO%d=%d)", enable ? "enabled" : "disabled", CAN_EN_PIN, level);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to control CAN transceiver enable pin: %s", esp_err_to_name(ret));
        return false;
    }
}

static bool configure_gpio_pins(void)
{
    // 配置CAN使能引脚
    gpio_config_t en_config = {
        .pin_bit_mask = (1ULL << CAN_EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&en_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CAN enable pin: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 初始禁用收发器
    can_transceiver_enable(false);
    
    ESP_LOGI(TAG, "CAN GPIO configured: TX=%d, RX=%d, EN=%d", CAN_TX_PIN, CAN_RX_PIN, CAN_EN_PIN);
    return true;
}

// 注意：已固定使用 TWAI_TIMING_CONFIG_500KBITS()，如需动态比特率可另行扩展

bool can_module_init(uint32_t bitrate)
{
    if (can_initialized) {
        ESP_LOGW(TAG, "CAN module already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing CAN module with bitrate %lu bps", (unsigned long)bitrate);
    
    // 配置GPIO引脚
    if (!configure_gpio_pins()) {
        return false;
    }
    
    // 先启用CAN收发器
    if (!can_transceiver_enable(true)) {
        ESP_LOGE(TAG, "Failed to enable CAN transceiver");
        return false;
    }
    
    // 等待收发器稳定
    vTaskDelay(10); // ticks
    
    // 使用简化的TWAI配置 - 正常模式，便于与总线其他节点互通
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  // 使用250K速率
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // 使用默认中断标志（0）。在复杂工程中，先让TWAI尽早抢占中断资源再说。
    g_config.intr_flags = 0;
    // 打开关键告警，便于诊断（总线关闭、恢复、收发）
    g_config.alerts_enabled = TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_SUCCESS |
                              TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_RECOVERY_IN_PROGRESS | TWAI_ALERT_BUS_RECOVERED;
    ESP_LOGI(TAG, "TWAI intr flags initial: LEVEL1|SHARED");
    
    ESP_LOGI(TAG, "TWAI config: TX=%d, RX=%d, Mode=%d, intr_flags=0, alerts=0x%08lx", 
             g_config.tx_io, g_config.rx_io, g_config.mode, (unsigned long)g_config.alerts_enabled);
    
    // 安装TWAI驱动 - 使用简化配置（注意：驱动内部可能使用ESP_ERROR_CHECK，因此这里不能依赖返回码重试）
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        can_transceiver_enable(false);
        return false;
    }
    
    can_initialized = true;
    can_reset_stats();
    
    ESP_LOGI(TAG, "CAN module initialized successfully");
    return true;
}

bool can_module_start(void)
{
    if (!can_initialized) {
        ESP_LOGE(TAG, "CAN module not initialized");
        return false;
    }
    
    if (can_running) {
        ESP_LOGW(TAG, "CAN module already running");
        return true;
    }
    
    // 启用CAN收发器
    if (!can_transceiver_enable(true)) {
        return false;
    }
    
    // 启动TWAI驱动
    esp_err_t ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        can_transceiver_enable(false);
        return false;
    }
    
    can_running = true;
    ESP_LOGI(TAG, "CAN module started successfully");
    return true;
}

bool can_module_stop(void)
{
    if (!can_running) {
        ESP_LOGW(TAG, "CAN module already stopped");
        return true;
    }
    
    // 停止TWAI驱动
    esp_err_t ret = twai_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 禁用CAN收发器
    can_transceiver_enable(false);
    
    can_running = false;
    ESP_LOGI(TAG, "CAN module stopped");
    return true;
}

bool can_module_deinit(void)
{
    if (can_running) {
        can_module_stop();
    }
    
    if (!can_initialized) {
        return true;
    }
    
    // 卸载TWAI驱动
    esp_err_t ret = twai_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall TWAI driver: %s", esp_err_to_name(ret));
        return false;
    }
    
    can_initialized = false;
    ESP_LOGI(TAG, "CAN module deinitialized");
    return true;
}

bool can_send_message(const can_message_t* message, uint32_t timeout_ms)
{
    if (!can_running) {
        ESP_LOGE(TAG, "CAN module not running");
        return false;
    }
    
    if (!message || message->data_length > 8) {
        ESP_LOGE(TAG, "Invalid message parameters");
        return false;
    }
    
    twai_message_t twai_msg;
    
    // 设置标识符和帧格式 - 参考工作代码的方式
    twai_msg.identifier = message->identifier;
    twai_msg.extd = (message->format == CAN_FRAME_EXT) ? 1 : 0;
    twai_msg.rtr = (message->type == CAN_MSG_TYPE_REMOTE) ? 1 : 0;
    twai_msg.data_length_code = message->data_length;
    
    // 复制数据
    for (int i = 0; i < message->data_length; i++) {
        twai_msg.data[i] = message->data[i];
    }
    
    // 发送消息
    esp_err_t ret = twai_transmit(&twai_msg, timeout_ms); // timeout in ticks
    if (ret == ESP_OK) {
        can_statistics.tx_count++;
        ESP_LOGI(TAG, "CAN message sent - ID: 0x%03X, Length: %d", 
                 (unsigned int)message->identifier, message->data_length);
        return true;
    } else {
        can_statistics.tx_error_count++;
        // 打印更多状态，便于定位 INVALID_STATE 等问题
        twai_status_info_t st = {0};
        if (twai_get_status_info(&st) == ESP_OK) {
            ESP_LOGE(TAG, "Failed to send CAN message: %s (state=%d, tx_err=%u, rx_err=%u)",
                     esp_err_to_name(ret), (int)st.state, (unsigned)st.tx_error_counter, (unsigned)st.rx_error_counter);
        } else {
            ESP_LOGE(TAG, "Failed to send CAN message: %s (status unavailable)", esp_err_to_name(ret));
        }
        return false;
    }
}

bool can_receive_message(can_message_t* message, uint32_t timeout_ms)
{
    if (!can_running) {
        ESP_LOGE(TAG, "CAN module not running");
        return false;
    }
    
    if (!message) {
        ESP_LOGE(TAG, "Invalid message pointer");
        return false;
    }
    
    twai_message_t twai_msg;
    esp_err_t ret = twai_receive(&twai_msg, timeout_ms); // timeout in ticks
    if (ret == ESP_OK) {
        // 转换消息格式
        message->identifier = twai_msg.identifier;
        message->format = twai_msg.extd ? CAN_FRAME_EXT : CAN_FRAME_STD;
        message->type = twai_msg.rtr ? CAN_MSG_TYPE_REMOTE : CAN_MSG_TYPE_DATA;
        message->data_length = twai_msg.data_length_code;
        memcpy(message->data, twai_msg.data, twai_msg.data_length_code);
        
        can_statistics.rx_count++;
        return true;
    } else {
        if (ret != ESP_ERR_TIMEOUT) {
            can_statistics.rx_error_count++;
            ESP_LOGE(TAG, "Failed to receive CAN message: %s", esp_err_to_name(ret));
        }
        return false;
    }
}

bool can_send_data(uint32_t id, const uint8_t* data, uint8_t length, uint32_t timeout_ms)
{
    if (!data || length > 8) {
        return false;
    }
    
    can_message_t message = {
        .identifier = id,
        .format = CAN_FRAME_STD,
        .type = CAN_MSG_TYPE_DATA,
        .data_length = length
    };
    
    memcpy(message.data, data, length);
    return can_send_message(&message, timeout_ms);
}

bool can_send_remote_frame(uint32_t id, uint8_t length, uint32_t timeout_ms)
{
    can_message_t message = {
        .identifier = id,
        .format = CAN_FRAME_STD,
        .type = CAN_MSG_TYPE_REMOTE,
        .data_length = length
    };
    
    return can_send_message(&message, timeout_ms);
}

can_state_t can_get_state(void)
{
    if (!can_initialized) {
        return CAN_STATE_STOPPED;
    }
    
    if (!can_running) {
        return CAN_STATE_STOPPED;
    }
    
    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get TWAI status: %s", esp_err_to_name(ret));
        return CAN_STATE_STOPPED;  // 获取状态失败时返回停止状态
    }
    
    if (status.state == TWAI_STATE_BUS_OFF) {
        return CAN_STATE_BUS_OFF;
    } else if (status.state == TWAI_STATE_RECOVERING) {
        return CAN_STATE_ERROR_PASSIVE;  // 使用恢复状态映射到错误被动
    } else if (status.state == TWAI_STATE_RUNNING) {
        return CAN_STATE_RUNNING;
    }
    
    return CAN_STATE_STOPPED;
}

bool can_get_stats(can_stats_t* stats)
{
    if (!stats) {
        return false;
    }
    
    *stats = can_statistics;
    return true;
}

void can_reset_stats(void)
{
    memset(&can_statistics, 0, sizeof(can_statistics));
    ESP_LOGI(TAG, "CAN statistics reset");
}

bool can_is_bus_healthy(void)
{
    can_state_t state = can_get_state();
    return (state == CAN_STATE_RUNNING);
}

bool can_register_rx_callback(can_rx_callback_t callback)
{
    rx_callback = callback;
    return true;
}

bool can_register_error_callback(can_error_callback_t callback)
{
    error_callback = callback;
    return true;
}

static void handle_can_alerts(uint32_t alerts)
{
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
        // 发送成功
    }
    
    if (alerts & TWAI_ALERT_RX_DATA) {
        // 有数据接收
        can_message_t message;
        while (can_receive_message(&message, 0)) {
            if (rx_callback) {
                rx_callback(&message);
            } else {
                // 默认打印一条简洁的接收日志，便于现场验证
                char data_str[3 * 8 + 1];
                int pos = 0;
                for (uint8_t i = 0; i < message.data_length && i < 8; i++) {
                    pos += snprintf(&data_str[pos], sizeof(data_str) - pos, "%02X ", message.data[i]);
                    if (pos >= (int)sizeof(data_str) - 1) break;
                }
                data_str[pos] = '\0';
                ESP_LOGI(TAG, "RX: %s ID=0x%08lX DLC=%u DATA: %s",
                         message.format == CAN_FRAME_EXT ? "EXT" : "STD",
                         (unsigned long)message.identifier,
                         (unsigned)message.data_length,
                         data_str);
            }
        }
    }
    
    if (alerts & TWAI_ALERT_BUS_OFF) {
        can_statistics.bus_off_count++;
        ESP_LOGW(TAG, "CAN bus off detected");
        if (error_callback) {
            error_callback(CAN_STATE_BUS_OFF, alerts);
        }
    }
    
    if (alerts & TWAI_ALERT_ARB_LOST) {
        can_statistics.arbitration_lost_count++;
        ESP_LOGD(TAG, "CAN arbitration lost");
    }
    
    if (alerts & TWAI_ALERT_BUS_ERROR) {
        ESP_LOGW(TAG, "CAN bus error detected");
        if (error_callback) {
            error_callback(CAN_STATE_ERROR_WARNING, alerts);
        }
    }
}

void can_task(void* pvParameters)
{
    ESP_LOGI(TAG, "CAN task started");
    
    uint32_t alerts;
    
    while (1) {
        if (can_running) {
            // 检查CAN警报
            esp_err_t ret = twai_read_alerts(&alerts, 100); // ticks
            if (ret == ESP_OK && alerts != 0) {
                handle_can_alerts(alerts);
                if (alerts & TWAI_ALERT_BUS_OFF) {
                    ESP_LOGW(TAG, "TWAI bus-off, initiating recovery...");
                    // 启动恢复流程
                    twai_initiate_recovery();
                }
                if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                    ESP_LOGI(TAG, "TWAI bus recovered");
                }
            }
        } else {
            // CAN未运行时休眠
            vTaskDelay(1000); // ticks
        }
    }
}

void can_test_loopback(void)
{
    ESP_LOGI(TAG, "Starting CAN loopback test...");
    
    if (!can_running) {
        ESP_LOGE(TAG, "CAN module not running");
        return;
    }
    
    // 发送测试数据
    uint8_t test_data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    
    for (int i = 0; i < 10; i++) {
        test_data[0] = i;
        if (can_send_data(0x123, test_data, sizeof(test_data), 1000)) {
            ESP_LOGI(TAG, "Test message %d sent successfully", i);
        } else {
            ESP_LOGE(TAG, "Failed to send test message %d", i);
        }
        
    vTaskDelay(500); // ticks
    }
    
    ESP_LOGI(TAG, "CAN loopback test completed");
}

void can_test_send_periodic(void)
{
    ESP_LOGI(TAG, "Starting periodic CAN transmission test...");
    
    static uint32_t counter = 0;
    uint8_t data[8];
    
    // 构造测试数据
    data[0] = (counter >> 24) & 0xFF;
    data[1] = (counter >> 16) & 0xFF;
    data[2] = (counter >> 8) & 0xFF;
    data[3] = counter & 0xFF;
    data[4] = 0xAA;
    data[5] = 0xBB;
    data[6] = 0xCC;
    data[7] = 0xDD;
    
    if (can_send_data(0x100, data, 8, 100)) {
        ESP_LOGI(TAG, "Periodic message sent: counter=%lu", (unsigned long)counter);
    } else {
        ESP_LOGE(TAG, "Failed to send periodic message");
    }
    
    counter++;
}