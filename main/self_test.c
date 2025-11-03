#include "self_test.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "hardware_config.h"
#include "gpio_module.h"
#include "can_module.h"
#include "rs485_module.h"
#include "motion_sensor.h"
#include "mqtt_module.h" // 包含 EG915 相关接口（send_at/等待OK/电源控制）
#include "battery_module.h"

static const char *TAG = "SELFTEST";

typedef struct {
    bool can_ok;
    bool rs485_tx_ok;
    bool motion_ok;
    bool eg915_ok;
    bool battery_ok;
} selftest_result_t;

static void log_summary(const selftest_result_t *r)
{
    ESP_LOGI(TAG, "================ Self-Test Summary ================");
    ESP_LOGI(TAG, "CAN:            %s", r->can_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "RS485 (TX):     %s", r->rs485_tx_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "Motion Sensor:  %s", r->motion_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "EG915 (AT):     %s", r->eg915_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "Battery ADC:    %s", r->battery_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "===================================================");
}

static bool test_can(void)
{
    ESP_LOGI(TAG, "[CAN] init/start @250K...");
    bool ok = false;
    if (can_module_init(CAN_BITRATE_250K)) {
        if (can_module_start()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            // 简单健康检查：不处于BUS OFF即认为通过
            ok = can_is_bus_healthy();

            // 试发一帧，便于观察告警
            uint8_t payload[8] = {0x53, 0x54, 0x5F, 0x43, 0x41, 0x4E, 0x00, 0x01}; // "ST_CAN\0\x01"
            if (!can_send_data(0x321, payload, 8, 100)) {
                ESP_LOGW(TAG, "[CAN] transmit returned error (bus may be idle/no ACK)");
            }
        }
    }
    ESP_LOGI(TAG, "[CAN] result: %s", ok ? "PASS" : "FAIL");
    return ok;
}

static bool test_rs485_tx(void)
{
    ESP_LOGI(TAG, "[RS485] init @115200...");
    if (!rs485_init(115200)) {
        ESP_LOGE(TAG, "[RS485] init failed");
        return false;
    }

    const char *pattern = "SELFTEST_RS485";
    int w = rs485_write((const uint8_t*)pattern, (int)strlen(pattern), pdMS_TO_TICKS(200));
    if (w < 0) {
        ESP_LOGE(TAG, "[RS485] write failed");
        return false;
    }

    // 非强制回环：尝试读取（若有回环或对端回显则能看到）
    uint8_t buf[64];
    int r = rs485_read(buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (r > 0) {
        ESP_LOGI(TAG, "[RS485] RX %d bytes (loopback/peer): first=%02X", r, buf[0]);
    } else {
        ESP_LOGI(TAG, "[RS485] no RX (ok if no loopback)");
    }
    return true; // 只要TX成功即可判定通过
}

static bool test_motion(void)
{
    ESP_LOGI(TAG, "[MOTION] init...");
    if (!motion_sensor_init()) {
        ESP_LOGE(TAG, "[MOTION] init failed");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    lis2dh12_accel_data_t a;
    if (lis2dh12_read_accel_data(&a)) {
        ESP_LOGI(TAG, "[MOTION] Accel: X=%.3fg Y=%.3fg Z=%.3fg", a.x, a.y, a.z);
        return true;
    }
    ESP_LOGE(TAG, "[MOTION] read accel failed");
    return false;
}

static bool test_eg915(void)
{
    ESP_LOGI(TAG, "[EG915] power cycle + AT...");
    modem_power_cycle();
    // 等待模组上电完成并输出 RDY 等 URC，适当加长等待时间
    vTaskDelay(pdMS_TO_TICKS(5000));

    send_at_command("AT");
    bool ok = wait_for_ok(2000);
    ESP_LOGI(TAG, "[EG915] AT result: %s", ok ? "PASS" : "FAIL");
    return ok;
}

static bool test_battery(void)
{
    ESP_LOGI(TAG, "[BAT] init + read ADC...");
    battery_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    uint32_t raw = read_adc_raw();
    uint32_t mv  = read_adc_voltage_mv();
    ESP_LOGI(TAG, "[BAT] ADC=%lu, %lu mV", (unsigned long)raw, (unsigned long)mv);
    return (raw > 0); // 粗略判断
}

void self_test_run(void)
{
    ESP_LOGI(TAG, "===== PE Board Self-Test Mode (SELF_TEST_ENABLE=1) =====");
    gpio_init();

    // 若需要，打开基本电源通道
    power_control_init();

    // 初始化 EG915U 所在的 UART1（用于AT通信）
    mqtt_init();

    while (1) {
        selftest_result_t r = {0};

        // 各子项测试
        r.can_ok = test_can();
        r.rs485_tx_ok = test_rs485_tx();
        r.motion_ok = test_motion();
    r.eg915_ok = test_eg915();
        r.battery_ok = test_battery();

        log_summary(&r);

        // 指示灯：LED1 亮表示全部通过，LED3 亮表示存在失败
        bool all_pass = r.can_ok && r.rs485_tx_ok && r.motion_ok && r.eg915_ok && r.battery_ok;
        led1_off();
        led3_off();
        if (all_pass) {
            led1_on();
        } else {
            led3_on();
        }

        // 间隔一段时间重复测试，便于台架观察
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
