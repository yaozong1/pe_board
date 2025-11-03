#include "eg915_selftest.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "hardware_config.h"
#include "mqtt_module.h"  // 复用 send_at_command / wait_for_ok / uart_read_response / modem_power_cycle / mqtt_init

static const char *TAG_ST = "EG915_TEST";

bool eg915_at_handshake_once(unsigned timeout_ms, int retries)
{
    // 发送AT并等待OK，重试若干次
    for (int i = 1; i <= retries; ++i) {
        ESP_LOGI(TAG_ST, "Handshake attempt %d/%d: sending AT...", i, retries);
        send_at_command("AT");
        if (wait_for_ok(timeout_ms)) {
            ESP_LOGI(TAG_ST, "AT handshake OK on attempt %d", i);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGW(TAG_ST, "AT handshake failed after %d attempts", retries);
    return false;
}

static void drain_boot_urcs(uint32_t ms)
{
    uint32_t t0 = xTaskGetTickCount();
    char buf[256];
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(ms)) {
        int n = uart_read_bytes(UART_EG915U_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            buf[n] = '\0';
            ESP_LOGI(TAG_ST, "URC: %s", buf);
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static void eg915_at_selftest_task(void *pv)
{
    (void)pv;
    ESP_LOGI(TAG_ST, "EG915 AT handshake self-test starting...");

    // 1) 确保GPIO初始化完成（app_main里会调用gpio_init）
    // 2) 初始化 UART1（与MQTT模块一致的串口设置）
    mqtt_init();

    // 3) 通过PWRKEY脉冲拉起模组，保证干净启动
    ESP_LOGI(TAG_ST, "Power-cycling EG915U via PWRKEY...");
    modem_power_cycle();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 4) 清理上电URC，避免干扰握手解析
    ESP_LOGI(TAG_ST, "Draining boot URCs for 3s...");
    drain_boot_urcs(3000);

    // 5) 进行AT握手（最多5次，每次1秒超时）
    bool ok = eg915_at_handshake_once(1000, 5);

    if (ok) {
        ESP_LOGI(TAG_ST, "EG915 AT self-test: PASS");
    } else {
        ESP_LOGE(TAG_ST, "EG915 AT self-test: FAIL");
    }

    // 自测完成后可选择保持任务或退出；这里退出任务
    vTaskDelete(NULL);
}

void start_eg915_at_selftest(void)
{
    xTaskCreate(eg915_at_selftest_task, "eg915_at_test", 4096, NULL, 10, NULL);
}
