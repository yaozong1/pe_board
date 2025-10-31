#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// 模块头文件、
#include "hardware_config.h"
#include "gpio_module.h"
#include "gnss_module.h"
#include "mqtt_module.h"
#include "battery_module.h"
#include "timer_module.h"

static const char *TAG = "APP";

void app_main(void)
{
    ESP_LOGI(TAG, "PE Board Application Starting...");
    
    // 1. 初始化GPIO
    gpio_init();
    
    // 2. 初始化GNSS模块
    gnss_init();
    
    // 3. 初始化MQTT模块
    mqtt_init();
    
    // 4. 初始化电池监控模块
    battery_init();
    
    // 5. 初始化定时器
    timer_init();
    timer_start();
    
    // 6. 启动GNSS任务
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 8, NULL);
    
    // 7. 启动GPS+MQTT周期任务
    xTaskCreate(gps_mqtt_cycle_task, "gps_cycle", 4096, NULL, 9, NULL);
    
    // 等待系统稳定
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "All modules initialized, entering main loop");

    // 主循环
    while (1) {
        // 读取 ADC 值进行电池监控
        uint32_t adc_reading = read_adc_raw();
        uint32_t voltage_mv = read_adc_voltage_mv();
        
        // 定期打印电池状态（可选）
        static uint32_t last_battery_log = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_battery_log) > pdMS_TO_TICKS(10000)) { // 每10秒打印一次
            ESP_LOGI(TAG, "Battery: ADC=%lu, Voltage=%lu mV", 
                     (unsigned long)adc_reading, (unsigned long)voltage_mv);
            last_battery_log = now;
        }

        // 处理按键触发的事件
        if (log_output) {
            if (gpio_get_level(EG915U_POWER) == 0) {
                ESP_LOGI("MODEM", "Power cycling modem via button press");
                modem_power_cycle();
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待模组启动
            log_output = false;
        }

        // 处理AT命令标志
        if (send_at_flag) {
            send_at_flag = false;
            send_at_command("AT");
        }

        // 主循环延时
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}