#include "gpio_module.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GPIO";

void gpio_init(void)
{
    // GPIO 初始化
    // IO2 在此设计上用于 IBL (ADC) 采样，不应被当作 LED 驱动。
    // 仅将其内部上/下拉关闭以避免影响 ADC 读数（不要改变引脚复用为输出）。
    gpio_set_pull_mode(IBL_PIN, GPIO_FLOATING);
    gpio_set_direction(LED_PIN_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction(QT_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(QT_POWER_PIN, 0);

    gpio_set_direction(POWER_EN_4V, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_GPS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_3V3_LDO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_GNSS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_RST_GNSS, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS_EN, GPIO_MODE_OUTPUT);

    gpio_set_direction(EG915U_POWER, GPIO_MODE_INPUT);
    
    // IGN_IN (IO35) 光耦检测输入引脚配置
    gpio_set_direction(IGN_IN_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IGN_IN_PIN, GPIO_PULLDOWN_ONLY);
    
    // IM_OUT (IO36) Immobilizer光耦输出引脚配置
    gpio_reset_pin(IM_OUT_PIN);  // 先复位引脚，清除可能的冲突配置
    gpio_set_direction(IM_OUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_drive_capability(IM_OUT_PIN, GPIO_DRIVE_CAP_3);  // 设置最大驱动能力(40mA)
    gpio_set_level(IM_OUT_PIN, 0);  // 初始为低电平
    ESP_LOGI(TAG, "IM_OUT_PIN (IO%d) configured as output", IM_OUT_PIN);
    
    // CAN_EN_PIN (IO41) CAN收发器使能引脚配置 - 强制拉低启用CAN
    gpio_reset_pin(CAN_EN_PIN);
    gpio_set_direction(CAN_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_EN_PIN, 0);  // 低电平=CAN收发器启用(TJA1051 active-low)
    ESP_LOGI(TAG, "CAN_EN_PIN (IO%d) locked to LOW (CAN transceiver enabled)", CAN_EN_PIN);
    
    // 初始化电源控制引脚状�?
    power_control_init();

    ESP_LOGI(TAG, "GPIO initialized");
}

void power_control_init(void)
{
    gpio_set_level(POWER_EN_4V, 1);
    gpio_set_level(LED_PIN_EN_GPS, 1);
    gpio_set_level(LED_PIN_EN_3V3_LDO, 1);
    gpio_set_level(LED_PIN_EN_GNSS, 1);
    gpio_set_level(LED_PIN_RST_GNSS, 0);
    gpio_set_level(RS_EN, 1);
    
    ESP_LOGI(TAG, "Power control pins initialized");
}

void led_set(int led_pin, bool state)
{
    // 如果传入的 LED 引脚是 IBL_PIN（IO2，作为 ADC 使用），则跳过驱动以避免干扰外设。
    if (led_pin == IBL_PIN) {
        ESP_LOGD(TAG, "led_set: skipping drive for IBL_PIN (IO%d)", IBL_PIN);
        return;
    }
    gpio_set_level(led_pin, state ? 1 : 0);
}

bool button_is_pressed(void)
{
    return gpio_get_level(BUTTON_PIN) == 0;
}

void led1_on(void)
{
    // led1 对应物理引脚现在语义为 IBL；保持向后兼容但传入 IBL_PIN
    led_set(IBL_PIN, true);
}

void led1_off(void)
{
    led_set(IBL_PIN, false);
}

void led3_on(void)
{
    led_set(LED_PIN_3, true);
}

void led3_off(void)
{
    led_set(LED_PIN_3, false);
}

// IGN光耦测试：检测IGN_IN引脚电平变化
// 返回true表示检测到�?>�?>高的完整转换，说明光耦工作正�?
bool ign_test_detect_transition(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    // 状态机�?=等待初始高电�? 1=检测到高电平等待低电平, 2=检测到低电平等待高电平
    int state = 0;
    int initial_level = gpio_get_level(IGN_IN_PIN);
    
    ESP_LOGI(TAG, "IGN test: initial level=%d", initial_level);
    
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        int level = gpio_get_level(IGN_IN_PIN);
        
        switch (state) {
            case 0:  // 等待初始高电�?
                if (level == 1) {
                    state = 1;
                    ESP_LOGI(TAG, "IGN test: detected HIGH");
                }
                break;
            case 1:  // 等待低电�?
                if (level == 0) {
                    state = 2;
                    ESP_LOGI(TAG, "IGN test: detected transition HIGH->LOW");
                }
                break;
            case 2:  // 等待再次高电�?
                if (level == 1) {
                    ESP_LOGI(TAG, "IGN test: detected transition LOW->HIGH, PASS!");
                    return true;
                }
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms轮询间隔
    }
    
    ESP_LOGW(TAG, "IGN test: timeout, state=%d", state);
    return false;
}

// 读取IGN_IN当前电平
int ign_get_level(void)
{
    return gpio_get_level(IGN_IN_PIN);
}

// IM光耦测试任务：持续每100ms翻转IO36供治具检测
void im_test_toggle_task(void *pv)
{
    (void)pv;
    ESP_LOGI(TAG, "IM toggle task started on IO%d", IM_OUT_PIN);
    
    int level = 0;
    while (1) {
        gpio_set_level(IM_OUT_PIN, level);
        level = !level;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
