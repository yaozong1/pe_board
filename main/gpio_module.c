#include "gpio_module.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "GPIO";

void gpio_init(void)
{
    // GPIO 初始化
    gpio_set_direction(LED_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction(QT_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(QT_POWER_PIN, 0);

    gpio_set_direction(LED_PIN_EN_4V, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_GPS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_3V3_LDO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_GNSS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_RST_GNSS, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS_EN, GPIO_MODE_OUTPUT);

    gpio_set_direction(EG915U_POWER, GPIO_MODE_INPUT);
    
    // 初始化电源控制引脚状态
    power_control_init();

    ESP_LOGI(TAG, "GPIO initialized");
}

void power_control_init(void)
{
    gpio_set_level(LED_PIN_EN_4V, 1);
    gpio_set_level(LED_PIN_EN_GPS, 1);
    gpio_set_level(LED_PIN_EN_3V3_LDO, 1);
    gpio_set_level(LED_PIN_EN_GNSS, 1);
    gpio_set_level(LED_PIN_RST_GNSS, 0);
    gpio_set_level(RS_EN, 1);
    
    ESP_LOGI(TAG, "Power control pins initialized");
}

void led_set(int led_pin, bool state)
{
    gpio_set_level(led_pin, state ? 1 : 0);
}

bool button_is_pressed(void)
{
    return gpio_get_level(BUTTON_PIN) == 0;
}

void led1_on(void)
{
    led_set(LED_PIN_1, true);
}

void led1_off(void)
{
    led_set(LED_PIN_1, false);
}

void led3_on(void)
{
    led_set(LED_PIN_3, true);
}

void led3_off(void)
{
    led_set(LED_PIN_3, false);
}