#ifndef GPIO_MODULE_H
#define GPIO_MODULE_H

#include <stdint.h>
#include <stdbool.h>

// GPIO控制函数
void gpio_init(void);
void led_set(int led_pin, bool state);
bool button_is_pressed(void);
void power_control_init(void);

// LED控制函数
void led1_on(void);
void led1_off(void);
void led3_on(void);
void led3_off(void);

// IGN光耦测试函数
bool ign_test_detect_transition(uint32_t timeout_ms);
int ign_get_level(void);

#endif // GPIO_MODULE_H