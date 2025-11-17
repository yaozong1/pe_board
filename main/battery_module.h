#ifndef BATTERY_MODULE_H
#define BATTERY_MODULE_H

#include <stdint.h>
#include <stdbool.h>

// ADC和电池相关函数
void battery_init(void);
float read_battery_voltage(void);
int estimate_soc_from_voltage(float voltage);
uint32_t read_adc_raw(void);
uint32_t read_adc_voltage_mv(void);
// IO2 (IBL) ADC helper
uint32_t read_io2_raw(void);
uint32_t read_io2_voltage_mv(void);

#endif // BATTERY_MODULE_H