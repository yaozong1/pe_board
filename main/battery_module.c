#include "battery_module.h"
#include "hardware_config.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "BATTERY";

// ADC校准特性
esp_adc_cal_characteristics_t *adc_chars = NULL;

void battery_init(void)
{
    // 初始化 ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
    
    ESP_LOGI(TAG, "ADC initialized for battery monitoring");
}

uint32_t read_adc_raw(void)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}

uint32_t read_adc_voltage_mv(void)
{
    uint32_t adc_reading = read_adc_raw();
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage;
}

float read_battery_voltage(void)
{
    uint32_t mv = read_adc_voltage_mv();
#ifndef VBAT_DIVIDER_RATIO
#define VBAT_DIVIDER_RATIO 2.0f
#endif
    return (float)mv / 1000.0f * VBAT_DIVIDER_RATIO;
}

int estimate_soc_from_voltage(float v)
{
    // 非线性电池曲线此处用近似阶梯/线性，后续可替换为查表/卡尔曼
    if (v >= 12.6f) { return 100; }
    if (v <= 10.5f) { return 0; }
    float soc = (v - 10.5f) / (12.6f - 10.5f) * 100.0f;
    if (soc < 0) {
        soc = 0;
    }
    if (soc > 100) {
        soc = 100;
    }
    return (int)(soc + 0.5f);
}