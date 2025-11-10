# K10-3U8 电压采集模块校准指南

## 问题描述

测试发现，K10-3U8 的不同通道在测量相同电压时存在差异：

**测试条件：** AI1 和 AI4 接在同一 4.62V 电源
- **AI1 读数：** 4801 mV（误差 +181 mV，约 +3.9%）
- **AI4 读数：** 4612 mV（误差 -8 mV，约 -0.2%）
- **差值：** 189 mV

这种通道间差异超出了工业级 ADC 通常的 ±0.5% 精度要求。

## 原因分析

### 1. 硬件特性
- 每个 ADC 通道都有独立的模拟前端电路
- 每个通道的零点偏移和增益略有不同
- 这是模拟电路的固有特性，需要通过校准来补偿

### 2. K10-3U8 规格
- **测量范围：** 0~10V
- **数字输出（工程量）：** 0~10000 mV
- **分辨率：** 1 mV
- **典型精度：** 需查阅规格书（通常为 ±0.5% 或 ±1%）

### 3. 为什么需要软件校准
- K10-3U8 可能提供硬件校准寄存器（需查阅规格书确认）
- 如果没有硬件校准功能，或者需要更灵活的校准方案，可以使用软件校准
- 软件校准可以在生产测试时针对每个通道进行调整

## 软件校准方案

### 校准公式
```c
calibrated_voltage = (raw_voltage * gain) + offset
```

### 参数说明
- **raw_voltage：** 从寄存器读取的原始值（单位：mV）
- **gain：** 增益校正系数（通常接近 1.0）
- **offset：** 零点偏移校正（单位：mV）
- **calibrated_voltage：** 校准后的电压值（单位：mV）

### 校准数据结构
```c
typedef struct {
    float gain;      // 增益校正系数
    int16_t offset;  // 零点偏移（mV）
} voltage_cal_t;
```

## 使用方法

### 1. 默认校准参数
固件已内置基于测试数据的默认校准：

```c
void app_main(void)
{
    voltage_adc_init();
    voltage_adc_load_default_calibration();  // 加载默认校准
    // ...
}
```

当前默认校准：
- **AI1：** Gain=1.0, Offset=-181 mV（校正 +181 mV 的偏差）
- **AI4：** 无校准（偏差仅 -8 mV，在可接受范围内）
- **其他通道：** 暂无校准数据

### 2. 手动设置校准参数
如果需要针对特定通道调整校准：

```c
// 设置 AI1 的校准参数
voltage_adc_set_calibration(0, 1.0f, -181);  // channel 0 = AI1

// 设置 AI4 的校准参数
voltage_adc_set_calibration(3, 1.0f, 8);     // channel 3 = AI4
```

### 3. 校准过程
建议在生产测试阶段执行以下步骤：

1. **准备标准电压源：**
   - 使用精密可调电源或基准电压源
   - 建议使用两个校准点：0V 和 5V（或接近满量程的 9V）

2. **测量零点偏移（0V）：**
   ```c
   // 将所有 AI 通道接地 (AGND)
   // 读取各通道数值，理想应为 0 mV
   // 如果 AI1 读数为 5 mV，则 offset = -5
   voltage_adc_set_calibration(0, 1.0f, -5);
   ```

3. **测量增益（5V 或 9V）：**
   ```c
   // 将所有 AI 通道接标准 5000 mV 电压
   // 如果 AI1 读数为 5050 mV，则：
   // gain = 5000 / 5050 = 0.9901
   voltage_adc_set_calibration(0, 0.9901f, -5);
   ```

4. **验证校准：**
   - 用多个不同电压点测试（如 1V, 3V, 5V, 7V, 9V）
   - 确保误差在 ±0.5% 以内

## 校准数据存储

### 方案 1：硬编码（当前方案）
校准参数直接写在代码中 `voltage_adc_load_default_calibration()`：
- **优点：** 简单，无需额外存储
- **缺点：** 每个测试治具需要单独编译固件

### 方案 2：NVS 存储（推荐用于量产）
将校准参数保存在 ESP32-S3 的 NVS（Non-Volatile Storage）中：
```c
#include "nvs_flash.h"
#include "nvs.h"

void save_calibration_to_nvs(uint8_t channel, float gain, int16_t offset)
{
    nvs_handle_t handle;
    nvs_open("voltage_cal", NVS_READWRITE, &handle);
    
    char key_gain[16], key_offset[16];
    snprintf(key_gain, sizeof(key_gain), "ch%d_gain", channel);
    snprintf(key_offset, sizeof(key_offset), "ch%d_offset", channel);
    
    // 将 float 转换为 uint32_t 存储
    uint32_t gain_u32;
    memcpy(&gain_u32, &gain, sizeof(gain));
    nvs_set_u32(handle, key_gain, gain_u32);
    nvs_set_i16(handle, key_offset, offset);
    
    nvs_commit(handle);
    nvs_close(handle);
}

void load_calibration_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open("voltage_cal", NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "No calibration data in NVS, using defaults");
        voltage_adc_load_default_calibration();
        return;
    }
    
    for (int i = 0; i < 8; i++) {
        char key_gain[16], key_offset[16];
        snprintf(key_gain, sizeof(key_gain), "ch%d_gain", i);
        snprintf(key_offset, sizeof(key_offset), "ch%d_offset", i);
        
        uint32_t gain_u32 = 0x3F800000;  // 默认 1.0f
        int16_t offset = 0;
        
        nvs_get_u32(handle, key_gain, &gain_u32);
        nvs_get_i16(handle, key_offset, &offset);
        
        float gain;
        memcpy(&gain, &gain_u32, sizeof(gain));
        
        voltage_adc_set_calibration(i, gain, offset);
    }
    
    nvs_close(handle);
}
```

### 方案 3：通过 UART 命令动态设置
在 `console_task` 中添加校准命令：
```c
// !CAL <channel> <gain> <offset>
// 例如：!CAL 0 1.0 -181
if (sscanf(cmd, "CAL %d %f %d", &ch, &gain, &offset) == 3) {
    voltage_adc_set_calibration(ch, gain, offset);
    usj_write("JIG: Calibration set\r\n");
}
```

## 验证校准效果

### 测试步骤
1. 使用精密电压源输出 4.62V
2. 同时连接到 AI1 和 AI4
3. 观察日志输出：

**校准前：**
```
VADC: AI1: Raw= 4801 = 4801 mV ( 4.801 V)
VADC: AI4: Raw= 4612 = 4612 mV ( 4.612 V)
```

**校准后（应用 -181 mV offset 到 AI1）：**
```
VADC: AI1: Raw= 4801 -> Calibrated= 4620 mV ( 4.620 V) [Gain=1.0000, Offset=-181]
VADC: AI4: Raw= 4612 = 4612 mV ( 4.612 V)
```

### 预期结果
- AI1 和 AI4 的读数应在 ±10 mV 以内（约 ±0.2%）
- 所有通道在相同电压下的读数差异应 < 50 mV

## 硬件校准（如果 K10-3U8 支持）

查阅 K10-3U8 用户手册，寻找以下内容：

1. **校准寄存器地址：**
   - 零点校准寄存器（Zero Calibration）
   - 满量程校准寄存器（Full Scale Calibration）
   - 可能每个通道都有独立的校准寄存器

2. **校准程序：**
   - 某些模块支持自动校准命令
   - 例如：写入特定值到校准寄存器触发自校准

3. **出厂校准数据：**
   - 模块可能已经有出厂校准
   - 检查是否有"恢复出厂校准"的命令

**如果找到硬件校准功能，建议优先使用硬件校准**，因为：
- 校准参数存储在模块内部，更换 ESP32-S3 也不影响
- 可能有更高的校准精度
- 减少软件复杂度

## 精度要求

### 工业标准
- **高精度应用：** ±0.1% FS (±10 mV for 0~10V range)
- **标准应用：** ±0.5% FS (±50 mV)
- **一般应用：** ±1% FS (±100 mV)

### 当前测试结果
- **AI1 未校准：** +3.9% (超标)
- **AI1 校准后：** 预计 ±0.2% (良好)
- **AI4：** -0.2% (已在标准范围内)

## 注意事项

1. **AGND 和 GND 连接：**
   - 确保 K10-3U8 的 AGND 与 ESP32-S3 的 GND 相连
   - 这是模拟信号测量的基础

2. **电源质量：**
   - K10-3U8 需要稳定的 24V 电源（或按规格书要求）
   - 电源纹波会影响测量精度

3. **布线：**
   - 电压输入线尽量短且屏蔽
   - 避免与强电信号并行走线

4. **环境因素：**
   - 温度变化会影响模拟电路
   - 如果需要在宽温度范围工作，可能需要温度补偿

5. **定期校准：**
   - 建议每 6~12 个月重新校准一次
   - 如果发现测量偏差增大，及时校准

## 后续工作

1. **完善校准数据：**
   - 测试所有 8 个通道的零点和增益
   - 建立完整的校准表

2. **自动化校准流程：**
   - 在生产测试工装中集成校准程序
   - 自动测量、计算并保存校准参数

3. **GUI 集成：**
   - 在 `factory_gui.py` 中显示校准状态
   - 提供校准向导界面

4. **文档完善：**
   - 记录每个测试治具的校准参数
   - 建立校准记录追溯系统

## 参考资料

- K10-3U8 用户手册（查找校准相关章节）
- Modbus RTU 协议规范
- ESP-IDF NVS 编程指南：https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/storage/nvs_flash.html
