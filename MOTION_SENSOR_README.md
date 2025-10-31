# LIS2DH12TR 运动传感器模块使用说明

## 概述
LIS2DH12TR 是一个低功耗的三轴线性加速度计，通过I2C接口与ESP32通信。

## 硬件连接
- **SDA**: GPIO14 (I2C数据线)
- **SCL**: GPIO15 (I2C时钟线)
- **VCC**: 3.3V
- **GND**: 地线

## 模块功能

### 基础功能
1. **设备检测**: 自动检测传感器I2C地址 (0x18 或 0x19)
2. **数据采集**: 读取X、Y、Z三轴加速度数据
3. **运动检测**: 实时监测加速度变化
4. **多种量程**: 支持 ±2g, ±4g, ±8g, ±16g
5. **可调输出率**: 1Hz 到 5.3kHz

### 核心函数

#### 初始化函数
```c
bool motion_sensor_init(void);
```
- 初始化I2C接口
- 检测传感器ID
- 配置默认参数 (100Hz, ±2g)

#### 数据读取
```c
bool lis2dh12_read_accel_data(lis2dh12_accel_data_t* data);
```
- 读取校准后的加速度值 (单位: g)
- 返回结构体包含 x, y, z 三个浮点数

#### 运动检测
```c
bool lis2dh12_data_ready(void);
```
- 检查是否有新数据可读

```c
float lis2dh12_calculate_magnitude(const lis2dh12_accel_data_t* data);
```
- 计算加速度向量的模长

### 配置选项

#### 输出数据率 (ODR)
```c
typedef enum {
    LIS2DH12_ODR_POWER_DOWN = 0x00,
    LIS2DH12_ODR_1HZ        = 0x10,
    LIS2DH12_ODR_10HZ       = 0x20,
    LIS2DH12_ODR_25HZ       = 0x30,
    LIS2DH12_ODR_50HZ       = 0x40,
    LIS2DH12_ODR_100HZ      = 0x50,  // 默认
    LIS2DH12_ODR_200HZ      = 0x60,
    LIS2DH12_ODR_400HZ      = 0x70
} lis2dh12_odr_t;
```

#### 满量程范围
```c
typedef enum {
    LIS2DH12_FS_2G  = 0x00,  // 默认, ±2g
    LIS2DH12_FS_4G  = 0x10,  // ±4g
    LIS2DH12_FS_8G  = 0x20,  // ±8g
    LIS2DH12_FS_16G = 0x30   // ±16g
} lis2dh12_fs_t;
```

### 使用示例

#### 简单的数据读取
```c
lis2dh12_accel_data_t accel_data;
if (lis2dh12_data_ready()) {
    if (lis2dh12_read_accel_data(&accel_data)) {
        printf("X=%.3fg, Y=%.3fg, Z=%.3fg\\n", 
               accel_data.x, accel_data.y, accel_data.z);
    }
}
```

#### 运动检测
```c
float magnitude = lis2dh12_calculate_magnitude(&accel_data);
if (magnitude > 1.1f) {  // 超过1.1g表示有运动
    printf("Motion detected! Magnitude: %.3fg\\n", magnitude);
}
```

#### 重新配置传感器
```c
// 配置为200Hz输出率，±4g量程
lis2dh12_configure(LIS2DH12_ODR_200HZ, LIS2DH12_FS_4G);
```

## 任务集成

运动传感器模块包含一个独立的FreeRTOS任务 `motion_sensor_task`，该任务：

1. **自动运行**: 在 `main.c` 中自动启动
2. **运动检测**: 检测加速度变化超过阈值 (默认0.1g)
3. **日志输出**: 检测到运动时打印详细信息
4. **低CPU占用**: 100ms采样间隔，不影响其他任务

## 故障排除

### 常见问题
1. **设备未检测到**
   - 检查I2C连线 (SDA=14, SCL=15)
   - 确认供电 (3.3V)
   - 检查地线连接

2. **数据读取失败**
   - 检查I2C时序是否正确
   - 确认传感器已正确初始化
   - 检查是否有I2C地址冲突

3. **运动检测不敏感**
   - 调整运动阈值 (`motion_threshold`)
   - 修改输出数据率提高响应速度
   - 检查传感器安装方向

### 调试信息
模块会输出详细的日志信息：
- 初始化状态
- 设备ID检测结果
- I2C通信状态
- 运动检测事件

可通过ESP-IDF监视器查看：
```bash
idf.py monitor
```

## 技术规格
- **接口**: I2C (100kHz)
- **分辨率**: 12位 (高分辨率模式)
- **量程**: ±2g/±4g/±8g/±16g
- **输出率**: 1Hz - 5.3kHz
- **功耗**: 低功耗模式支持
- **工作电压**: 2.16V - 3.6V
- **温度范围**: -40°C 到 +85°C