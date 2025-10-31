# LIS2DH12TR I2C通信故障排除指南

## 错误现象
```
E (484) MOTION_SENSOR: Read register 0x0F failed: ESP_FAIL
```

## 错误分析
该错误表明ESP32无法通过I2C与LIS2DH12TR传感器通信。寄存器0x0F是WHO_AM_I寄存器，用于设备识别。

## 可能原因及解决方案

### 1. 硬件连接问题 ⚡
**检查项目：**
- ✅ SDA线连接到GPIO14
- ✅ SCL线连接到GPIO15  
- ✅ VCC连接到3.3V
- ✅ GND正确接地
- ✅ 连接牢固，无虚焊

**解决方案：**
```bash
用万用表检查连线连通性
检查焊接质量
确认连接无短路
```

### 2. 电源问题 🔋
**检查项目：**
- 传感器VCC电压是否稳定在3.3V
- 电流供应是否充足
- 是否有电源噪声

**解决方案：**
```bash
用万用表测量VCC电压
检查电源纹波
添加去耦电容（100nF和10µF）
```

### 3. I2C上拉电阻问题 📈
**检查项目：**
- SDA和SCL线上是否有上拉电阻
- 上拉电阻阻值是否合适（推荐4.7kΩ）
- 多设备共享I2C时上拉是否足够

**解决方案：**
```bash
添加4.7kΩ上拉电阻到VCC
检查现有上拉电阻值
移除额外的并联电阻
```

### 4. I2C地址问题 📍
**可能地址：**
- 0x18 (当SDO/SA0引脚接GND时)
- 0x19 (当SDO/SA0引脚接VCC时)

**解决方案：**
```c
// 检查SDO/SA0引脚连接
// 如果悬空，通常默认为0x18
// 代码会自动尝试两个地址
```

### 5. GPIO引脚冲突 ⚠️
**检查项目：**
- GPIO14/15是否被其他功能占用
- 是否与JTAG调试器冲突
- 引脚多路复用配置

**解决方案：**
```c
// 检查hardware_config.h中的引脚定义
// 确保没有引脚重复使用
// 运行gpio诊断测试
```

### 6. 时序和频率问题 ⏱️
**可能问题：**
- I2C频率过高
- 信号上升时间不足
- 总线容量过大

**解决方案：**
```c
// 降低I2C频率
.master.clk_speed = 50000,  // 50kHz
// 或更低
.master.clk_speed = 10000,  // 10kHz
```

### 7. 传感器型号问题 🔍
**检查项目：**
- 确认传感器确实是LIS2DH12TR
- 检查WHO_AM_I预期值(0x33)
- 排除类似传感器(LIS2DH、LIS3DH等)

## 调试步骤

### 第一步：基础硬件检查
```bash
1. 用万用表检查所有连线
2. 测量VCC电压(应为3.3V±5%)
3. 检查SDA/SCL上的上拉电阻
```

### 第二步：运行诊断代码
当前代码已包含增强的调试功能：
```c
motion_sensor_debug_test();  // 运行完整诊断
```

### 第三步：查看调试输出
关注以下信息：
```
- Pin levels (should be 1 if pullups working)
- I2C scan results 
- Address response tests
- GPIO configuration results
```

### 第四步：逐步排除
1. **引脚测试**: 检查GPIO能否正确设置
2. **总线扫描**: 查看是否检测到任何I2C设备
3. **地址测试**: 专门测试0x18和0x19
4. **时序调整**: 尝试不同的I2C频率

## 常见解决方案

### 方案1：降低I2C频率
```c
// 在motion_sensor.c中修改
.master.clk_speed = 10000,  // 从100kHz降到10kHz
```

### 方案2：更换引脚
如果GPIO14/15有问题，可以更换：
```c
// 在hardware_config.h中修改
#define MOTION_SENSOR_SDA_PIN 21    // 更换为GPIO21
#define MOTION_SENSOR_SCL_PIN 22    // 更换为GPIO22
```

### 方案3：添加硬件上拉
在PCB上添加4.7kΩ电阻：
```
SDA ---- 4.7kΩ ---- VCC(3.3V)
SCL ---- 4.7kΩ ---- VCC(3.3V)
```

### 方案4：检查电源稳定性
添加去耦电容：
```
传感器VCC ---- 100nF ---- GND (靠近传感器)
传感器VCC ---- 10µF  ---- GND (电源入口)
```

## 监视器输出示例

### 正常情况：
```
I (1234) MOTION_SENSOR: Pin levels after init: SDA=1, SCL=1
I (1235) MOTION_SENSOR: Found device at address 0x18
I (1236) MOTION_SENSOR: LIS2DH12 found at address 0x18, WHO_AM_I=0x33
```

### 异常情况：
```
E (1234) MOTION_SENSOR: Pin levels after init: SDA=0, SCL=1  # SDA被拉低
W (1235) MOTION_SENSOR: No I2C devices found! Check wiring
E (1236) MOTION_SENSOR: Read register 0x0F failed: ESP_FAIL
```

## 如果仍无法解决

1. **验证硬件**: 用另一个已知工作的传感器测试
2. **检查PCB**: 查看PCB布线和焊接质量  
3. **更换引脚**: 尝试其他GPIO引脚
4. **降级测试**: 先用简单的GPIO读写测试引脚功能
5. **示波器检查**: 查看I2C信号波形(如果有设备)

## 联系支持

如果以上步骤都无法解决问题，请提供：
- 完整的监视器输出
- 硬件连接照片
- PCB原理图(如果有)
- 传感器型号确认