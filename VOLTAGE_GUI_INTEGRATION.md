# 电压采集 GUI 集成测试指南

## 功能说明

ESP32-S3 测试治具通过 K10-3U8 模块采集 8 路电压，并通过 USB-Serial-JTAG (COM24) 实时发送到上位机 GUI 显示。

## 数据流程

```
K10-3U8 ──(Modbus RTU)──> ESP32-S3 ──(USB-Serial-JTAG)──> GUI
 8路电压                   治具固件      COM24            Python GUI
                         (每2秒采集)                     (实时显示)
```

## 固件端实现

### 1. 数据采集（voltage_adc.c）
- 每 2 秒通过 Modbus RTU 读取 K10-3U8 的 8 个通道
- 寄存器地址：0x0010~0x0017（工程量，直接 mV 值）
- 应用校准参数：`calibrated = (raw * gain) + offset`

### 2. 数据发送（voltage_adc.c）
```c
void voltage_adc_send_payload(const uint16_t voltages[8])
{
    char payload[256];
    snprintf(payload, sizeof(payload),
        "VOLTAGE_ADC: {"
        "\"ch1\":%u,\"ch2\":%u,\"ch3\":%u,\"ch4\":%u,"
        "\"ch5\":%u,\"ch6\":%u,\"ch7\":%u,\"ch8\":%u"
        "}\r\n",
        voltages[0], voltages[1], voltages[2], voltages[3],
        voltages[4], voltages[5], voltages[6], voltages[7]
    );
    usb_serial_jtag_write_bytes((const uint8_t*)payload, len, ...);
}
```

**JSON 格式示例：**
```
VOLTAGE_ADC: {"ch1":4620,"ch2":0,"ch3":0,"ch4":4612,"ch5":0,"ch6":0,"ch7":0,"ch8":0}
```

### 3. 自动任务（main.c）
```c
void voltage_task(void *arg)
{
    while (1) {
        uint16_t voltages[8];
        bool ok = voltage_adc_read_all(voltages);
        if (ok) {
            voltage_adc_send_payload(voltages);  // 发送到 COM24
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // 每 2 秒
    }
}
```

## GUI 端实现

### 1. 界面显示（factory_gui.py）
```python
# 8 路电压表格
self.volt_table = QTableWidget(8, 3)
self.volt_table.setHorizontalHeaderLabels(["通道", "电压(V)", "状态"])

# 初始化 8 行
for i in range(8):
    self.volt_table.setItem(i, 0, QTableWidgetItem(f"AI{i+1}"))
    self.volt_table.setItem(i, 1, QTableWidgetItem("-"))
    self.volt_table.setItem(i, 2, QTableWidgetItem("-"))
```

### 2. 数据解析（factory_gui.py）
```python
def _parse_voltage_adc(self, text: str):
    """解析电压 ADC JSON 数据"""
    pattern = r'VOLTAGE_ADC:\s*(\{[^}]+\})'
    match = re.search(pattern, text)
    if match:
        data = json.loads(match.group(1))
        for i in range(8):
            ch_key = f"ch{i+1}"
            voltage_mv = int(data[ch_key])
            voltage_v = voltage_mv / 1000.0
            
            # 更新显示
            self.volt_table.item(i, 1).setText(f"{voltage_v:.3f}")
            
            # 状态判断
            if voltage_mv > 10000:
                status = "过压"  # 红色
            elif voltage_mv == 0:
                status = "未接"  # 灰色
            else:
                status = "OK"    # 绿色
```

### 3. 实时更新（on_tick）
```python
def on_tick(self):
    chunk = self.flash_serial.read_lines()  # 从 COM24 读取
    if chunk:
        self._append_log(chunk)
        self._parse_voltage_adc(chunk)  # 解析电压数据
        # ... 其他处理
```

## 测试步骤

### 第 1 步：硬件连接

**ESP32-S3 测试治具：**
- USB-Serial-JTAG → PC 的 COM24
- UART1 (GPIO17/18) → K10-3U8 模块

**K10-3U8 模块：**
```
[K10-3U8]
  AI1 ──→ 测试电压源 (+5V)
  AI2 ──→ (未接或接其他电压)
  AI3 ──→ (未接)
  AI4 ──→ 测试电压源 (+5V，与 AI1 并联)
  AI5 ──→ (未接)
  AI6 ──→ (未接)
  AI7 ──→ (未接)
  AI8 ──→ (未接)
  
  AGND ─→ ESP32-S3 GND (必须连接！)
```

### 第 2 步：烧录固件

```powershell
cd c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\test_jig_firmware
idf.py build flash
```

### 第 3 步：运行 GUI

```powershell
cd c:\Users\h1576\Desktop\Roam\pe_code\pe_board\tools\factory_gui
python factory_gui.py
```

### 第 4 步：连接串口

1. **刷新串口：** 点击 "刷新串口" 按钮
2. **选择端口：**
   - 控制口(治具): COM24 (USB-Serial-JTAG)
   - 刷机口(DUT): COM24 (同一个，用于接收数据)
3. **连接：** 点击 "连接" 按钮

### 第 5 步：观察电压显示

**预期结果：**
- GUI 每 2 秒自动更新一次电压表格
- AI1 和 AI4 应显示接近的电压值（如 4.620V）
- 未接通道显示 0.000V，状态为 "未接"
- 有电压的通道显示绿色 "OK"

**示例显示：**
```
┌────────┬──────────┬────────┐
│ 通道   │ 电压(V)  │ 状态   │
├────────┼──────────┼────────┤
│ AI1    │ 4.620    │ OK     │ ← 绿色背景
│ AI2    │ 0.000    │ 未接   │ ← 灰色背景
│ AI3    │ 0.000    │ 未接   │
│ AI4    │ 4.612    │ OK     │ ← 绿色背景
│ AI5    │ 0.000    │ 未接   │
│ AI6    │ 0.000    │ 未接   │
│ AI7    │ 0.000    │ 未接   │
│ AI8    │ 0.000    │ 未接   │
└────────┴──────────┴────────┘
```

### 第 6 步：日志验证

在 GUI 的日志窗口中应该看到：

```
VADC: === Voltage Values (工程量，单位 mV) ===
VADC:   AI1: Raw= 4801 -> Calibrated= 4620 mV ( 4.620 V) [Gain=1.0000, Offset=-181]
VADC:   AI2: Raw=    0 =     0 mV ( 0.000 V)
VADC:   AI3: Raw=    0 =     0 mV ( 0.000 V)
VADC:   AI4: Raw= 4612 =  4612 mV ( 4.612 V)
...
VADC: ======================
VOLTAGE_ADC: {"ch1":4620,"ch2":0,"ch3":0,"ch4":4612,"ch5":0,"ch6":0,"ch7":0,"ch8":0}
```

## 状态判断规则

| 电压范围        | 状态   | 颜色     | 说明               |
|-----------------|--------|----------|--------------------|
| > 10000 mV      | 过压   | 浅红色   | 超出量程           |
| < 0 mV          | 异常   | 浅红色   | 负电压（错误）     |
| = 0 mV          | 未接   | 浅灰色   | 通道未连接         |
| 0 ~ 10000 mV    | OK     | 浅绿色   | 正常范围           |

## 故障排查

### 问题 1：GUI 中电压表不更新

**检查：**
1. COM24 是否正确连接？
   ```powershell
   # 查看端口
   mode COM24
   ```

2. 固件是否正在发送数据？
   - 打开串口助手，连接 COM24
   - 应该每 2 秒看到 `VOLTAGE_ADC: {...}`

3. GUI 日志中是否有电压数据？
   - 如果有原始日志但表格不更新 → 检查 GUI 代码
   - 如果没有原始日志 → 检查串口连接

### 问题 2：电压值不准确

**检查：**
1. 校准参数是否正确？
   - 查看 `voltage_adc.c` 中的 `voltage_adc_load_default_calibration()`
   - 根据实际测量调整偏移量

2. AGND 和 GND 是否相连？
   - **必须连接**，否则测量会漂移

3. 电压源是否稳定？
   - 用万用表实际测量电压源输出

### 问题 3：所有通道显示异常值

**可能原因：**
1. Modbus 通信失败
   - 检查 UART1 (GPIO17/18) 接线
   - 查看日志是否有 Modbus 错误

2. K10-3U8 未上电
   - 检查模块电源（24V）

3. 波特率不匹配
   - 确认 K10-3U8 配置为 9600 baud

## 高级功能

### 自定义电压阈值

如果需要对特定通道设置阈值报警，修改 GUI 代码：

```python
def _parse_voltage_adc(self, text: str):
    # ... 现有代码 ...
    
    # 自定义阈值示例
    thresholds = {
        0: (4500, 5500),  # AI1: 4.5V~5.5V
        3: (4500, 5500),  # AI4: 4.5V~5.5V
        # 其他通道不检查
    }
    
    for i in range(8):
        # ... 现有代码 ...
        
        # 阈值检查
        if i in thresholds:
            min_mv, max_mv = thresholds[i]
            if voltage_mv < min_mv or voltage_mv > max_mv:
                status = "超限"
                color = QColor(255, 165, 0)  # 橙色
```

### 数据记录

如果需要记录电压数据到文件：

```python
def _parse_voltage_adc(self, text: str):
    # ... 现有代码 ...
    
    # 记录到 CSV
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open("voltage_log.csv", "a") as f:
        voltages = [data[f"ch{i+1}"]/1000.0 for i in range(8)]
        f.write(f"{timestamp},{','.join(map(str, voltages))}\n")
```

## 性能指标

- **更新频率：** 每 2 秒（由固件 `voltage_task` 控制）
- **精度：** ±10 mV（校准后，取决于 K10-3U8 和校准质量）
- **响应延迟：** < 100 ms（GUI 轮询频率 20Hz）
- **显示精度：** 3 位小数（0.001V = 1mV）

## 下一步优化

1. **可配置更新频率：** 在 GUI 中添加滑块调整采集频率
2. **历史曲线图：** 使用 matplotlib 显示电压变化趋势
3. **自动校准向导：** GUI 中集成校准流程
4. **数据导出：** 导出电压记录到 Excel
5. **多治具支持：** 同时监控多个测试治具
