# GNSS 日志控制宏使用说明

## 新增功能

已添加 `GNSS_PARSE_LOG_ENABLED` 宏来控制GNSS解析日志的输出，减少冗余的调试信息。

## 配置选项

在 `hardware_config.h` 中有以下三个GNSS日志控制宏：

### 1. GNSS_RAW_LOG_ENABLED
```c
#define GNSS_RAW_LOG_ENABLED 0  // 默认关闭
```
- **功能**: 控制原始NMEA数据输出
- **输出内容**: 完整的NMEA语句（如 $GPGGA,123456.00,...)
- **使用场景**: 调试NMEA解析问题时开启

### 2. GNSS_UBX_LOG_ENABLED  
```c
#define GNSS_UBX_LOG_ENABLED 1  // 默认开启
```
- **功能**: 控制UBX协议相关日志
- **输出内容**: UBX帧解析、ACK/NAK响应等
- **使用场景**: 调试UBX命令和低功耗模式

### 3. GNSS_PARSE_LOG_ENABLED (新增)
```c
#define GNSS_PARSE_LOG_ENABLED 0  // 默认关闭
```
- **功能**: 控制GNSS解析结果输出
- **输出内容**: 
  - `RMC: time=xxx status=xxx lat=xxx lng=xxx`
  - `GGA: fixQ=x sats=x HDOP=x.x -> FIXED/NO FIX`
  - `GSA: dim=NO FIX (val=1) HDOP=100.0`
  - `FIX SUMMARY: RMC=xxx | GGA:xxx | GSA:xxx => FIXED/NO FIX`

## 修改前后对比

### 修改前 (冗余输出)
```
I (671555) GNSS: GSA: dim=NO FIX (val=1) HDOP=100.0
I (671555) GNSS: GSA: dim=NO FIX (val=1) HDOP=100.0  
I (671565) GNSS: GSA: dim=NO FIX (val=1) HDOP=100.0
I (671785) GNSS: GSA: dim=NO FIX (val=1) HDOP=100.0
I (671785) GNSS: RMC: time=000509.700 status=V lat= lng= spd(kn)= crs=
I (672555) GNSS: GGA: fixQ=0 sats=0 HDOP=100.0 -> NO FIX
I (672555) GNSS: GSA: dim=NO FIX (val=1) HDOP=100.0
```

### 修改后 (清洁输出)
```
I (671555) GNSS: RMC fix state changed: NO FIX
I (672555) GNSS: FIX state changed by GGA: NO FIX
```

## 如何启用详细日志

如果需要临时查看详细的GNSS解析信息，可以：

### 方法1: 修改宏定义
在 `hardware_config.h` 中修改：
```c
#define GNSS_PARSE_LOG_ENABLED 1  // 启用解析日志
```

### 方法2: 编译时定义
```bash
idf.py build -DGNSS_PARSE_LOG_ENABLED=1
```

## 保留的重要日志

以下关键状态变化日志仍然会输出，用于监控系统状态：

- ✅ `RMC fix state changed: FIXED/NO FIX`
- ✅ `FIX state changed by GGA: FIXED/NO FIX`  
- ✅ `ACK PMTK161` / `ACK PMTK225`
- ✅ `Wake GNSS: about to exit Standby`
- ✅ `GNSS fix acquired, bringing up EG915 for MQTT publish`

## 日志级别总结

| 宏开关 | 日志类型 | 默认状态 | 用途 |
|--------|----------|----------|------|
| GNSS_RAW_LOG | 原始NMEA | 关闭 | 协议调试 |
| GNSS_UBX_LOG | UBX协议 | 开启 | 低功耗调试 |
| GNSS_PARSE_LOG | 解析结果 | 关闭 | 定位状态监控 |
| ESP_LOGI | 状态变化 | 始终开启 | 系统运行监控 |

## 性能优化

关闭 `GNSS_PARSE_LOG_ENABLED` 的好处：

1. **减少串口输出** - 降低CPU占用
2. **清洁日志** - 重要信息更容易发现
3. **节省存储** - 日志文件更小
4. **提高响应** - 减少中断处理时间

这样既保持了必要的状态监控，又避免了冗余的调试信息输出！