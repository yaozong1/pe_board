"""
为EG915自测添加IMEI和ICCID获取功能

修改内容:
1. 在selftest_report_t结构体中添加IMEI和ICCID字段
2. 添加获取IMEI和ICCID的函数
3. 在JSON payload中包含这些信息
"""

# 首先读取文件
with open('main/eg915_selftest.c', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. 修改结构体，添加IMEI和ICCID字段
old_struct = '''typedef struct {
    bool eg915_ok;
    // Motion
    bool motion_ok;
    float motion_mag;
    // RS485
    bool rs485_inited;
    int  rs485_rx_bytes;
    int  rs485_written;
    bool rs485_pass;
    // CAN
    bool can_inited;
    bool can_started;
    int  can_state; // cast of can_state_t
    bool can_pass;
    // GNSS
    bool gnss_uart_ok;
    int  gnss_bytes;
    // Battery
    bool battery_ok;
    float battery_v;
    // IGN optocoupler
    bool ign_tested;
    bool ign_pass;
} selftest_report_t;'''

new_struct = '''typedef struct {
    bool eg915_ok;
    char eg915_imei[32];  // IMEI号码
    char eg915_iccid[32]; // ICCID号码
    // Motion
    bool motion_ok;
    float motion_mag;
    // RS485
    bool rs485_inited;
    int  rs485_rx_bytes;
    int  rs485_written;
    bool rs485_pass;
    // CAN
    bool can_inited;
    bool can_started;
    int  can_state; // cast of can_state_t
    bool can_pass;
    // GNSS
    bool gnss_uart_ok;
    int  gnss_bytes;
    // Battery
    bool battery_ok;
    float battery_v;
    // IGN optocoupler
    bool ign_tested;
    bool ign_pass;
} selftest_report_t;'''

if old_struct in content:
    content = content.replace(old_struct, new_struct)
    print("✓ Updated selftest_report_t structure")
else:
    print("✗ Failed to find selftest_report_t structure")
    import sys
    sys.exit(1)

# 2. 在eg915_at_handshake_once函数后添加获取IMEI和ICCID的函数
get_info_functions = '''

// 获取EG915模组的IMEI
static bool eg915_get_imei(char *imei_buf, size_t buf_size)
{
    if (!imei_buf || buf_size < 16) return false;
    
    memset(imei_buf, 0, buf_size);
    ESP_LOGI(TAG_ST, "Getting IMEI...");
    send_at_command("AT+GSN");
    
    char resp[128];
    int len = uart_read_response(UART_EG915U_NUM, resp, sizeof(resp), 2000);
    if (len > 0) {
        // 查找IMEI（通常是15位数字）
        char *p = resp;
        while (*p) {
            if (*p >= '0' && *p <= '9') {
                int digits = 0;
                char *start = p;
                while (p[digits] >= '0' && p[digits] <= '9') digits++;
                if (digits == 15) {  // IMEI是15位
                    strncpy(imei_buf, start, 15);
                    imei_buf[15] = '\\0';
                    ESP_LOGI(TAG_ST, "IMEI: %s", imei_buf);
                    return true;
                }
                p += digits;
            } else {
                p++;
            }
        }
    }
    ESP_LOGW(TAG_ST, "Failed to get IMEI");
    strcpy(imei_buf, "N/A");
    return false;
}

// 获取SIM卡的ICCID
static bool eg915_get_iccid(char *iccid_buf, size_t buf_size)
{
    if (!iccid_buf || buf_size < 20) return false;
    
    memset(iccid_buf, 0, buf_size);
    ESP_LOGI(TAG_ST, "Getting ICCID...");
    send_at_command("AT+CCID");
    
    char resp[128];
    int len = uart_read_response(UART_EG915U_NUM, resp, sizeof(resp), 2000);
    if (len > 0) {
        // 查找ICCID（通常是19-20位数字）
        // 格式可能是 +CCID: 89860123456789012345
        char *p = strstr(resp, "+CCID:");
        if (p) {
            p += 6;  // 跳过"+CCID:"
            while (*p == ' ') p++;  // 跳过空格
            
            int digits = 0;
            while (p[digits] >= '0' && p[digits] <= '9') digits++;
            if (digits >= 19 && digits <= 20) {
                strncpy(iccid_buf, p, digits);
                iccid_buf[digits] = '\\0';
                ESP_LOGI(TAG_ST, "ICCID: %s", iccid_buf);
                return true;
            }
        }
    }
    ESP_LOGW(TAG_ST, "Failed to get ICCID");
    strcpy(iccid_buf, "N/A");
    return false;
}
'''

# 在eg915_at_handshake_once函数后插入
marker = '''bool eg915_at_handshake_once(unsigned timeout_ms, int retries)
{
    // 发送AT并等待OK，重试若干次
    for (int i = 1; i <= retries; ++i) {
        ESP_LOGI(TAG_ST, "Handshake attempt %d/%d: sending AT...", i, retries);
        send_at_command("AT");
        if (wait_for_ok(timeout_ms)) {
            ESP_LOGI(TAG_ST, "AT handshake OK on attempt %d", i);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGW(TAG_ST, "AT handshake failed after %d attempts", retries);
    return false;
}'''

new_marker = marker + get_info_functions

if marker in content:
    content = content.replace(marker, new_marker)
    print("✓ Added eg915_get_imei and eg915_get_iccid functions")
else:
    print("✗ Failed to find insertion point for functions")
    import sys
    sys.exit(1)

# 保存修改
with open('main/eg915_selftest.c', 'w', encoding='utf-8') as f:
    f.write(content)

print("\n" + "="*80)
print("第一步完成：添加了IMEI/ICCID字段和获取函数")
print("="*80)
print("\n接下来需要:")
print("1. 在EG915自测通过后调用这些函数")
print("2. 将IMEI/ICCID添加到JSON payload")
print("3. 修改GUI解析和显示")
print("="*80)
