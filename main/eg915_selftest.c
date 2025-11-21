#include "eg915_selftest.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#include "hardware_config.h"
#include "mqtt_module.h"   // 复用 send_at_command / wait_for_ok / uart_read_response / modem_power_cycle / mqtt_init
#include "motion_sensor.h" // 运动传感器自测
#include "rs485_module.h"  // RS485 自测
#include "can_module.h"    // CAN 自测
#include "battery_module.h"// 电池/ADC 自测
#include "gnss_module.h"   // GNSS 自测（直接读UART）
#include "gpio_module.h"   // IGN 光耦自测

static const char *TAG_ST = "SELFTEST";

// GUI 发送的握手 ACK，收到后输出一次自测结果
#define GUI_SELFTEST_ACK "!GUI_SELFTEST_ACK"

typedef struct {
    bool eg915_ok;
    char eg915_imei[32];  // IMEI number
    char eg915_iccid[32]; // ICCID number
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
    // IBL (IO2) ADC
    uint32_t ibl_mv;
    bool     ibl_ok;
    // IGN optocoupler
    bool ign_tested;
    bool ign_pass;
} selftest_report_t;

bool eg915_at_handshake_once(unsigned timeout_ms, int retries)
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
}

// Get IMEI from EG915 module
static bool eg915_get_imei(char *imei_buf, size_t buf_size)
{
    if (!imei_buf || buf_size < 16) return false;
    
    memset(imei_buf, 0, buf_size);
    ESP_LOGI(TAG_ST, "Getting IMEI...");
    send_at_command("AT+GSN");
    
    char resp[128];
    int len = uart_read_response(resp, sizeof(resp), 1000);
    if (len > 0) {
        // Find IMEI (usually 15 digits)
        char *p = resp;
        while (*p) {
            if (*p >= '0' && *p <= '9') {
                int digits = 0;
                char *start = p;
                while (p[digits] >= '0' && p[digits] <= '9') digits++;
                if (digits == 15) {  // IMEI is 15 digits
                    strncpy(imei_buf, start, 15);
                    imei_buf[15] = '\0';
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

// Get ICCID from SIM card
static bool eg915_get_iccid(char *iccid_buf, size_t buf_size)
{
    if (!iccid_buf || buf_size < 20) return false;
    
    memset(iccid_buf, 0, buf_size);
    ESP_LOGI(TAG_ST, "Getting ICCID...");
    send_at_command("AT+CCID");
    
    char resp[128];
    int len = uart_read_response(resp, sizeof(resp), 1000);
    if (len > 0) {
        // Find ICCID (usually 19-20 digits)
        // Format might be +CCID: 89860123456789012345
        char *p = strstr(resp, "+CCID:");
        if (p) {
            p += 6;  // Skip "+CCID:"
            while (*p == ' ') p++;  // Skip spaces
            
            int digits = 0;
            while (p[digits] >= '0' && p[digits] <= '9') digits++;
            if (digits >= 19 && digits <= 20) {
                strncpy(iccid_buf, p, digits);
                iccid_buf[digits] = '\0';
                ESP_LOGI(TAG_ST, "ICCID: %s", iccid_buf);
                return true;
            }
        }
    }
    ESP_LOGW(TAG_ST, "Failed to get ICCID");
    strcpy(iccid_buf, "N/A");
    return false;
}

static void drain_boot_urcs(uint32_t ms)
{
    // 智能URC drain: 最多等ms毫秒，但连续200ms无数据就提前退出
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_data = t0;
    char buf[256];
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(ms)) {
        int n = uart_read_bytes(UART_EG915U_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            last_data = xTaskGetTickCount();
            buf[n] = '\0';
            ESP_LOGI(TAG_ST, "URC: %s", buf);
        }
        // 连续200ms无数据，认为URC已清空
        if ((xTaskGetTickCount() - last_data) > pdMS_TO_TICKS(200)) {
            ESP_LOGI(TAG_ST, "URC drain complete (no data for 200ms)");
            break;
        }
    }
}

static bool selftest_motion(selftest_report_t *r)
{
    static bool motion_inited = false;
    bool ok = true;
    if (!motion_inited) {
        ok = motion_sensor_init();
        motion_inited = ok;
    }
    if (!ok) {
        r->motion_ok = false;
        r->motion_mag = 0.0f;
        ESP_LOGW(TAG_ST, "Motion sensor init failed");
        return false;
    }
    // 读一次加速度
    lis2dh12_accel_data_t acc;
    if (lis2dh12_read_accel_data(&acc)) {
        float mag = lis2dh12_calculate_magnitude(&acc);
        r->motion_ok = true;
        r->motion_mag = mag;
        ESP_LOGI(TAG_ST, "Motion: X=%.3fg Y=%.3fg Z=%.3fg Mag=%.3fg", acc.x, acc.y, acc.z, mag);
        return true;
    }
    r->motion_ok = false;
    r->motion_mag = 0.0f;
    ESP_LOGW(TAG_ST, "Motion sensor read failed");
    return false;
}

static void selftest_rs485(selftest_report_t *r)
{
    r->rs485_inited = rs485_init(115200);
    r->rs485_rx_bytes = 0;
    r->rs485_written = -1;
    r->rs485_pass = false;
    if (!r->rs485_inited) {
        ESP_LOGW(TAG_ST, "RS485 init failed");
        return;
    }
    // 严格要求收到 01 02 03 04 05 06 07 08
    const uint8_t patt[8] = {1,2,3,4,5,6,7,8};
    int w = rs485_write(patt, sizeof(patt), pdMS_TO_TICKS(100));
    r->rs485_written = w;
    ESP_LOGI(TAG_ST, "RS485 write %d bytes", w);
    uint8_t buf[64];
    int total = 0;
    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(800)) {
        int n = rs485_read(buf + total, sizeof(buf) - total, pdMS_TO_TICKS(50));
        if (n > 0) {
            total += n;
            r->rs485_rx_bytes = total;
            // 滑动窗口匹配
            for (int i = 0; i + 8 <= total; ++i) {
                if (memcmp(buf + i, patt, 8) == 0) {
                    r->rs485_pass = true;
                    break;
                }
            }
            if (r->rs485_pass) break;
            if (total >= (int)sizeof(buf)) break;
        }
    }
    rs485_deinit();
}

static void selftest_can(selftest_report_t *r)
{
    // 检测CAN_EN_PIN (GPIO41) 的电平状态
    int can_en_level = gpio_get_level(CAN_EN_PIN);
    ESP_LOGI(TAG_ST, "CAN_EN_PIN (IO%d) level before init: %d (should be 0 for normal, 1 for shutdown)", CAN_EN_PIN, can_en_level);
    
    if (can_en_level != 0) {
        ESP_LOGE(TAG_ST, "ERROR: CAN_EN_PIN should be LOW(0) but is HIGH(1) - transceiver is in shutdown mode!");
    }
    
    r->can_inited = can_module_init(CAN_BITRATE_250K);
    r->can_started = false;
    r->can_state = -1;
    r->can_pass = false;
    if (!r->can_inited) {
        ESP_LOGW(TAG_ST, "CAN init failed");
        // 再次检测GPIO41,看是否被错误拉高
        can_en_level = gpio_get_level(CAN_EN_PIN);
        ESP_LOGE(TAG_ST, "CAN_EN_PIN (IO%d) level after init failure: %d", CAN_EN_PIN, can_en_level);
        return;
    }
    
    // 初始化后再次检测
    can_en_level = gpio_get_level(CAN_EN_PIN);
    ESP_LOGI(TAG_ST, "CAN_EN_PIN (IO%d) level after init: %d", CAN_EN_PIN, can_en_level);
    
    r->can_started = can_module_start();
    if (!r->can_started) {
        ESP_LOGW(TAG_ST, "CAN start failed");
        // 启动失败后检测GPIO41
        can_en_level = gpio_get_level(CAN_EN_PIN);
        ESP_LOGE(TAG_ST, "CAN_EN_PIN (IO%d) level after start failure: %d", CAN_EN_PIN, can_en_level);
    }
    
    can_state_t st = can_get_state();
    r->can_state = (int)st;
    ESP_LOGI(TAG_ST, "CAN state=%d started=%d", r->can_state, r->can_started);

    if (r->can_started) {
        // 发送前先清空接收队列,避免读到旧数据
        can_message_t dummy;
        int flushed = 0;
        while (can_receive_message(&dummy, 10)) {
            flushed++;
        }
        if (flushed > 0) {
            ESP_LOGW(TAG_ST, "CAN: Flushed %d old messages from RX queue", flushed);
        }
        
        can_message_t msg = {
            .identifier = 0x321,
            .format = CAN_FRAME_STD,
            .type = CAN_MSG_TYPE_DATA,
            .data_length = 8,
            .data = {1,2,3,4,5,6,7,8}
        };
        
        bool sent = can_send_message(&msg, 100);
        ESP_LOGI(TAG_ST, "CAN send result: %s", sent ? "OK" : "FAIL");

        TickType_t t0 = xTaskGetTickCount();
        can_message_t rx;
        int rx_count = 0;
        while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(2000)) {
            if (can_receive_message(&rx, 50)) {
                rx_count++;
                ESP_LOGI(TAG_ST, "CAN RX#%d: ID=0x%03lx, len=%d, data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                    rx_count, (unsigned long)rx.identifier, rx.data_length,
                    rx.data[0], rx.data[1], rx.data[2], rx.data[3],
                    rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
                
                // 严格匹配: 必须是8字节且内容完全一致
                if (rx.data_length == 8 && memcmp(rx.data, msg.data, 8) == 0) {
                    ESP_LOGI(TAG_ST, "CAN: Received matching pattern {01 02 03 04 05 06 07 08} - PASS");
                    r->can_pass = true;
                    break;
                } else {
                    ESP_LOGW(TAG_ST, "CAN: Data mismatch, continue waiting...");
                }
            }
        }
        
        if (!r->can_pass) {
            ESP_LOGW(TAG_ST, "CAN: No matching loopback received (rx_count=%d)", rx_count);
        }
    }

    // 结束后清理
    can_module_stop();
    can_module_deinit();
}

static void selftest_battery(selftest_report_t *r)
{
    static bool bat_inited = false;
    if (!bat_inited) { battery_init(); bat_inited = true; }
    float v = read_battery_voltage();
    r->battery_v = v;
    // EBL判定标准: 0.49V ±10% (0.441V - 0.539V)
    r->battery_ok = (v >= 0.441f && v <= 0.539f);
    ESP_LOGI(TAG_ST, "Battery voltage=%.2fV (ok=%d, expect 0.49V±10%%)", v, r->battery_ok);
}

static void selftest_gnss(selftest_report_t *r)
{
    // 初始化GNSS UART并上电
    static bool gnss_inited = false;
    if (!gnss_inited) { gnss_init(); gnss_inited = true; }
    gnss_enable(true);
    gnss_reset_release();
    vTaskDelay(pdMS_TO_TICKS(500));

    // 尝试读取少量数据
    uint8_t buf[128];
    int total = 0;
    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(800)) {
        int n = uart_read_bytes(GNSS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n > 0) { total += n; if (total >= 32) break; }
    }
    r->gnss_bytes = total;
    r->gnss_uart_ok = (total > 0);
    ESP_LOGI(TAG_ST, "GNSS UART bytes=%d (ok=%d)", total, r->gnss_uart_ok);
}

static void selftest_ign(selftest_report_t *r)
{
    r->ign_tested = true;
    // 测试IGN光耦：等待1秒检测电平变化（高->低->高）
    // 治具的IGN_TEST_PIN每100ms切换，1秒内足够检测到完整的转换
    ESP_LOGI(TAG_ST, "Testing IGN optocoupler (timeout=1000ms)...");
    bool pass = ign_test_detect_transition(1000);
    r->ign_pass = pass;
    ESP_LOGI(TAG_ST, "IGN optocoupler: %s", pass ? "PASS" : "FAIL");
}

static void selftest_ibl(selftest_report_t *r)
{
    // 读取 IO2 的 ADC 电压 (mV)
    r->ibl_mv = read_io2_voltage_mv();
    r->ibl_ok = (r->ibl_mv > 0);
    ESP_LOGI(TAG_ST, "IBL (IO2) = %lu mV (ok=%d)", (unsigned long)r->ibl_mv, r->ibl_ok);
}

static void print_summary(const selftest_report_t *r)
{
    ESP_LOGI(TAG_ST,
        "SELFTEST SUMMARY:\n"
        "{\n"
        "  \"eg915_ok\": %s,\n"
        "  \"eg915_imei\": \"%s\",\n"
        "  \"eg915_iccid\": \"%s\",\n"
        "  \"motion\": { \"ok\": %s, \"mag\": %.3f },\n"
    "  \"rs485\": { \"inited\": %s, \"written\": %d, \"rx_bytes\": %d, \"pass\": %s },\n"
    "  \"can\": { \"inited\": %s, \"started\": %s, \"state\": %d, \"pass\": %s },\n"
        "  \"gnss\": { \"uart_ok\": %s, \"bytes\": %d },\n"
        "  \"battery\": { \"ok\": %s, \"voltage\": %.2f },\n"
        "  \"ibl_mv\": %lu,\n"
        "  \"ign\": { \"tested\": %s, \"pass\": %s }\n"
        "}\n",
        r->eg915_ok ? "true" : "false",
        r->eg915_imei,
        r->eg915_iccid,
        r->motion_ok ? "true" : "false", r->motion_mag,
    r->rs485_inited ? "true" : "false", r->rs485_written, r->rs485_rx_bytes, r->rs485_pass ? "true" : "false",
    r->can_inited ? "true" : "false", r->can_started ? "true" : "false", r->can_state, r->can_pass ? "true" : "false",
        r->gnss_uart_ok ? "true" : "false", r->gnss_bytes,
        r->battery_ok ? "true" : "false", r->battery_v,
        (unsigned long)r->ibl_mv,
        r->ign_tested ? "true" : "false", r->ign_pass ? "true" : "false"
    );
}

static void print_human_summary(const selftest_report_t *r)
{
    bool rs485_ok = r->rs485_pass; // 严格要求收到 01..08
    bool can_ok   = r->can_pass;   // 严格要求收到 01..08
    bool ign_ok   = r->ign_pass;   // IGN光耦测试通过
    bool overall  = r->eg915_ok && r->motion_ok && rs485_ok && can_ok && r->gnss_uart_ok && r->battery_ok && ign_ok;

    ESP_LOGI(TAG_ST, "================= SELFTEST RESULT =================");
    ESP_LOGI(TAG_ST, "EG915       : %s", r->eg915_ok     ? "PASS" : "FAIL");
    ESP_LOGI(TAG_ST, "Motion      : %s (|a|=%.3fg)", r->motion_ok ? "PASS" : "FAIL", r->motion_mag);
    ESP_LOGI(TAG_ST, "RS485       : %s (written=%d, rx=%d)", rs485_ok ? "PASS" : "FAIL", r->rs485_written, r->rs485_rx_bytes);
    ESP_LOGI(TAG_ST, "CAN         : %s (inited=%d, started=%d, state=%d)", can_ok ? "PASS" : "FAIL", r->can_inited, r->can_started, r->can_state);
    ESP_LOGI(TAG_ST, "GNSS UART   : %s (bytes=%d)", r->gnss_uart_ok ? "PASS" : "FAIL", r->gnss_bytes);
    ESP_LOGI(TAG_ST, "Battery/ADC : %s (V=%.2f)", r->battery_ok ? "PASS" : "FAIL", r->battery_v);
    ESP_LOGI(TAG_ST, "IGN Opto    : %s", ign_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG_ST, "---------------------------------------------------");
    ESP_LOGI(TAG_ST, "OVERALL     : %s", overall ? "PASS" : "FAIL");
}

// 在COM6(USB-JTAG)上输出清晰的格式化表格,便于用户直接通过串口查看
static void print_com6_formatted_result(const selftest_report_t *r)
{
    bool rs485_ok = r->rs485_pass;
    bool can_ok   = r->can_pass;
    bool ign_ok   = r->ign_pass;
    bool overall  = r->eg915_ok && r->motion_ok && rs485_ok && can_ok && r->gnss_uart_ok && r->battery_ok && ign_ok;

    // 使用ESP_LOGI输出格式化表格,确保在COM6上可见
    ESP_LOGI(TAG_ST, "========== FORMATTED REPORT START ==========");
    ESP_LOGI(TAG_ST, " ");
    ESP_LOGI(TAG_ST, "╔═══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG_ST, "║              PE BOARD FACTORY SELF-TEST REPORT                ║");
    ESP_LOGI(TAG_ST, "╠═══════════════════════════════════════════════════════════════╣");
    ESP_LOGI(TAG_ST, "║ Test Item          │ Result  │ Details                        ║");
    ESP_LOGI(TAG_ST, "╟────────────────────┼─────────┼────────────────────────────────╢");
    
    ESP_LOGI(TAG_ST, "║ EG915 Module       │ %-7s │ AT command response            ║", 
           r->eg915_ok ? "PASS" : "FAIL");
    
    ESP_LOGI(TAG_ST, "║ Motion Sensor      │ %-7s │ Magnitude: %.3f g             ║", 
           r->motion_ok ? "PASS" : "FAIL", r->motion_mag);
    
    ESP_LOGI(TAG_ST, "║ RS485 Loopback     │ %-7s │ TX:%d RX:%d bytes            ║", 
           rs485_ok ? "PASS" : "FAIL", r->rs485_written, r->rs485_rx_bytes);
    
    ESP_LOGI(TAG_ST, "║ CAN Bus Loopback   │ %-7s │ Init:%d Start:%d State:%d     ║", 
           can_ok ? "PASS" : "FAIL", r->can_inited, r->can_started, r->can_state);
    
    ESP_LOGI(TAG_ST, "║ GNSS UART          │ %-7s │ Received: %d bytes            ║", 
           r->gnss_uart_ok ? "PASS" : "FAIL", r->gnss_bytes);
    
    ESP_LOGI(TAG_ST, "║ Battery Voltage    │ %-7s │ Voltage: %.2f V               ║", 
           r->battery_ok ? "PASS" : "FAIL", r->battery_v);
            ESP_LOGI(TAG_ST, "║ IBL (IO2)          │ %-7s │ %4lu mV                        ║", 
                r->ibl_ok ? "OK" : "N/A", (unsigned long)r->ibl_mv);
    
    ESP_LOGI(TAG_ST, "║ IGN Optocoupler    │ %-7s │ Signal transition detected     ║", 
           ign_ok ? "PASS" : "FAIL");
    
    ESP_LOGI(TAG_ST, "╠════════════════════╧═════════╧════════════════════════════════╣");
    ESP_LOGI(TAG_ST, "║ OVERALL RESULT:  %-44s  ║", 
           overall ? "✓ ALL TESTS PASSED" : "✗ SOME TESTS FAILED");
    ESP_LOGI(TAG_ST, "╚═══════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG_ST, " ");
    
    // 额外输出调试详情(如果有失败项)
    if (!overall) {
        ESP_LOGI(TAG_ST, "┌─── FAILURE DETAILS ───────────────────────────────────────────┐");
        if (!r->eg915_ok)     ESP_LOGI(TAG_ST, "│ ✗ EG915: No AT response - check module power/UART connection │");
        if (!r->motion_ok)    ESP_LOGI(TAG_ST, "│ ✗ Motion: Sensor read failed - check I2C connection          │");
        if (!rs485_ok)        ESP_LOGI(TAG_ST, "│ ✗ RS485: Pattern mismatch - check loopback wiring            │");
        if (!can_ok)          ESP_LOGI(TAG_ST, "│ ✗ CAN: Loopback failed - check bus termination/GPIO41        │");
        if (!r->gnss_uart_ok) ESP_LOGI(TAG_ST, "│ ✗ GNSS: No data received - check module power/UART           │");
        if (!r->battery_ok)   ESP_LOGI(TAG_ST, "│ ✗ Battery: Low voltage - check ADC/power supply              │");
        if (!ign_ok)          ESP_LOGI(TAG_ST, "│ ✗ IGN: No transition - check optocoupler/jig connection      │");
        ESP_LOGI(TAG_ST, "└───────────────────────────────────────────────────────────────┘");
        ESP_LOGI(TAG_ST, " ");
    }
    ESP_LOGI(TAG_ST, "========== FORMATTED REPORT END ==========");
}

// 将 JSON 摘要通过 UART0 的 U0TXD/U0RXD(43/44) 发送给治具
static void emit_summary_over_uart0(const selftest_report_t *r)
{
    // 构造与 print_summary 相同的 JSON 文本
    char json_buf[512];
    int n = snprintf(json_buf, sizeof(json_buf),
        "SELFTEST SUMMARY:\n"
        "{\n"
        "  \"eg915_ok\": %s,\n"
        "  \"eg915_imei\": \"%s\",\n"
        "  \"eg915_iccid\": \"%s\",\n"
        "  \"motion\": { \"ok\": %s, \"mag\": %.3f },\n"
        "  \"rs485\": { \"inited\": %s, \"written\": %d, \"rx_bytes\": %d, \"pass\": %s },\n"
        "  \"can\": { \"inited\": %s, \"started\": %s, \"state\": %d, \"pass\": %s },\n"
        "  \"gnss\": { \"uart_ok\": %s, \"bytes\": %d },\n"
        "  \"battery\": { \"ok\": %s, \"voltage\": %.2f },\n"
        "  \"ibl_mv\": %lu,\n"
        "  \"ign\": { \"tested\": %s, \"pass\": %s }\n"
        "}\n",
        r->eg915_ok ? "true" : "false",
        r->eg915_imei,
        r->eg915_iccid,
        r->motion_ok ? "true" : "false", r->motion_mag,
        r->rs485_inited ? "true" : "false", r->rs485_written, r->rs485_rx_bytes, r->rs485_pass ? "true" : "false",
        r->can_inited ? "true" : "false", r->can_started ? "true" : "false", r->can_state, r->can_pass ? "true" : "false",
        r->gnss_uart_ok ? "true" : "false", r->gnss_bytes,
        r->battery_ok ? "true" : "false", r->battery_v,
        (unsigned long)r->ibl_mv,
        r->ign_tested ? "true" : "false", r->ign_pass ? "true" : "false"
    );
    if (n <= 0) return;

    // 独立安装 UART0 驱动到 43/44，发送后删除，避免与 RS485/其他占用长期冲突
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 若 UART0 之前被其他模块占用/删除，重新安装；失败则放弃透传
    // 为避免 RS485 总线上也出现同样的数据：
    // 1) 显式复位 RS485_TX_PIN 的矩阵映射，避免残留的 UART0 TX 映射仍驱动 GPIO10
    // 2) 临时拉低 RS_EN（若存在），让收发器进入接收/高阻状态
    int rs_en_prev = -1;
#ifdef RS_EN
    rs_en_prev = gpio_get_level(RS_EN);
    gpio_set_level(RS_EN, 0);
#endif
    gpio_reset_pin(RS485_TX_PIN);

    if (uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0) == ESP_OK) {
        uart_param_config(UART_NUM_0, &cfg);
        uart_set_pin(UART_NUM_0, U0TXD_PIN, U0RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_write_bytes(UART_NUM_0, json_buf, strlen(json_buf));
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(200));
        uart_driver_delete(UART_NUM_0);
    }

#ifdef RS_EN
    if (rs_en_prev >= 0) {
        gpio_set_level(RS_EN, rs_en_prev);
    }
#endif
}

static void wait_for_gui_ack_once(void)
{
    ESP_LOGI(TAG_ST, "Waiting for GUI ACK '%s'...", GUI_SELFTEST_ACK);
    const int ack_buf_len = 64;
    char ack_buf[ack_buf_len];
    int ack_pos = 0;
    memset(ack_buf, 0, sizeof(ack_buf));
    while (1) {
        uint8_t ch;
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
        if (n == 1) {
            if (ack_pos < ack_buf_len - 1) {
                ack_buf[ack_pos++] = (char)ch;
                ack_buf[ack_pos] = '\0';
            } else {
                memmove(ack_buf, ack_buf + 1, ack_buf_len - 2);
                ack_buf[ack_buf_len - 2] = (char)ch;
                ack_buf[ack_buf_len - 1] = '\0';
            }
            if (strstr(ack_buf, GUI_SELFTEST_ACK)) {
                ESP_LOGI(TAG_ST, "GUI ACK received.");
                break;
            }
        }
    }
}

void Selftest_task(void *pv)
{
    (void)pv;
    ESP_LOGI(TAG_ST, "Self-test task starting (RUN TESTS NOW -> WAIT ACK -> OUTPUT)...");

    // 基础初始化
    mqtt_init();
    int pwr_state = gpio_get_level(EG915U_POWER);
    if (pwr_state == 0) {
        ESP_LOGI(TAG_ST, "Power OFF -> pulsing PWRKEY...");
        modem_power_cycle();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGI(TAG_ST, "Draining boot URCs for 3s...");
    drain_boot_urcs(3000);

    // 立即执行各项测试（但先不通过 UART0 发回结果）
    selftest_report_t rep = (selftest_report_t){0};
    rep.eg915_ok = eg915_at_handshake_once(500, 3);
    ESP_LOGI(TAG_ST, "EG915 AT: %s", rep.eg915_ok ? "PASS" : "FAIL");
    
    // Get IMEI and ICCID if AT handshake passed
    if (rep.eg915_ok) {
        eg915_get_imei(rep.eg915_imei, sizeof(rep.eg915_imei));
        eg915_get_iccid(rep.eg915_iccid, sizeof(rep.eg915_iccid));
    } else {
        strcpy(rep.eg915_imei, "N/A");
        strcpy(rep.eg915_iccid, "N/A");
    }
    
    selftest_motion(&rep);
    selftest_rs485(&rep);
    selftest_can(&rep);
    selftest_battery(&rep);
    // 采集 IBL (IO2) 电压
    selftest_ibl(&rep);
    selftest_gnss(&rep);
    selftest_ign(&rep);

    // 等待 GUI ACK 后再输出（无限等待，严格满足“收到ACK才发”）
    if (uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) == ESP_OK) {
        uart_config_t cfg = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        uart_param_config(UART_NUM_0, &cfg);
        uart_set_pin(UART_NUM_0, U0TXD_PIN, U0RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        // 在 UART0 (COM6) 等待 ACK 时，周期性打印调试信息
        const int ack_buf_len = 64;
        char ack_buf[ack_buf_len];
        int ack_pos = 0;
        memset(ack_buf, 0, sizeof(ack_buf));
        int wait_ms = 0;
    // 通过 UART0 (COM6) 主动发一行提示，便于串口工具/GUI识别
    const char *wait_msg = "Waiting for ACK: !GUI_SELFTEST_ACK\r\n";
    uart_write_bytes(UART_NUM_0, wait_msg, strlen(wait_msg));
    ESP_LOGI(TAG_ST, "[COM6][WAIT] DUT自测已完成，等待GUI发送ACK指令: %s", GUI_SELFTEST_ACK);
        while (1) {
            uint8_t ch;
            int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
            wait_ms += 100;
            if (n == 1) {
                if (ack_pos < ack_buf_len - 1) {
                    ack_buf[ack_pos++] = (char)ch;
                    ack_buf[ack_pos] = '\0';
                } else {
                    memmove(ack_buf, ack_buf + 1, ack_buf_len - 2);
                    ack_buf[ack_buf_len - 2] = (char)ch;
                    ack_buf[ack_buf_len - 1] = '\0';
                }
                if (strstr(ack_buf, GUI_SELFTEST_ACK)) {
                    ESP_LOGI(TAG_ST, "[COM6][WAIT] 收到ACK，准备输出自测结果");
                    break;
                }
            }
            if (wait_ms % 1000 == 0) {
                ESP_LOGI(TAG_ST, "[COM6][WAIT] DUT已完成自测，正在等待ACK... 已等待%d ms", wait_ms);
            }
        }
        uart_driver_delete(UART_NUM_0);
    } else {
        ESP_LOGW(TAG_ST, "UART0 driver install failed; cannot wait for ACK. Emitting immediately.");
    }

    // 现在才通过 COM6 输出一次结果
    print_human_summary(&rep);
    vTaskDelay(pdMS_TO_TICKS(100));
    emit_summary_over_uart0(&rep);
    ESP_LOGI(TAG_ST, "Self-test summary emitted after ACK. Task exiting.");
    vTaskDelete(NULL);
}

// start_eg915_at_selftest removed; task is created directly from app_main
