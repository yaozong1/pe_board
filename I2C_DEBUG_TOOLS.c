// I2C诊断和故障排除工具
// 添加到motion_sensor.c的底部，用于调试I2C问题

// 详细的GPIO检查
static void check_gpio_status(void)
{
    ESP_LOGI(TAG, "=== GPIO Status Check ===");
    
    // 检查SDA引脚
    esp_err_t ret = gpio_reset_pin(MOTION_SENSOR_SDA_PIN);
    ESP_LOGI(TAG, "SDA Pin %d reset result: %s", MOTION_SENSOR_SDA_PIN, esp_err_to_name(ret));
    
    ret = gpio_set_direction(MOTION_SENSOR_SDA_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    ESP_LOGI(TAG, "SDA Pin %d set direction result: %s", MOTION_SENSOR_SDA_PIN, esp_err_to_name(ret));
    
    ret = gpio_set_pull_mode(MOTION_SENSOR_SDA_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "SDA Pin %d set pullup result: %s", MOTION_SENSOR_SDA_PIN, esp_err_to_name(ret));
    
    // 检查SCL引脚
    ret = gpio_reset_pin(MOTION_SENSOR_SCL_PIN);
    ESP_LOGI(TAG, "SCL Pin %d reset result: %s", MOTION_SENSOR_SCL_PIN, esp_err_to_name(ret));
    
    ret = gpio_set_direction(MOTION_SENSOR_SCL_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    ESP_LOGI(TAG, "SCL Pin %d set direction result: %s", MOTION_SENSOR_SCL_PIN, esp_err_to_name(ret));
    
    ret = gpio_set_pull_mode(MOTION_SENSOR_SCL_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "SCL Pin %d set pullup result: %s", MOTION_SENSOR_SCL_PIN, esp_err_to_name(ret));
    
    // 读取引脚状态
    int sda_level = gpio_get_level(MOTION_SENSOR_SDA_PIN);
    int scl_level = gpio_get_level(MOTION_SENSOR_SCL_PIN);
    ESP_LOGI(TAG, "Pin levels: SDA=%d, SCL=%d (should be 1 if pullups working)", sda_level, scl_level);
}

// 简化的I2C检测，不使用I2C驱动
static void manual_i2c_test(void)
{
    ESP_LOGI(TAG, "=== Manual I2C Test ===");
    
    // 先释放I2C驱动
    i2c_driver_delete(I2C_MASTER_NUM);
    
    // 手动检查GPIO
    check_gpio_status();
    
    // 重新初始化I2C
    i2c_master_init();
}

// 低频率I2C测试
static bool test_low_speed_i2c(void)
{
    ESP_LOGI(TAG, "=== Testing Low Speed I2C (10kHz) ===");
    
    // 先删除现有驱动
    i2c_driver_delete(I2C_MASTER_NUM);
    
    // 低速配置
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MOTION_SENSOR_SDA_PIN,
        .scl_io_num = MOTION_SENSOR_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 10000,  // 10kHz 非常低速
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Low speed I2C param config failed: %s", esp_err_to_name(err));
        return false;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Low speed I2C driver install failed: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "Low speed I2C initialized successfully");
    
    // 等待一会儿
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 尝试扫描设备
    i2c_scan_devices();
    
    return true;
}