#include <stdio.h>
#include "esp_err.h"
#include "ble_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"

#include "driver/i2c.h"      // ESP-IDF I2C 驱动库
#include "esp_log.h"         // ESP-IDF 日志库
#include "esp_pm.h"
#include "esp_timer.h"



// === I2C initial===
#define I2C_MASTER_SCL_IO 32   // SCL 连接 GPIO 22
#define I2C_MASTER_SDA_IO 33   // SDA 连接 GPIO 21
#define I2C_MASTER_NUM I2C_NUM_0 // 使用 I2C0 端口
#define I2C_MASTER_FREQ_HZ 400000 // I2C 时钟：100kHz
#define ADS1115_ADDR 0x48  // ADS1115 I2C 地址 (ADDR 引脚接 GND)
#define ADS1115_REG_CONVERSION 0x00  // 转换寄存器（存储 ADC 结果）
#define ADS1115_REG_CONFIG 0x01  // 配置寄存器
#define SAMPLE_RATE 250
#define SAMPLE_SIZE 2  // 每个样本 2 字节
#define BUFFER_SIZE (5 * SAMPLE_RATE * SAMPLE_SIZE)

#define TAG "[ESP32-BLE-test]"
uint8_t eeg_data_buffer[BUFFER_SIZE];  // EEG 数据缓存
int sample_index = 0;
uint8_t ads1115_mode = 0;
volatile bool task_running = false;
volatile int cycle_count = 0;  // 控制轮次的计数器

QueueHandle_t ble_queue;  // 定义 BLE 任务队列

// === 低功耗设置 ===
void configure_power_management() {
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
}


// 初始化 I2C
static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,  // 设置为主机模式
        .sda_io_num = I2C_MASTER_SDA_IO,  // 设置 SDA
        .scl_io_num = I2C_MASTER_SCL_IO,  // 设置 SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // 使能上拉
        .scl_pullup_en = GPIO_PULLUP_ENABLE,  // 使能上拉
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // 设定 I2C 时钟频率
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);  // 配置 I2C
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);  // 安装 I2C 驱动
}

// 配置 ADS1115，设置输入通道和采样率
static void ads1115_config(uint8_t config_high, uint8_t config_low) {
    uint8_t config[3] = {ADS1115_REG_CONFIG, config_high, config_low};  
    // 0xC3 0x83:
    // - 0xC3 = 1100 0011
    //   - [15] 1 = 单次转换模式
    //   - [14:12] 100 = AIN0 对 GND 采样
    //   - [11:9]  001 = 4.096V 量程
    // - 0x83 = 1000 0011
    //   - [8:5] 1000 = 128 SPS（采样率）
    //   - [4:0] 0011 = 使能转换

    // 0xC2 : 1100 0010 连续模式
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config, 3, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// 读取 ADS1115 数据
static int16_t ads1115_read_adc() {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ADS1115_REG_CONVERSION, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (data[0] << 8) | data[1];  // 合成 16 位数据
}


void mytask1(void* param)
{
    // uint8_t my_data[20]={0};  // 假设这里是你的数据
    //uint16_t count2 = 100;
    // int64_t start_time = esp_timer_get_time();

    while(1)
    {
        if (sample_index + 1 < BUFFER_SIZE) {
            // int64_t start_time = esp_timer_get_time();

            int16_t sample = ads1115_read_adc();
            eeg_data_buffer[sample_index] = sample & 0xFF;
            eeg_data_buffer[sample_index+1] = (sample >> 8) & 0xFF;
            sample_index += 2;

            vTaskDelay(pdMS_TO_TICKS( 1000 / SAMPLE_RATE)); // 250Hz 采样
            ESP_LOGI(TAG, "index: %d", sample_index);


        }
        else if (sample_index >= BUFFER_SIZE){

            ESP_LOGI(TAG, "EEG Data Buffer: ");
            // int64_t end_time = esp_timer_get_time();
            // int64_t time_diff = end_time - start_time;
            // ESP_LOGI(TAG, "Time difference: %lld microseconds", time_diff);

            // for (int i = 0; i < sample_index; i += 2) {
            //     int16_t value = (eeg_data_buffer[i + 1] << 8) | eeg_data_buffer[i];
            //     ESP_LOGI(TAG, "[%d] %d", i / 2, value);
            // }
            ESP_LOGI(TAG, "channel number: %d", ads1115_mode);    //收到命令，输出收到的值

            ble_set_ch1_value_large(eeg_data_buffer, BUFFER_SIZE);

            sample_index = 0;
            // int64_t start_time = esp_timer_get_time();

        }
    }
}

void IRAM_ATTR timer_callback() {
    if (task_running) {
        ESP_LOGI(TAG, "pass");
        return; // 任务未完成，跳过
    }

    task_running = true;

    if (sample_index + 1 < BUFFER_SIZE) {
        int16_t sample = ads1115_read_adc();

        if (ads1115_mode != 1)
        {
            uint8_t current_channel = (cycle_count % ads1115_mode) + 1;  // 根据模式计算通道
            
            switch (current_channel) {
                case 1:
                    ads1115_config(0xc2, 0xe3);  // 配置通道1
                    break;
                case 2:
                    ads1115_config(0xd2, 0xe3);  // 配置通道2
                    break;
                case 3:
                    ads1115_config(0xe3, 0xe3);  // 配置通道3
                    break;
                case 4:
                    ads1115_config(0xf3, 0xe3);  // 配置通道4
                    break;
                default:
                    ESP_LOGW("task_sampling", "Invalid channel: %d", current_channel);
                    break;
            }
            cycle_count ++;
        }
        

        eeg_data_buffer[sample_index] = sample & 0xFF;
        eeg_data_buffer[sample_index+1] = (sample >> 8) & 0xFF;
        sample_index += 2;

    }
    else if (sample_index >= BUFFER_SIZE) {
        // 采集满了，发送 BLE 任务请求
        ESP_EARLY_LOGI(TAG, "Buffer Full, Sending to BLE Task");

        // 发送 BLE 任务到队列
        if (xQueueSendFromISR(ble_queue, eeg_data_buffer, NULL) == pdTRUE) {
            sample_index = 0; // 重新开始采样
        }
    }

    task_running = false;
}

// BLE 任务
void ble_task(void *pvParameters) {
    uint8_t ble_buffer[BUFFER_SIZE];

    while (1) {
        if (xQueueReceive(ble_queue, &ble_buffer, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Sending BLE Data...");
            ble_set_ch1_value_large(ble_buffer, BUFFER_SIZE);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ble_cfg_net_init();
    configure_power_management(); // 低功耗设置
    i2c_master_init();  // 初始化 I2C

    while (ads1115_mode==0)
    {
        ESP_LOGI(TAG, "Mode  not setting... (0)");
        vTaskDelay(pdMS_TO_TICKS(1000));  // 延时1秒
    }
    ESP_LOGI(TAG, "Mode setted, channel number: %d", ads1115_mode);    //收到命令，输出channel
    int sample_time = 4000;
    if (ads1115_mode == 1)
    {
        ads1115_config(0xD0, 0xe3);   // 配置 continuous mode
        sample_time = 4000;           // 250Hz
    }else if (ads1115_mode == 2)
    {
        ads1115_config(0xC3, 0xe3);   // 配置 danci mode
        // ads1115_config(0xD0, 0xe3);   // 配置 continuous mode
        sample_time = 5000;           // 100 Hz
    }else if (ads1115_mode == 3)
    {
        ads1115_config(0xC3, 0xe3);   // 配置 danci mode
        sample_time = 3333;           // 100 Hz
    }else if (ads1115_mode == 4)
    {
        ads1115_config(0xC3, 0xe3);   // 配置 danci mode
        sample_time = 2500;           // 100 Hz
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // 9ms 等待第一次转换
    
    ble_queue = xQueueCreate(5, BUFFER_SIZE);
    // 启动 BLE 任务
    xTaskCreate(ble_task, "BLE Task", 8192, NULL, 5, NULL);


    // 创建定时器
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  // 在系统任务中执行
        .name = "periodic_timer"
    };


    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    // 启动定时器，每 4ms 触发一次
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, sample_time));  // 1000us = 1ms

    // xTaskCreatePinnedToCore(mytask1,"mytask",4096,NULL,3,NULL,1);

}

