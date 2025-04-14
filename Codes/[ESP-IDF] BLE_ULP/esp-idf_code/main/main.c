#include <stdio.h>
#include "esp_err.h"
#include "ble_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_log.h"         
#include "esp_pm.h"

#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "ulp_main.h"

#include <inttypes.h>
#include "sdkconfig.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include <string.h>
#include "soc/sens_reg.h"
#include "driver/gpio.h"

#include "esp_timer.h"

// #include "esp_task_wdt.h"

#define TAG "[ESP32-BLE-test2]"

// == ULP initial ==
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

const gpio_num_t GPIO_SCL = GPIO_NUM_32;
const gpio_num_t GPIO_SDA = GPIO_NUM_33;

// === I2C initial===
#define SAMPLE_SIZE 2  // 
#define SAMPLE_NUM 1400
#define BUFFER_SIZE (SAMPLE_NUM  * SAMPLE_SIZE)



uint8_t eeg_data_buffer[BUFFER_SIZE];  //
int sample_index = 0;
uint8_t ads1115_mode = 0;

#define TAG "[ESP32-BLE-test2]"
// QueueHandle_t ble_queue;  // 

static void init_ulp_program(void);

static void start_ulp_program(void);

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
    
    /* Set ULP wake up period to 1s */
    ulp_set_wakeup_period(0, 5*1000);

    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    rtc_gpio_init(GPIO_SCL);
    rtc_gpio_init(GPIO_SDA);
    rtc_gpio_set_direction(GPIO_SCL, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(GPIO_SDA, RTC_GPIO_MODE_INPUT_ONLY);
}


static void start_ulp_program() {
    ulp_buffer_index = 0;           // reset index
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}


void ulp_copy_data(void) {
    for (size_t i = 0; i < SAMPLE_NUM; i++) {
        int16_t sample = (uint16_t)(&ulp_data_buffer)[i];
        eeg_data_buffer[sample_index] = sample & 0xFF;
        eeg_data_buffer[sample_index+1] = (sample >> 8) & 0xFF;
        sample_index += 2;
    }
}

void ulp_send_data(void) {
    // ESP_LOGI(TAG, "index: %d", sample_index);
    // for (int i = 0; i < sample_index; i += 2) {
    //             int16_t value = (eeg_data_buffer[i + 1] << 8) | eeg_data_buffer[i];
    //             ESP_LOGI(TAG, "[%d] %d", i / 2, value);
    //         }
    // sample_index = 0;
    ESP_ERROR_CHECK(nvs_flash_init());
    ble_cfg_net_init();
    int time_limit = 0;
    while (ads1115_mode==0)
    {
        ESP_LOGI(TAG, "Mode  not setting... (0)");
        vTaskDelay(pdMS_TO_TICKS(100));  // 0.1s
        time_limit += 0.1;
        if (time_limit >= 8)
        {
            esp_deep_sleep_start();
        }
    }
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 1s
    ESP_LOGI(TAG, "Sending BLE Data...");
    ble_set_ch1_value_large(eeg_data_buffer, BUFFER_SIZE);
}


void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Initialization\n");
        init_ulp_program();
        start_ulp_program();
    } else {
        int64_t start_time = esp_timer_get_time(); // time consuming
        printf("Woke up from ULP!\n");
        // printf("Hello World. ULP counter: %lu\n",  ulp_buffer_index& UINT16_MAX);
        ulp_copy_data();
        start_ulp_program();
        ulp_send_data();
        int64_t end_time = esp_timer_get_time();   // end
        printf("run time: %lld us\n", (end_time - start_time));
        
    }
    printf("Entering deep sleep\n\n");
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    /* RTC peripheral power domain needs to be kept on to keep SAR ADC related configs during sleep */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
    esp_deep_sleep_start();
}

    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);



// void ble_task(void *pvParameters) {
//     uint8_t ble_buffer[BUFFER_SIZE];

//     while (1) {
//         if (xQueueReceive(ble_queue, &ble_buffer, portMAX_DELAY) == pdTRUE) {
//             ESP_LOGI(TAG, "Sending BLE Data...");
//             ble_set_ch1_value_large(ble_buffer, BUFFER_SIZE);
//         }
//     }
// }