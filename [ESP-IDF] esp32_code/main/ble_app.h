#ifndef _BLE_APP_H_
#define _BLE_APP_H_
#include "esp_err.h"

/**
 * 初始化并启动蓝牙BLE
 * @param 无
 * @return 是否成功
 */
esp_err_t ble_cfg_net_init(void);

// /**
//  * 设置特征1的值
//  * @param value 值
//  * @return 无
//  */
// void ble_set_ch1_value(uint16_t value);

/**
 * 设置特征1的值
 * @param data 8位地址
 * @param total_length 16位长度
 * @return 无
 */
void ble_set_ch1_value_large(uint8_t *data, uint16_t total_length);


/**
 * 设置特征2的值
 * @param value 值
 * @return 无
 */
void ble_set_ch2_value(uint16_t value);

#endif
