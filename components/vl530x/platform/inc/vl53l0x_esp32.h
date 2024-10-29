#ifndef _VL53L0X_ESP32_H_
#define _VL53L0X_ESP32_H_

#include "vl53l0x_platform.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "stdio.h"   
#include "string.h"
#include "vl53l0x_api.h"
#include <esp_log.h>

#define    BYTES_PER_WORD        2
#define    BYTES_PER_DWORD       4

#define VL53L0X_DEFAULT_MODE 30000
#define VL53L0X_HIGH_ACCURACY 200000
#define VL53L0X_LONG_RANGE 33000
#define VL53L0X_HIGH_SPEED 20000

#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define I2C_TIMEOUT_MS 50
#define I2C_MASTER_FREQ_HZ  100000 // 100kHz

void i2c_master_init();
VL53L0X_Error VL53L0X_device_initialise(VL53L0X_Dev_t *pDevice, uint8_t new_i2c_addr, uint32_t RangeProfile);

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_Dev_t *pDevice);

VL53L0X_Error VL53L0X_SingleRanging(VL53L0X_Dev_t *pDevice, VL53L0X_RangingMeasurementData_t *MeasureData);
VL53L0X_Error VL53L0X_ContinuousRanging(VL53L0X_Dev_t *pDevice, uint16_t *MeasuredData, uint16_t RangeCount, uint16_t *validCount);

void init_xshuts(uint8_t *dist_sensors_xshuts, int cnt);
void set_all_xshut_states(uint8_t *dist_sensors_xshuts, bool running, int cnt);
void set_dist_sensors_consecutive_addrs(uint8_t *dist_sensors_addrs, uint8_t start_address, int cnt);
VL53L0X_Error init_dist_sensor(VL53L0X_Dev_t *pDevice, uint8_t XSHUT, uint8_t address, uint32_t range_profile);
void read_all_sensors(uint16_t *dist_sensor_data, VL53L0X_Dev_t *dist_sensors, int cnt);

#endif //_VL53L0X_RP2040_H_

