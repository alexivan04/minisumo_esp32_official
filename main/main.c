#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <vl53l0x_esp32.h>
#include <mma845x.h>

#define ONBOARD_LED 2
#define DIST_SENSORS_CNT 3

#define TASK_STACK_DEPTH 2048

VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {27, 13, 12};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x31, 0x32, 0x33};
VL53L0X_RangingMeasurementData_t dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
static mma845x_sensor_t* accel;


void setup()
{
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD_LED, 0);

    i2c_master_init();

    //DIST SENSORS
    ESP_LOGI("main", "XHSUT resetting");
    init_xshuts(dist_sensors_xshuts, DIST_SENSORS_CNT);
    set_all_xshut_states(dist_sensors_xshuts, true, DIST_SENSORS_CNT);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    set_all_xshut_states(dist_sensors_xshuts, false, DIST_SENSORS_CNT);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ESP_LOGI("main", "Dist sensor initializing");
    for(int i = 0; i < DIST_SENSORS_CNT; i++)
    {
        status = init_dist_sensor(&dist_sensors[i], dist_sensors_xshuts[i], dist_sensors_addrs[i], VL53L0X_HIGH_SPEED);
        if(status != VL53L0X_ERROR_NONE)
        {
            ESP_LOGE("main", "Dist sensor %d initialization failed", i);
            ESP_LOGE("main", "Error: %d", status);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    //ACCEL
    ESP_LOGI("main", "Accel initializing");
    accel = mma845x_init_sensor(I2C_MASTER_NUM, MMA845X_I2C_ADDRESS_1);
    if (accel)
    {   
        // set polarity and type of INT signals if necessary
        mma845x_config_int_signals (accel, mma845x_high_active, mma845x_push_pull);

        // config HPF and enable it for accel output data
        mma845x_config_hpf (accel, 0, true);
        
        // set scale and mode
        mma845x_set_scale(accel, mma845x_scale_2_g);
        mma845x_set_mode (accel, mma845x_normal, mma845x_odr_50, true, false);
    }
    else
        ESP_LOGE("main", "Could not initialize MMA845X accel. Error: %d", status);

}
  
void app_main(void)
{

}
