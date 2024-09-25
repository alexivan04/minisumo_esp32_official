#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <vl53l0x_esp32.h>
#include <mma845x.h>

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_mac.h"


#define ONBOARD_LED 2
#define DIST_SENSORS_CNT 3

#define TASK_STACK_DEPTH 2048

#define WIFI_SSID      "mama"
#define WIFI_PASS      "e8tj_Rwzmzbee"
#define WIFI_CHANNEL   1
#define WIFI_MAX_CONN  2

#define PORT 999

//#define WING_MOTOR - 6th pin on the righ

VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {27, 13, 12};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x31, 0x32, 0x33};
VL53L0X_RangingMeasurementData_t dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
static mma845x_sensor_t* accel;

static const char *WIFI_TAG = "wifi";
static const char *MAIN_TAG = "main";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(WIFI_TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(WIFI_TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}


void setup()
{
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD_LED, 0);

    i2c_master_init();

    //DIST SENSORS
    ESP_LOGI(MAIN_TAG, "XHSUT resetting");
    init_xshuts(dist_sensors_xshuts, DIST_SENSORS_CNT);
    set_all_xshut_states(dist_sensors_xshuts, true, DIST_SENSORS_CNT);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    set_all_xshut_states(dist_sensors_xshuts, false, DIST_SENSORS_CNT);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ESP_LOGI(MAIN_TAG, "Dist sensor initializing");
    for(int i = 0; i < DIST_SENSORS_CNT; i++)
    {
        status = init_dist_sensor(&dist_sensors[i], dist_sensors_xshuts[i], dist_sensors_addrs[i], VL53L0X_HIGH_SPEED);
        if(status != VL53L0X_ERROR_NONE)
        {
            ESP_LOGE(MAIN_TAG, "Dist sensor %d initialization failed", i);
            ESP_LOGE(MAIN_TAG, "Error: %d", status);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    //ACCEL
    ESP_LOGI(MAIN_TAG, "Accel initializing");
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
        ESP_LOGE(MAIN_TAG, "Could not initialize MMA845X accel. Error: %d", status);

    
    //WIFI
    ESP_LOGI(MAIN_TAG, "Wifi initializing");

}
  
void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
}
