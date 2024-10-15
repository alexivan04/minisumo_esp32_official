#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "vl53l0x_esp32.h"
#include "mma845x.h"
#include "motor_control.h"

#include <esp_wifi.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>
#include <math.h>
#include <freertos/queue.h>

#define ONBOARD_LED 2
#define DIST_SENSORS_CNT 7

#define LINE_RIGHT 15 //RIGHT
#define LINE_LEFT 14 //LEFT 
#define LINE_BACK 23 //BACK

#define MODE_BUTTON 35

#define M1_FIN 5
#define M1_BIN 17

#define M2_FIN 18
#define M2_BIN 19

#define BLACK_FIELD 1

#define LONG_PRESS_DELAY 1000
#define DOUBLE_PRESS_DELAY 3000
//#define WING_MOTOR - 6th pin on the right

#define MAX_RANGE 350
#define MIN_RANGE 25

// #define ACTIVE_ACCEL
// #define ACTIVE_DEBUG
// #define ACTIVE_DEBUG_DISTANCE_1
// #define ACTIVE_DEBUG_DISTANCE_2
// #define ACTIVE_DEBUG_DISTANCE_3
// #define ACTIVE_DEBUG_LINE

// SENSOR ORDER(ROBOT VIEW): FR F FL R90 R45 L90 L45
VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {13, 12, 27, 33, 32, 25, 26};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
uint16_t dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
QueueHandle_t dist_sensor_queues[DIST_SENSORS_CNT];
SemaphoreHandle_t i2cMutex;

static mma845x_sensor_t* accel;
int line_right, line_left, line_back;
int mode = 0;
int patrol_state = 0;
int digital_distace = 25;
TaskHandle_t sensor_task_handles[DIST_SENSORS_CNT];
static const char *MAIN_TAG = "main";
static const char *FRONT_GROUP_TAG = "front";
static const char *RIGHT_GROUP_TAG = "right";
static const char *LEFT_GROUP_TAG = "left";

typedef struct 
{
    float move_forward_duty_cycle;
    float move_backward_duty_cycle;
    float turn_duty_cycle;
    int move_delay;
    int turn_delay;
} PatrolParams;

typedef struct
{
    int sensor_id;
} SensorTaskParams;

typedef struct
{
    double max_integral;

    double kp;
    double ki;
    double kd;
} SensorRange_PID;

enum PatrolSates
{
    PATROL1 = 1,
    PATROL2 = 2
};

enum AttackStates
{
    ATTACK1 = 1,
    ATTACK2 = 2
};


//TODO: set lower timing budget
static int setup()
{
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD_LED, 0);

    //mode button
    gpio_reset_pin(MODE_BUTTON);
    gpio_set_direction(MODE_BUTTON, GPIO_MODE_INPUT);

    //motors PWM
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Setting up motors");
    #endif

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_FIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_BIN);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, M2_FIN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, M2_BIN);

    mcpwm_config_t pwm_config = {
        .frequency = 10000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    //i2c
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "I2C initializing");
    #endif
    i2c_master_init();

    #ifdef ACTIVE_DEBUG
    //DIST SENSORS
    ESP_LOGI(MAIN_TAG, "XHSUT resetting");
    #endif
    init_xshuts(dist_sensors_xshuts, DIST_SENSORS_CNT);
    set_all_xshut_states(dist_sensors_xshuts, true, DIST_SENSORS_CNT);
    vTaskDelay(25 / portTICK_PERIOD_MS);
    set_all_xshut_states(dist_sensors_xshuts, false, DIST_SENSORS_CNT);
    vTaskDelay(25 / portTICK_PERIOD_MS);

    //setup queues
    for(int i = 0; i < DIST_SENSORS_CNT; i++)
    {
        dist_sensor_queues[i] = xQueueCreate(15, sizeof(uint16_t));
        if(dist_sensor_queues[i] == NULL)
        {
            #ifdef ACTIVE_DEBUG
            ESP_LOGE(MAIN_TAG, "Queue %d creation failed", i);
            #endif
        }
    }

    //setup mutex
    i2cMutex = xSemaphoreCreateMutex();

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Dist sensor initializing");
    #endif
    int return_value = 1;
    for(int i = 0; i < DIST_SENSORS_CNT; i++)
    {
        status = init_dist_sensor(&dist_sensors[i], dist_sensors_xshuts[i], dist_sensors_addrs[i], VL53L0X_HIGH_SPEED);
        if(status != VL53L0X_ERROR_NONE)
        {
            #ifdef ACTIVE_DEBUG
            ESP_LOGE(MAIN_TAG, "Dist sensor %d initialization failed", i);
            ESP_LOGE(MAIN_TAG, "Error: %d", status);
            #endif
            return_value = 0;
        }
        status = VL53L0X_SetDeviceMode(&dist_sensors[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 
        status = VL53L0X_StartMeasurement(&dist_sensors[i]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    #ifdef ACTIVE_ACCEL
    //ACCEL
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Accel initializing");
    #endif
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
    {
        #ifdef ACTIVE_DEBUG
        ESP_LOGE(MAIN_TAG, "Could not initialize MMA845X accel. Error: %d", status);
        #endif
        return_value = 0;
    }
    #endif

    //LINE SENSOR
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Line sensors initializing");
    #endif
    gpio_reset_pin(LINE_RIGHT);
    gpio_set_direction(LINE_RIGHT, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_LEFT);
    gpio_set_direction(LINE_LEFT, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_BACK);
    gpio_set_direction(LINE_BACK, GPIO_MODE_INPUT);
    return return_value;

}
  
static void mode_select()
{
    int press_time = 0;
    int double_press = 0;
    mode = 0;

    //detect long press
    while(!double_press)
    {
        if(gpio_get_level(MODE_BUTTON) == 0)
        {
            //detect long press
            while(gpio_get_level(MODE_BUTTON) == 0)
            {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                press_time += 100;
            }

            if(press_time >= LONG_PRESS_DELAY)
            {
                mode++;
                press_time = 0;
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
                #endif

                //blink LED for mode selection
                for(int i = 0; i < mode; i++)
                {
                    gpio_set_level(ONBOARD_LED, 1);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    gpio_set_level(ONBOARD_LED, 0);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                }
            }

            else
            {
                //detect double press
                while(press_time < DOUBLE_PRESS_DELAY)
                {
                    if(gpio_get_level(MODE_BUTTON) == 0)
                    {
                        while(gpio_get_level(MODE_BUTTON) == 0)
                        {
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                            press_time += 100;
                        }
                        double_press = 1;
                        break;
                    }
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    press_time += 100;
                }
            }
        }
    }

    //TURN LED ON AFTER EXIT
    gpio_set_level(ONBOARD_LED, 1);

}

static void single_sensor_task_queue(void *arg)
{
    SensorTaskParams *params = (SensorTaskParams *)arg;
    int sensor_id = params->sensor_id;
    VL53L0X_RangingMeasurementData_t measurement;
    uint8_t dataReady = 0;
    uint16_t data;

    while(1)
    {
        //wait for max 20ms to take the mutex
        if(xSemaphoreTake(i2cMutex, 20 / portTICK_PERIOD_MS) == pdTRUE)
        {
            status = VL53L0X_GetMeasurementDataReady(&dist_sensors[sensor_id], &dataReady);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            if(dataReady && status == VL53L0X_ERROR_NONE)
            {
                status = VL53L0X_PerformSingleRangingMeasurement(&dist_sensors[sensor_id], &measurement);
                if(status == VL53L0X_ERROR_NONE)
                {
                    data = measurement.RangeMilliMeter;
                    data = (data < MIN_RANGE) ? MIN_RANGE : data;
                    data = (data > MAX_RANGE) ? MAX_RANGE : data;
                    xQueueSend(dist_sensor_queues[sensor_id], &data, 0);
                }
            }
            xSemaphoreGive(i2cMutex);
        }
    }
}

static int sensors_valid_readings()
{
    for(int i = 0; i < DIST_SENSORS_CNT; i++)
        if(dist_sensor_data[i] < MAX_RANGE && dist_sensor_data[i] > MIN_RANGE)
            return 1;
    return 0;
}

static void line_sensors_task(void* arg) 
{
    while (1) 
    {
        if(BLACK_FIELD)
        {
            line_right = !gpio_get_level(LINE_RIGHT);
            line_left = !gpio_get_level(LINE_LEFT);
            line_back = !gpio_get_level(LINE_BACK);
        }
        else
        {
            line_right = gpio_get_level(LINE_RIGHT);
            line_left = gpio_get_level(LINE_LEFT);
            line_back = gpio_get_level(LINE_BACK);
        }

        #ifdef ACTIVE_DEBUG_LINE
        ESP_LOGI(MAIN_TAG, "Line 1: %d, Line 2: %d, Line 3: %d", line_right, line_left, line_back);
        #endif
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//PATROL MODES
void mode1_patrol(void *pvParameters)
{
    PatrolParams *params = (PatrolParams *)pvParameters;  // Cast the parameters

    float move_forward_duty_cycle = params->move_forward_duty_cycle;
    float move_backward_duty_cycle = params->move_backward_duty_cycle;
    float turn_duty_cycle = params->turn_duty_cycle;
    int move_delay = params->move_delay;
    int turn_delay = params->turn_delay;

    while(1)
    {
        if(patrol_state == 1)
        {
            if(line_right && line_left && line_back)
            {
                move_break();
                vTaskDelay(5000 / portTICK_PERIOD_MS);                
            }

            else
            {
                move_forward(move_forward_duty_cycle);
                // patrol_state = 0;
            }

            if(line_right || line_left)
            {
                move_backward(move_backward_duty_cycle);
                vTaskDelay(move_delay / portTICK_PERIOD_MS);
                move_right_backward(turn_duty_cycle);
                vTaskDelay(turn_delay / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        
    }
}

void mode2_patrol_spin(void *pvParameters)
{
    PatrolParams *params = (PatrolParams *)pvParameters;  // Cast the parameters

    float move_forward_duty_cycle = params->move_forward_duty_cycle;
    float move_backward_duty_cycle = params->move_backward_duty_cycle;
    float turn_duty_cycle = params->turn_duty_cycle;
    int move_delay = params->move_delay;
    int turn_delay = params->turn_delay;

    int i = 0;

    while(1)
    {
        if(patrol_state)
        {
            if(i > 100000)
            {
                move_forward(move_forward_duty_cycle);
                vTaskDelay(move_delay / portTICK_PERIOD_MS);
                i = 0;
            }
            
            else if(line_right && line_left && line_back)
            {
                move_break();
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }

            else move_right(turn_duty_cycle);

            if(line_right || line_left)
            {
                move_backward(move_backward_duty_cycle);
                vTaskDelay(move_delay / portTICK_PERIOD_MS);
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
            i += 1;
        }
    }
}

//ATTACK MODES
void mode_turn_center_object_PID_queue(void *pvParameters)
{
    // ESP_LOGI(MAIN_TAG, "Checking distance sensors...");
    SensorRange_PID *range = (SensorRange_PID *)pvParameters;

    double max_integral = range->max_integral;
    double kp = range->kp;
    double ki = range->ki;
    double kd = range->kd;
    
    int output = 0;
    double integral = 0;
    double error_prev = 0, error;
    double derivative = 0;

    while(1)
    {
        xQueueReceive(dist_sensor_queues[0], &dist_sensor_data[0], 0);
        xQueueReceive(dist_sensor_queues[1], &dist_sensor_data[1], 0);
        xQueueReceive(dist_sensor_queues[2], &dist_sensor_data[2], 0);

        error = dist_sensor_data[0] - dist_sensor_data[2];
        integral += error;

        derivative = (error - error_prev) * 0.9 + derivative * 0.1; // Apply smoothing factor
        output = kp * error + ki * integral + kd * derivative;

        if(output > 0)
        {
            move_right(output);
        }

        else if(output < 0)
        {
            move_left(-output);
        }

        else move_stop();

        error_prev = error;
        integral = (integral > max_integral) ? max_integral : integral;
        integral = (integral < -max_integral) ? -max_integral : integral;

        #ifdef ACTIVE_DEBUG
        ESP_LOGI("pid", "error: %d", error);
        ESP_LOGI("pid", "output: %d", output);
        #endif

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}


void app_main(void) 
{
    int setup_status = setup();
    if(setup_status == 1)
    {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    else
    {
        gpio_set_level(ONBOARD_LED, 1);
    }

    mode_select();
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
    #endif

    xTaskCreatePinnedToCore(single_sensor_task_queue, "Sensor 0", 2048, (void *)&(SensorTaskParams){.sensor_id = 0}, 5, &sensor_task_handles[0], 1);
    xTaskCreatePinnedToCore(single_sensor_task_queue, "Sensor 1", 2048, (void *)&(SensorTaskParams){.sensor_id = 1}, 5, &sensor_task_handles[1], 1);
    xTaskCreatePinnedToCore(single_sensor_task_queue, "Sensor 2", 2048, (void *)&(SensorTaskParams){.sensor_id = 2}, 5, &sensor_task_handles[2], 1);
    xTaskCreate(line_sensors_task, "Line sensors", 2048, NULL, 2, NULL);

    //PATROL1 WITH PID
    if(mode == 1)
    {
        static SensorRange_PID range = {
            .max_integral = 1000000,

            .kp = 0.25,
            .ki = 0,
            .kd = 0,
        };

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        xTaskCreatePinnedToCore(mode_turn_center_object_PID_queue, "Mode 0", 2048, (void *)&range, 6, NULL, 0);
    }
}