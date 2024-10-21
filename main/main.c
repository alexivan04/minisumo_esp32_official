#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <stdint.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "vl53l0x_esp32.h"
#include "mma845x.h"
#include "motor_control.h"
#include "bdc_motor.h"

#include <esp_wifi.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include <math.h>
#include <freertos/queue.h>
#include <string.h>

// #include "soc/rtc_wdt.h"

#define ONBOARD_LED 2
#define DIST_SENSORS_CNT 7

#define LINE_RIGHT 15 //RIGHT
#define LINE_LEFT 14 //LEFT 
#define LINE_BACK 23 //BACK

#define MODE_BUTTON 35

#define BLACK_FIELD 1

#define LONG_PRESS_DELAY 1000
#define DOUBLE_PRESS_DELAY 3000
//#define WING_MOTOR - 6th pin on the right

#define MAX_RANGE 350
#define MIN_RANGE 25

#define MOTOR_CONTROL_TIMER_PERIOD 10
#define SENSOR_READING_DELAY 10

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             5000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_1A              5
#define BDC_MCPWM_GPIO_1B              17
#define BDC_MCPWM_GPIO_2A              18
#define BDC_MCPWM_GPIO_2B              19
bdc_motor_handle_t motor1 = NULL, motor2 = NULL;

typedef enum
{
    FORWARD,
    BACKWARD
} direction_t;

typedef enum
{
    PATROL,
    ATTACK,
    IDLE,
    RETREAT
} state_t;

// #define ACTIVE_ACCEL
#define ACTIVE_DEBUG
// #define ACTIVE_DEBUG_DISTANCE_1
// #define ACTIVE_DEBUG_DISTANCE_2
// #define ACTIVE_DEBUG_DISTANCE_3
// #define ACTIVE_DEBUG_LINE

// SENSOR ORDER(ROBOT VIEW): FR F FL R90 R45 L90 L45
VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {13, 12, 27, 33, 32, 25, 26};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
int dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
SemaphoreHandle_t sensorsMutex;
SemaphoreHandle_t pidMutex;
SemaphoreHandle_t stateMutex;
int pid_output;
int direction = 0;
state_t state = IDLE;


// SemaphoreHandle_t i2cMutex;

TimerHandle_t motorControlTimer;

static mma845x_sensor_t* accel;
int line_right, line_left, line_back;
int mode = 0;
int patrol_state = 0;
int digital_distace = 25;
TaskHandle_t sensor_task_handles[DIST_SENSORS_CNT];
static const char *MAIN_TAG = "main";

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
//FOR TESTING


//TODO: set lower timing budget
static int setup()
{
    // rtc_wdt_protect_off();
    // rtc_wdt_disable();
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD_LED, 0);

    ESP_LOGI(MAIN_TAG, "Create DC motors");
    bdc_motor_config_t motor_config1 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_1A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_1B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config1 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config1, &mcpwm_config1, &motor1));
    ESP_LOGI(MAIN_TAG, "Enable motor 1");
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    ESP_LOGI(MAIN_TAG, "Forward motor 1");
    ESP_ERROR_CHECK(bdc_motor_forward(motor1));

    bdc_motor_config_t motor_config2 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_2A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_2B,
    };

    bdc_motor_mcpwm_config_t mcpwm_config2 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config2, &mcpwm_config2, &motor2));
    ESP_LOGI(MAIN_TAG, "Enable motor 2");
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));
    ESP_LOGI(MAIN_TAG, "Forward motor 2");
    ESP_ERROR_CHECK(bdc_motor_forward(motor2));

    //mode button
    gpio_reset_pin(MODE_BUTTON);
    gpio_set_direction(MODE_BUTTON, GPIO_MODE_INPUT);

    //motors PWM
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Setting up motors");
    #endif

    //i2c

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
    // pid_data_queue = xQueueCreate(10, sizeof(int));

    //setup mutex for i2c bus
    sensorsMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();
    // i2cMutex = xSemaphoreCreateMutex();


    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "I2C initializing");
    #endif
    i2c_master_init();

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Dist sensor initializing");
    #endif
    int return_value = 1;
    for(int i = 0; i < 7; i++)
    {
        // Add the sensor device to the I2C bus
        status = init_dist_sensor(&dist_sensors[i], dist_sensors_xshuts[i], dist_sensors_addrs[i], VL53L0X_HIGH_SPEED);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(status != VL53L0X_ERROR_NONE)
        {
            #ifdef ACTIVE_DEBUG
            ESP_LOGE(MAIN_TAG, "Dist sensor %d initialization failed", i);
            ESP_LOGE(MAIN_TAG, "Error: %d", status);
            #endif
            return_value = 0;
        }
        status = VL53L0X_SetDeviceMode(&dist_sensors[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 
        vTaskDelay(20 / portTICK_PERIOD_MS);
        status = VL53L0X_StartMeasurement(&dist_sensors[i]);
        vTaskDelay(20 / portTICK_PERIOD_MS);
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

static void sensors_read_task(void *arg)
{
    VL53L0X_RangingMeasurementData_t measurement;
    int data[7] = {0};
    int local_line_right = 0, local_line_left = 0, local_line_back = 0;

    while(1)
    {
        for(int i = 0; i < 7; i++)
        {
            status = VL53L0X_GetRangingMeasurementData(&dist_sensors[i], &measurement);
            if(status == VL53L0X_ERROR_NONE)
            {
                data[i] = measurement.RangeMilliMeter;
                // ESP_LOGI("sensor1_task", "Sensor %d: %d", i, data[i]);
                data[i] = (data[i] < MIN_RANGE) ? MIN_RANGE : data[i];
                data[i] = (data[i] > MAX_RANGE) ? MAX_RANGE : data[i];
            }
        }

        #ifdef BLACK_FIELD
        local_line_right = !gpio_get_level(LINE_RIGHT);
        local_line_left = !gpio_get_level(LINE_LEFT);
        local_line_back = !gpio_get_level(LINE_BACK);
        #endif

        #ifndef BLACK_FIELD
        local_line_right = gpio_get_level(LINE_RIGHT);
        local_line_left = gpio_get_level(LINE_LEFT);
        local_line_back = gpio_get_level(LINE_BACK);
        #endif

        if(xSemaphoreTake(sensorsMutex, 10 / portMAX_DELAY) == pdTRUE)
        {
            memcpy(dist_sensor_data, data, 7 * sizeof(int)); 
            line_right = local_line_right;
            line_left = local_line_left;
            line_back = local_line_back;
            xSemaphoreGive(sensorsMutex);
        }

        #ifdef ACTIVE_DEBUG_LINE
        ESP_LOGI(MAIN_TAG, "Line 1: %d, Line 2: %d, Line 3: %d", line_right, line_left, line_back);
        #endif

        vTaskDelay(10 / portTICK_PERIOD_MS);
        
    }
}

static void line_sensors_task(void* arg) 
{
    int local_line_right = 0, local_line_left = 0, local_line_back = 0;
    state_t local_state = IDLE;
    while (1) 
    {
        if(xSemaphoreTake(sensorsMutex, 10 / portMAX_DELAY) == pdTRUE)
        {
            local_line_right = line_right;
            local_line_left = line_left;
            local_line_back = line_back;
            xSemaphoreGive(sensorsMutex);
        }

        if(xSemaphoreTake(stateMutex, 10 / portMAX_DELAY) == pdTRUE)
        {
            local_state = state;
            xSemaphoreGive(stateMutex);
        }

        if(local_line_right && local_line_left && local_line_back)
            local_state = IDLE;

        else if(local_line_right || local_line_left)
            local_state = RETREAT;

        else if(local_state != ATTACK && local_state != RETREAT)
            local_state = PATROL;

        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            state = local_state;
            xSemaphoreGive(stateMutex);
        }
    }
}

//ATTACK MODES
void mode_turn_center_object_PID(void *pvParameters)
{
    SensorRange_PID *range = (SensorRange_PID *)pvParameters;
    int local_sensor_data[7] = {0};
    int local_line_right = 0, local_line_left = 0, local_line_back = 0;
    state_t local_state = IDLE;

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
        if(xSemaphoreTake(sensorsMutex,  10 / portMAX_DELAY) == pdTRUE)  // Wait for sensor data
        {
            memcpy(local_sensor_data, dist_sensor_data, 7 * sizeof(int));
            xSemaphoreGive(sensorsMutex);
        }

        if(xSemaphoreTake(stateMutex, 10 / portMAX_DELAY) == pdTRUE)
        {
            local_state = state;
            xSemaphoreGive(stateMutex);
        }

        // PID control logic
        error = local_sensor_data[1] - local_sensor_data[2];
        if(error != 0)
            local_state = ATTACK;

        else 
            local_state = PATROL;

        integral += error;
        derivative = (error - error_prev) * 0.9 + derivative * 0.1;  // Apply smoothing factor
        output = (int)(kp * error + ki * integral + kd * derivative);

        error_prev = error;
        integral = (integral > max_integral) ? max_integral : integral;
        integral = (integral < -max_integral) ? -max_integral : integral;

        if(output > BDC_MCPWM_DUTY_TICK_MAX)
            output = BDC_MCPWM_DUTY_TICK_MAX;

        else if(output < -BDC_MCPWM_DUTY_TICK_MAX)
            output = -BDC_MCPWM_DUTY_TICK_MAX;

        if(xSemaphoreTake(pidMutex, 10 / portMAX_DELAY) == pdTRUE)
        {
            pid_output = output;
            xSemaphoreGive(pidMutex);
        }

        if(xSemaphoreTake(stateMutex, 10 / portMAX_DELAY) == pdTRUE)
        {
            state = local_state;
            xSemaphoreGive(stateMutex);
        }

        // if(xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
        // {
        //     memcpy(&pid_output, &output, sizeof(int));
        //     set_motor_speed(output, -output);
        // }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void motor_controller_callback(TimerHandle_t xTimer)
{
    int output = 0;
    state_t local_state = IDLE, prev_state = IDLE;
    if(xSemaphoreTake(pidMutex, 10 / portMAX_DELAY) == pdTRUE)
    {
        output = pid_output;
        xSemaphoreGive(pidMutex);
    }

    prev_state = local_state;
    if(xSemaphoreTake(stateMutex, 10 / portMAX_DELAY) == pdTRUE)
    {
        local_state = state;
        xSemaphoreGive(stateMutex);
    }

    switch(local_state)
    {
        case PATROL:
            bdc_motor_forward(motor1);
            bdc_motor_forward(motor2);
            bdc_motor_set_speed(motor1, 0.35 * BDC_MCPWM_DUTY_TICK_MAX);
            bdc_motor_set_speed(motor2, 0.35 * BDC_MCPWM_DUTY_TICK_MAX);
            break;

        case ATTACK:
            bdc_motor_forward(motor1);
            bdc_motor_forward(motor2);
            bdc_motor_set_speed(motor1, output);
            bdc_motor_set_speed(motor2, output);
            break;

        case RETREAT:
            bdc_motor_reverse(motor1);
            bdc_motor_reverse(motor2);
            bdc_motor_set_speed(motor1, 0.70 * BDC_MCPWM_DUTY_TICK_MAX);
            bdc_motor_set_speed(motor2, 0.70 * BDC_MCPWM_DUTY_TICK_MAX);
            break;

        case IDLE:
            bdc_motor_brake(motor1);
            bdc_motor_brake(motor2);
            break;
        
        default:
            break;
    }
    // ESP_LOGI(MAIN_TAG, "Output: %d", output);
    bdc_motor_set_speed(motor1, output);
    bdc_motor_set_speed(motor2, output);
}

// void motor_controller_callback(TimerHandle_t xTimer)
// {
//     int output = 0;
//     if(xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
//     {
//         output = pid_output;
//         xSemaphoreGive(pidMutex);
//     }
//     if(output < 0)
//         set_motor_a_speed_step(-output, DIRECTION_REVERSE, 5);

//     else
//         set_motor_a_speed_step(output, DIRECTION_FORWARD, 5);
// }


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
    

    //PATROL1 WITH PID
    if(mode == 1)
    {
        static SensorRange_PID range = {
            .max_integral = 1000000,

            .kp = 0.3,
            .ki = 0,
            .kd = 0,
        };

        xTaskCreatePinnedToCore(line_sensors_task, "Line sensors", 4096, NULL, 5, NULL, 1);
        xTaskCreatePinnedToCore(sensors_read_task, "Group 1", 4096, NULL, 5, NULL, 0);
        xTaskCreatePinnedToCore(mode_turn_center_object_PID, "Mode 0", 4096, (void *)&range, 6, NULL, 1);

        motorControlTimer = xTimerCreate("MotorAControlTimer", 
                                    pdMS_TO_TICKS(20), 
                                    pdTRUE, // Auto-reload
                                    (void *)0, 
                                    motor_controller_callback);

    
    
        // Start the timer
        if (motorControlTimer != NULL)
        {
            if (xTimerStart(motorControlTimer, 0) != pdPASS)
            {
                #ifdef ACTIVE_DEBUG
                ESP_LOGE(MAIN_TAG, "Failed to start Motor control timer");
                #endif
            }
        }
    }
}