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
// #define SINGLE_RANGING
#define CONTINUOUS_RANGING
#define MAX_RANGE 500
#define MIN_RANGE 10
#define CENTERED_TRESHOLD 20

#define MOTOR_CONTROL_TIMER_PERIOD 15
#define SENSOR_READING_DELAY 10

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_1A              17
#define BDC_MCPWM_GPIO_1B              5
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
    FOUND,
    IDLE,
    RETREAT
} state_t;

// #define ACTIVE_ACCEL
#define ACTIVE_DEBUG
// #define SENSORS_DEBUG
// #define ACTIVE_DEBUG_DISTANCE_1
// #define ACTIVE_DEBUG_DISTANCE_2
// #define ACTIVE_DEBUG_DISTANCE_3
// #define ACTIVE_DEBUG_LINE
#define SAFE_MODE


// SemaphoreHandle_t i2cMutex;

esp_timer_handle_t motorControlTimer;
esp_timer_handle_t sensorReadingTimer;
esp_timer_handle_t retreatTimer;
// esp_timer_handle_t pid_timer;

static mma845x_sensor_t* accel;
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

// SENSOR ORDER(ROBOT VIEW): FR F FL R90 R45 L90 L45
VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {13, 12, 27, 33, 32, 25, 26};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x30, 0x32, 0x34, 0x36, 0x38, 0x40, 0x42};
uint16_t dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
SemaphoreHandle_t sensorsMutex;
SemaphoreHandle_t pidMutex;
SemaphoreHandle_t stateMutex;
int output;
int direction = 0;
state_t state = PATROL;
bool retreating = false;

double max_integral = 1000000;
double kp = 0.2;
double ki = 0;
double kd = 0;
state_t state_PID = PATROL;

int output_PID = 0;
double integral_PID = 0;
double error_prev_PID = 0, error_PID;
double derivative_PID = 0;

VL53L0X_RangingMeasurementData_t measurement;
int line_right, line_left, line_back;

int output_MOTOR_CALLBACK = 0;
state_t state_MOTOR_CALLBACK = PATROL;


//TODO: set lower timing budget
static int setup()
{
    // esp_timer_early_init();
    // esp_timer_init();

    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD_LED, 0);

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Create DC motors");
    #endif

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
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Enable motor 1");
    #endif
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Forward motor 1");
    #endif
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
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Enable motor 2");
    #endif
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Forward motor 2");
    #endif
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
    // sensorsMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();
    // stateMutex = xSemaphoreCreateMutex();
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

        #ifdef CONTINUOUS_RANGING
        status = VL53L0X_SetDeviceMode(&dist_sensors[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        status = VL53L0X_StartMeasurement(&dist_sensors[i]);
         

        #endif

        #ifdef SINGLE_RANGING
        status = VL53L0X_SetDeviceMode(&dist_sensors[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);
        #endif
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

int percent_to_duty_cycle(int percent)
{
    return (percent * BDC_MCPWM_DUTY_TICK_MAX) / 100;
}

void read_sensors()
{
    for(int i = 0; i < 7; i++)
    {
        // status = VL53L0X_StartMeasurement(&dist_sensors[i]);
        status = VL53L0X_GetRangingMeasurementData(&dist_sensors[i], &measurement);
        if(status != VL53L0X_ERROR_NONE)
        {
            dist_sensor_data[i] = MAX_RANGE;
            continue;
        }
        // status = VL53L0X_StopMeasurement(&dist_sensors[i]);

        dist_sensor_data[i] = measurement.RangeMilliMeter;
        // #ifdef SENSORS_DEBUG
        // #endif
        dist_sensor_data[i] = (dist_sensor_data[i] > MAX_RANGE) ? MAX_RANGE : dist_sensor_data[i];
        ESP_LOGI("sensor1_task", "Sensor %d: %d", i, dist_sensor_data[i]);
    }

    // #ifdef SENSORS_DEBUG
    ESP_LOGI("sensor1_task", "Sensor %d: %d", 6, dist_sensor_data[6]);
    // #endif
    



    //read line sensors
    #ifdef BLACK_FIELD
    line_right = !gpio_get_level(LINE_RIGHT);
    line_left = !gpio_get_level(LINE_LEFT);
    line_back = !gpio_get_level(LINE_BACK);
    #endif

    #ifndef BLACK_FIELD
    local_line_right = gpio_get_level(LINE_RIGHT);
    local_line_left = gpio_get_level(LINE_LEFT);
    local_line_back = gpio_get_level(LINE_BACK);
    #endif
}

void line_sensors_state()
{
    //stop robot if both line sensors detect line indefinitely
    state_PID = PATROL;
    if(line_right && line_left)
    {
        state_PID = IDLE;
        return;
    }

    if(line_right || line_left)
    {
        state_PID = RETREAT;
        return;
    }
}

void distance_sensors_state()
{
    if(state_PID == PATROL)
    {
        if(dist_sensor_data[1] < MAX_RANGE)
            state_PID = ATTACK;

        else if(error_PID > CENTERED_TRESHOLD)
            state_PID = FOUND;
    }
}

void sensors_to_PID_task(void *arg)
{
    while(1)
    {
        read_sensors();

        if(line_left && line_right)
        {
            state_PID = IDLE;
            output_PID = 0;
        }

        else if(line_right || line_left)
        {
            state_PID = RETREAT;
            output_PID = 85;
        }

        else if(dist_sensor_data[1] < MAX_RANGE)
        {
            state_PID = ATTACK;
            #ifdef SAFE_MODE
            output_PID = 60;
            #else
            output_PID = 100;
            #endif
        }

        else if(dist_sensor_data[0] < dist_sensor_data[2] || dist_sensor_data[5] < MAX_RANGE || dist_sensor_data[6] < MAX_RANGE)
        {
            state_PID = FOUND;
            output_PID = -70;
        }

        else if(dist_sensor_data[0] > dist_sensor_data[2] || dist_sensor_data[3] < MAX_RANGE || dist_sensor_data[4] < MAX_RANGE)
        {
            state_PID = FOUND;
            output_PID = 70;
        }

        else
        {
            state_PID = PATROL;
            output_PID = 60;
        }

        if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
        {
            output = output_PID;
            state = state_PID;
            xSemaphoreGive(pidMutex);
        }
    }
}

void retreat_timer_callback(TimerHandle_t xTimer)
{
    state_MOTOR_CALLBACK = PATROL;
    retreating = false;

}
void motor_controller_callback(TimerHandle_t xTimer)
{
    if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
    {
        output_MOTOR_CALLBACK = output;
        state_MOTOR_CALLBACK = state;
        xSemaphoreGive(pidMutex);
    }
    // else return;

    if(retreating)
    {
        ESP_LOGI("motor timer", "retreating");
        bdc_motor_reverse(motor1);
        bdc_motor_reverse(motor2);
        state_MOTOR_CALLBACK = RETREAT;
        output_MOTOR_CALLBACK = 100;
    }

    else
    {
        switch(state_MOTOR_CALLBACK)
        {
            case PATROL:
            case ATTACK:
                bdc_motor_forward(motor1);
                bdc_motor_forward(motor2);
                break;

            case FOUND:
                if(output_MOTOR_CALLBACK < 0)
                {
                    bdc_motor_forward(motor1);
                    bdc_motor_reverse(motor2);
                    output_MOTOR_CALLBACK = -output_MOTOR_CALLBACK;
                }

                else
                {
                    bdc_motor_reverse(motor1);
                    bdc_motor_forward(motor2);
                }
                break;

            case IDLE:
                bdc_motor_brake(motor1);
                bdc_motor_brake(motor2);
                break;

            case RETREAT:
                if (!retreating)
                {
                    retreating = true;
                    esp_timer_stop(retreatTimer);
                    esp_timer_start_once(retreatTimer, 100 * 1000);
                }
                break;
        }

    }
    
    bdc_motor_set_speed(motor1, percent_to_duty_cycle(output_MOTOR_CALLBACK));
    bdc_motor_set_speed(motor2, percent_to_duty_cycle(output_MOTOR_CALLBACK));

    ESP_LOGI("motor timer", "output: %d", output_MOTOR_CALLBACK);
    // ESP_LOGI("motor timer", "state: %d", state_MOTOR_CALLBACK);

    #ifdef ACTIVE_DEBUG
    // ESP_LOGI(MAIN_TAG, "timer running");
    #endif
}

void blink_led(int cnt, int delay)
{
    for(int i = 0; i < cnt; i++)
    {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

void app_main(void) 
{
    int setup_status = setup();
    if(setup_status == 1)
        blink_led(3, 150);
    else
        gpio_set_level(ONBOARD_LED, 1);

    mode_select();
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
    #endif
    
    //PATROL1 WITH PID
    if(mode == 1)
    {

        // xTaskCreatePinnedToCore(line_sensors_task, "Line sensors", 4096, NULL, 5, NULL, 1);
        // xTaskCreatePinnedToCore(sensors_read_task, "Group 1", 4096, NULL, 5, NULL, 0);
        xTaskCreatePinnedToCore(sensors_to_PID_task, "PID Task", 8192, NULL, 6, NULL, 1);
        
        esp_timer_create_args_t motorControlTimerArgs = {
            .callback = &motor_controller_callback,
            .name = "motor_control_timer"
        };
        esp_timer_create(&motorControlTimerArgs, &motorControlTimer);
        esp_timer_start_periodic(motorControlTimer, MOTOR_CONTROL_TIMER_PERIOD * 1000);

        esp_timer_create_args_t retreatTimerArgs = {
            .callback = &retreat_timer_callback,
            .name = "retreat_timer"
        };
        esp_timer_create(&retreatTimerArgs, &retreatTimer);
    }
}