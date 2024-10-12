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
#include "esp_timer.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "math.h"

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

// SENSOR ORDER(ROBOT VIEW): FR F FL R90 R45
VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {13, 12, 27, 33, 32, 26, 25};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
VL53L0X_RangingMeasurementData_t dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
static mma845x_sensor_t* accel;
int line_right, line_left, line_back;
int mode = 0;
int patrol_state = 0;

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
    int min_range;
    int max_range;

    double kp;
    double ki;
    double kd;
} SensorRange_PID;

typedef struct
{
    int min_range;
    int max_range;

    double lambda;
    double alpha;
    double K1;
    double K2;
    double epsilon;
} SensorRange_SMC;

static void setup()
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
    ESP_LOGI(MAIN_TAG, "Setting up motors");
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
    ESP_LOGI(MAIN_TAG, "I2C initializing");
    i2c_master_init();

    //DIST SENSORS
    ESP_LOGI(MAIN_TAG, "XHSUT resetting");
    init_xshuts(dist_sensors_xshuts, DIST_SENSORS_CNT);
    set_all_xshut_states(dist_sensors_xshuts, true, DIST_SENSORS_CNT);
    vTaskDelay(25 / portTICK_PERIOD_MS);
    set_all_xshut_states(dist_sensors_xshuts, false, DIST_SENSORS_CNT);
    vTaskDelay(25 / portTICK_PERIOD_MS);

    ESP_LOGI(MAIN_TAG, "Dist sensor initializing");
    for(int i = 0; i < DIST_SENSORS_CNT; i++)
    {
        status = init_dist_sensor(&dist_sensors[i], dist_sensors_xshuts[i], dist_sensors_addrs[i], VL53L0X_HIGH_SPEED);
        if(status != VL53L0X_ERROR_NONE)
        {
            ESP_LOGE(MAIN_TAG, "Dist sensor %d initialization failed", i);
            ESP_LOGE(MAIN_TAG, "Error: %d", status);
                gpio_set_level(ONBOARD_LED, 1);
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);
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

    //LINE SENSOR
    ESP_LOGI(MAIN_TAG, "Line sensors initializing");
    gpio_reset_pin(LINE_RIGHT);
    gpio_set_direction(LINE_RIGHT, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_LEFT);
    gpio_set_direction(LINE_LEFT, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_BACK);
    gpio_set_direction(LINE_BACK, GPIO_MODE_INPUT);
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
                ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);

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

// Task to read the first group of 3 sensors
static void sensor_group_1_task(void* arg) 
{
    uint8_t measured_data_ready = 0;
    while (1) 
    {
        for (int i = 0; i < 3; i++) 
        {
            VL53L0X_PerformSingleRangingMeasurement(&dist_sensors[i], &dist_sensor_data[i]);
            // while(!measured_data_ready)
            // {
            //     VL53L0X_GetMeasurementDataReady(&dist_sensors[i], &measured_data_ready);
            //     vTaskDelay(5 / portTICK_PERIOD_MS);
            // }
            ESP_LOGI(FRONT_GROUP_TAG, "Sensor %d: %d", i, dist_sensor_data[i].RangeMilliMeter);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Task to read the second group of 2 sensors
static void sensor_group_2_task(void* arg) 
{
    uint8_t measured_data_ready = 0;
    while (1) 
    {
        for (int i = 3; i < 5; i++) 
        {
            VL53L0X_PerformSingleRangingMeasurement(&dist_sensors[i], &dist_sensor_data[i]);
            // while(!measured_data_ready)
            // {
            //     VL53L0X_GetMeasurementDataReady(&dist_sensors[i], &measured_data_ready);
            //     vTaskDelay(5 / portTICK_PERIOD_MS);
            // }
            ESP_LOGI(RIGHT_GROUP_TAG, "Sensor %d: %d", i, dist_sensor_data[i].RangeMilliMeter);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Task to read the third group of 2 sensors
static void sensor_group_3_task(void* arg) 
{
    uint8_t measured_data_ready = 0;
    while (1) 
    {
        for (int i = 5; i < 7; i++) 
        {
            VL53L0X_PerformSingleRangingMeasurement(&dist_sensors[i], &dist_sensor_data[i]);
            // while(!measured_data_ready)
            // {
            //     VL53L0X_GetMeasurementDataReady(&dist_sensors[i], &measured_data_ready);
            //     vTaskDelay(5 / portTICK_PERIOD_MS);
            // }
            ESP_LOGI(LEFT_GROUP_TAG, "Sensor %d: %d", i, dist_sensor_data[i].RangeMilliMeter);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
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
        // ESP_LOGI(MAIN_TAG, "Line 1: %d, Line 2: %d, Line 3: %d", line_right, line_left, line_back);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Motor 1 Control
static void brushed_motor_1_forward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

static void brushed_motor_1_backward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

static void brushed_motor_1_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
}

static void brushed_motor_1_break()
{
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
}

// Motor 2 Control
static void brushed_motor_2_forward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

static void brushed_motor_2_backward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

static void brushed_motor_2_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
}

static void brushed_motor_2_break()
{
    mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
}

static void move_forward(float duty_cycle)
{
    brushed_motor_1_forward(duty_cycle);
    brushed_motor_2_forward(duty_cycle);
}

static void move_backward(float duty_cycle)
{
    brushed_motor_1_backward(duty_cycle);
    brushed_motor_2_backward(duty_cycle);
}

static void move_stop()
{
    brushed_motor_1_stop();
    brushed_motor_2_stop();
}

static void move_break()
{
    brushed_motor_1_break();
    brushed_motor_2_break();
}

static void move_right(float duty_cycle)
{
    brushed_motor_1_forward(duty_cycle);
    brushed_motor_2_backward(duty_cycle);
}

static void move_right_forward(float duty_cycle)
{
    brushed_motor_1_forward(duty_cycle);
    brushed_motor_2_break();
}

static void move_right_backward(float duty_cycle)
{
    brushed_motor_1_break();
    brushed_motor_2_backward(duty_cycle);
}

static void move_left(float duty_cycle)
{
    brushed_motor_1_backward(duty_cycle);
    brushed_motor_2_forward(duty_cycle);
}

static void move_left_forward(float duty_cycle)
{
    brushed_motor_1_break();
    brushed_motor_2_forward(duty_cycle);
}

static void move_left_backward(float duty_cycle)
{
    brushed_motor_1_backward(duty_cycle);
    brushed_motor_2_break();
}

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
        if(patrol_state)
        {
            if(line_right && line_left && line_back)
            {
                move_break();
                vTaskDelay(5000 / portTICK_PERIOD_MS);                
            }

            else move_forward(move_forward_duty_cycle);
            // vTaskDelay( / portTICK_PERIOD_MS);

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


//maybe implement filtering? sounds overkill
void mode_turn_center_object_SMC(void *pvParameters)
{
    // ESP_LOGI(MAIN_TAG, "Checking distance sensors...");
    SensorRange_SMC *range = (SensorRange_SMC *)pvParameters;
    int min_range = range->min_range;
    int max_range = range->max_range;

    double lambda = range->lambda;
    double alpha = range->alpha;
    double K1 = range->K1;
    double K2 = range->K2;
    double epsilon = range->epsilon;
    
    double error, error_dot, error_ddot, sliding_surface, sliding_surface_dot;
    double output = 0;
    double error_prev = 0, error_dot_prev;

    while(1)
    {
        // error will be of angle (0 meaning centered), calculated from the LF and RF sensors
        error = dist_sensor_data[0].RangeMilliMeter - dist_sensor_data[2].RangeMilliMeter;

        if(dist_sensor_data[1].RangeMilliMeter > max_range || dist_sensor_data[1].RangeMilliMeter < min_range)
        {
            error = 0;
            patrol_state = 1;
        }
        else patrol_state = 0;

        if(!patrol_state)
        {
            error_dot = error - error_prev;           //first order derivative
            error_ddot = error_dot - error_dot_prev;  //second order derivative

            sliding_surface = lambda * error + error_dot + alpha * error_ddot;  //first order sliding surfaces
            sliding_surface_dot = error_dot + alpha * error_dot;                //second order sliding surface

            // apply control law: 2-SMC
            if (fabs(sliding_surface) > epsilon) 
            {
                // higher-order switching control
                if (fabs(sliding_surface_dot) > epsilon) 
                    output = K1 * ((sliding_surface > 0) ? 1 : -1) + K2 * ((sliding_surface_dot > 0) ? 1 : -1);
                else 
                    output = K1 * (sliding_surface / epsilon);
            } 

            else 
            {
                output = K1 * (sliding_surface / epsilon);
            }

            // ESP_LOGI(MAIN_TAG, "Error: %f, Output: %f", error, output);
            if(output > 0)
            {
                move_right(output);
            }

            else if(output < 0)
            {
                move_left(-output);
            }

            else move_forward(100);

            error_dot_prev = error_dot;
            error_prev = error;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void mode_turn_center_object_PID(void *pvParameters)
{
    // ESP_LOGI(MAIN_TAG, "Checking distance sensors...");
    SensorRange_PID *range = (SensorRange_PID *)pvParameters;
    int min_range = range->min_range;
    int max_range = range->max_range;

    double kp = range->kp;
    double ki = range->ki;
    double kd = range->kd;
    
    double output = 0;
    int derivative, error_prev = 0, error, integral = 0, integral_prev = 0;
    int bias = 0;

    while(1)
    {
        // error will be of angle (0 meaning centered), calculated from the LF and RF sensors
        error = dist_sensor_data[0].RangeMilliMeter - dist_sensor_data[2].RangeMilliMeter;
        if(dist_sensor_data[1].RangeMilliMeter > max_range || dist_sensor_data[1].RangeMilliMeter < min_range)
        {
            error = 0;
            patrol_state = 1;
        }
        else patrol_state = 0;

        // ESP_LOGI(MAIN_TAG, "Error: %d", error);
        integral += error;
        derivative = error - error_prev;
        output = kp * error + ki * integral + kd * derivative ;

        // ESP_LOGI(MAIN_TAG, "Error: %d, Output: %f", error, output);

        if(output > 0)
        {
            move_right(output);
        }

        else if(output < 0)
        {
            move_left(-output);
        }

        else patrol_state = 1;

        error_prev = error;
        integral_prev = integral;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void) 
{
    setup();
    mode_select();
    ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);

    xTaskCreatePinnedToCore(sensor_group_1_task, "Sensor Group 1", 2048, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(sensor_group_2_task, "Sensor Group 2", 2048, NULL, 2, NULL, 1);
    // xTaskCreatePinnedToCore(sensor_group_3_task, "Sensor Group 3", 2048, NULL, 2, NULL, 1);
    xTaskCreate(line_sensors_task, "Line Sensors", 2048, NULL, 2, NULL);

    //PATROL1 WITH PID
    if(mode == 1)
    {
        SensorRange_PID range = {
            .min_range = 25,
            .max_range = 250,

            .kp = 0.2,
            .ki = 0,
            .kd = 0,
        };

        PatrolParams patrol_params = {
            .move_forward_duty_cycle = 40,
            .move_backward_duty_cycle = 80,
            .turn_duty_cycle = 80,
            .move_delay = 250,
            .turn_delay = 100
        };

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        xTaskCreatePinnedToCore(mode1_patrol, "Mode 2", 2048, (void *)&patrol_params, 3, NULL, 0);
        xTaskCreatePinnedToCore(mode_turn_center_object_PID, "Mode 0", 2048, (void *)&range, 4, NULL, 0);
    }

    //PATROL2 WITH SMC
    if(mode == 2)
    {
        SensorRange_SMC range = {
            .min_range = 25,
            .max_range = 250,

            .lambda = 1,
            .alpha = 0.1,
            .K1 = 10,
            .K2 = 50,
            
            .epsilon = 0.1,
        };

        PatrolParams patrol_params = {
            .move_forward_duty_cycle = 40,
            .move_backward_duty_cycle = 80,
            .turn_duty_cycle = 80,
            .move_delay = 250,
            .turn_delay = 100
        };

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        xTaskCreatePinnedToCore(mode1_patrol, "Mode 2", 2048, (void *)&patrol_params, 3, NULL, 0); 
        xTaskCreatePinnedToCore(mode_turn_center_object_SMC, "Mode 1", 2048, (void *)&range, 4, NULL, 0); 
    }
}