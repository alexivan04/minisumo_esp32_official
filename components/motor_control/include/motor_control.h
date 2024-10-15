#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

void brushed_motor_1_forward(float duty_cycle);
void brushed_motor_1_backward(float duty_cycle);
void brushed_motor_1_stop();
void brushed_motor_1_break();
void brushed_motor_2_forward(float duty_cycle);
void brushed_motor_2_backward(float duty_cycle);
void brushed_motor_2_stop();
void brushed_motor_2_break();

void move_forward(float duty_cycle);
void move_backward(float duty_cycle);
void move_stop();
void move_break();
void move_right(float duty_cycle);
void move_right_forward(float duty_cycle);
void move_right_backward(float duty_cycle);
void move_left(float duty_cycle);
void move_left_forward(float duty_cycle);
void move_left_backward(float duty_cycle);