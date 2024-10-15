#include "motor_control.h"

void brushed_motor_1_forward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void brushed_motor_1_backward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void brushed_motor_1_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
}

void brushed_motor_1_break()
{
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
}

// Motor 2 Control
void brushed_motor_2_forward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void brushed_motor_2_backward(float duty_cycle)
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void brushed_motor_2_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
}

void brushed_motor_2_break()
{
    mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
}

void move_forward(float duty_cycle)
{
    brushed_motor_1_forward(duty_cycle);
    brushed_motor_2_forward(duty_cycle);
}

void move_backward(float duty_cycle)
{
    brushed_motor_1_backward(duty_cycle);
    brushed_motor_2_backward(duty_cycle);
}

void move_stop()
{
    brushed_motor_1_stop();
    brushed_motor_2_stop();
}

void move_break()
{
    brushed_motor_1_break();
    brushed_motor_2_break();
}

void move_right(float duty_cycle)
{
    brushed_motor_1_forward(duty_cycle);
    brushed_motor_2_backward(duty_cycle);
}

void move_right_forward(float duty_cycle)
{
    brushed_motor_1_forward(duty_cycle);
    brushed_motor_2_break();
}

void move_right_backward(float duty_cycle)
{
    brushed_motor_1_break();
    brushed_motor_2_backward(duty_cycle);
}

void move_left(float duty_cycle)
{
    brushed_motor_1_backward(duty_cycle);
    brushed_motor_2_forward(duty_cycle);
}

void move_left_forward(float duty_cycle)
{
    brushed_motor_1_break();
    brushed_motor_2_forward(duty_cycle);
}

void move_left_backward(float duty_cycle)
{
    brushed_motor_1_backward(duty_cycle);
    brushed_motor_2_break();
}