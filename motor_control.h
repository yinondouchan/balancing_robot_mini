#ifndef MOVE_MOTOR_H
#define MOVE_MOTOR_H

#include <stdint.h>
#include <Arduino.h>

// driver pins for left motor - phase/enable interface
#define LEFT_MOTOR_NSLEEP 9
#define LEFT_MOTOR_PHASE 6
#define LEFT_MOTOR_PE_ENABLE 5

// driver pins for left motor - phase/enable interface
#define RIGHT_MOTOR_NSLEEP 11
#define RIGHT_MOTOR_PHASE 12
#define RIGHT_MOTOR_PE_ENABLE 10

// driver pins for right motor
#define RIGHT_MOTOR_ENABLE_PIN 9
#define RIGHT_MOTOR_PIN1 6
#define RIGHT_MOTOR_PIN2 5

// driver pins for right motor
#define LEFT_MOTOR_ENABLE_PIN 10
#define LEFT_MOTOR_PIN1 11
#define LEFT_MOTOR_PIN2 12

// stop mode for phase/enable interface
#define STOP_MODE_BRAKE 0
#define STOP_MODE_COAST 1

// PID variables for velocity and difference control
#define VELOCITY_P 0.03
#define VELOCITY_I 0.0000002
#define VELOCITY_D 0.00001
#define DIFF_P 0.0

#define ANGLE_RATE_RATIO 0.4
#define ANGLE_RESPONSE 0.006
#define SPEED_RESPONSE 0.06
#define POSITION_RESPONSE 0.000

#define BALANCE_POINT_COEFF 0.00

#define MOTOR_BACKWARD 0 
#define MOTOR_FORWARD 1

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

extern int32_t motor_ctrl_right_velocity, motor_ctrl_left_velocity;

// control the motor from power -100 to 100
void control_motor(uint8_t motor, int8_t power);

// control motor using the phase/enable method
void control_motor_phase_en(uint8_t motor, int8_t power, bool stop_mode);

// PID control on motor velocity and tick diff
void motor_ctrl_vel_diff(int32_t desired_vel, int32_t desired_diff);

// initialize motors
void init_motors();

// read velocities using the hall effect sensors in units of ticks per second
void read_velocities();

void balance_point_control(int32_t angle, int32_t ang_vel, int32_t vel, int32_t pos);

#endif // MOVE_MOTOR_H
