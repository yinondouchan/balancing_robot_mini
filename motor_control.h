#ifndef MOVE_MOTOR_H
#define MOVE_MOTOR_H

#include <stdint.h>
#include <Arduino.h>

#include "LSM6.h"

// driver pins for left motor - phase/enable interface
#define LEFT_MOTOR_NSLEEP 6 //9
#define LEFT_MOTOR_PHASE 2 //6
#define LEFT_MOTOR_PE_ENABLE 5 //5

// driver pins for right motor - phase/enable interface
#define RIGHT_MOTOR_NSLEEP 10 //11
#define RIGHT_MOTOR_PHASE 12 //12
#define RIGHT_MOTOR_PE_ENABLE 9 //10

// driver pins for right motor
#define RIGHT_MOTOR_ENABLE_PIN 6
#define RIGHT_MOTOR_PIN1 11
#define RIGHT_MOTOR_PIN2 10

// driver pins for right motor
#define LEFT_MOTOR_ENABLE_PIN 5
#define LEFT_MOTOR_PIN1 9
#define LEFT_MOTOR_PIN2 3

// stop mode for phase/enable interface
#define STOP_MODE_BRAKE 0
#define STOP_MODE_COAST 1

// PID variables for velocity and difference control
#define VELOCITY_P 0.03
#define VELOCITY_I 0.0000004
#define VELOCITY_D 0.00001

#define VEL_LPF_TC 200000.0
#define ANGLE_LPF_TC 100000.0
#define ANGLE_RATE_RATIO 0.8 // 0.8
#define ANGLE_RESPONSE 0.0003 // 0.0003

#define ANGLE_LPF_TC2 100000.0
#define VEL_LPF_TC2 200000.0

#define BP_SPEED_P -2.4 // -1.2
#define BP_SPEED_I 0.006 // 0.006
#define BP_SPEED_D 100000.0

#define BALANCE_POINT_COEFF 0.00

#define MOTOR_BACKWARD 0 
#define MOTOR_FORWARD 1

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define BALANCE_ANGLE 2000  // measured angle where robot is balanced
#define ZERO_MOTION_POWER 0

extern int32_t motor_ctrl_right_velocity, motor_ctrl_left_velocity;

// control the motor from power -100 to 100
void control_motor(uint8_t motor, int8_t power, bool stop_mode);

// control motor using the phase/enable method
void control_motor_phase_en(uint8_t motor, int8_t power, bool stop_mode);

// PID control on motor velocity and tick diff
void motor_ctrl_vel_diff(int32_t desired_vel, int32_t desired_diff, bool stop_mode);

// initialize motors
void init_motors();

// read velocities using the hall effect sensors in units of ticks per second
void read_velocities();

void position_control(LSM6 *imu, int32_t desired_position, int32_t max_velocity, int32_t desired_ang_vel);

void balance_point_control(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel);

void balance_point_control2(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel);

void simple_pid_control(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel);

void motor_ctrl_reset();

void set_trimming(int trimming);

void calibrate_balance_angle();

#endif // MOVE_MOTOR_H
