#ifndef MOVE_MOTOR_H
#define MOVE_MOTOR_H

#include <stdint.h>
#include <Arduino.h>

// driver pins for left motor
#define LEFT_MOTOR_ENABLE_PIN 10
#define LEFT_MOTOR_PIN1 11
#define LEFT_MOTOR_PIN2 12

// driver pins for right motor
#define RIGHT_MOTOR_ENABLE_PIN 9
#define RIGHT_MOTOR_PIN1 6
#define RIGHT_MOTOR_PIN2 5

// PID variables for angle control
#define ANGLE_P 0.1 // 3.0
#define ANGLE_I 0.0001 // 0.0
#define ANGLE_D 0.0 // 10.0

// PID variables for velocity control
#define VELOCITY_P 0.002 // 0.002
#define VELOCITY_I 0.013 // 0.0001
#define VELOCITY_D -0.5 // -0.005

#define ANGLE_RATE_RATIO 0.8
#define ANGLE_RESPONSE 0.0003
#define SPEED_RESPONSE 0.006

#define BALANCE_POINT_COEFF 0.00

#define MOTOR_BACKWARD 0 
#define MOTOR_FORWARD 1

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

extern int motor_ctrl_right_velocity, motor_ctrl_left_velocity;

// control velocity using an I controller (units: ticks per second)
void velocity_control(int32_t desired_vel, int32_t current_vel, int32_t current_angle);

// control motors using PID for keeping a desired angle given current angle
void angle_control(int32_t desired_angle, int32_t current_angle);

// control the motor from power -100 to 100
void control_motor(uint8_t motor, int8_t power);

// initialize motors
void init_motors();

// read velocities using the hall effect sensors in units of ticks per second
void read_velocities();

void balance_point_control(int32_t angle, int32_t ang_vel, int32_t vel);

#endif // MOVE_MOTOR_H
