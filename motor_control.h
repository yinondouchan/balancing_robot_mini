#ifndef MOVE_MOTOR_H
#define MOVE_MOTOR_H

#include <stdint.h>
#include <Arduino.h>

// driver pins for left motor
#define LEFT_MOTOR_ENABLE_PIN 11
#define LEFT_MOTOR_PIN1 12
#define LEFT_MOTOR_PIN2 13

// driver pins for right motor
#define RIGHT_MOTOR_ENABLE_PIN 8
#define RIGHT_MOTOR_PIN1 9
#define RIGHT_MOTOR_PIN2 10

// PID variables for angle control
#define ANGLE_P 5.0
#define ANGLE_I 0.0
#define ANGLE_D 40.0

// PID variables for velocity control
#define VELOCITY_P 5.0
#define VELOCITY_I 0.1
#define VELOCITY_D 40.0

#define MOTOR_BACKWARD 0  
#define MOTOR_FORWARD 1

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

extern int motor_ctrl_right_velocity, motor_ctrl_left_velocity;

// control motors using PID for keeping a desired angle given current angle
void angle_control(int16_t desired_angle, int16_t current_angle);

// control the motor from power -100 to 100
void control_motor(uint8_t motor, uint8_t power);

// initialize motors
void init_motors();

// read velocities using the hall effect sensors in units of ticks per second
void read_velocities();

#endif // MOVE_MOTOR_H
