#ifndef MOVE_MOTOR_H
#define MOVE_MOTOR_H

#include <stdint.h>
#include <Arduino.h>

#define MOTOR1_ENABLE_PIN 11
#define MOTOR1_PIN1 12
#define MOTOR1_PIN2 13

#define MOTOR2_ENABLE_PIN 7
#define MOTOR2_PIN1 23
#define MOTOR2_PIN2 26

// PID variables for position control
#define POSITION_P 5.0
#define POSITION_I 0.1
#define POSITION_D 40.0

// error tolerance in position control
#define ERROR_TOLERANCE_TICKS 0

#define MOTOR_BACKWARD 0  
#define MOTOR_FORWARD 1

// control the motor from power -100 to 100
void control_motor(uint8_t dir, uint8_t power);

// initialize motors
void init_motors();

#endif // MOVE_MOTOR_H
