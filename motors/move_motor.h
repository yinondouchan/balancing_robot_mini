#ifndef MOVE_MOTOR_H
#define MOVE_MOTOR_H

#include <stdint.h>

#define ENABLE_PIN 7
#define MOTOR1_PIN1 23
#define MOTOR1_PIN2 26

// PID variables for position control
#define POSITION_P 5.0
#define POSITION_I 0.1
#define POSITION_D 40.0

// error tolerance in position control
#define ERROR_TOLERANCE_TICKS 0

float prev_error;
float error_integral;
float error_difference;

void position_control(int ticks);
void control_motor(float power);

void init_motor();

#endif // MOVE_MOTOR_H
