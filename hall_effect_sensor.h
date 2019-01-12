#ifndef HALL_EFFECT_SENSOR_H
#define HALL_EFFECT_SENSOR_H

#include <stdint.h>
#include <Arduino.h>
#include "definitions.h"

//#define LEFT_MOTOR_HALL_EFFECT_OUT1_PIN 7
//#define LEFT_MOTOR_HALL_EFFECT_OUT2_PIN 3
//#define RIGHT_MOTOR_HALL_EFFECT_OUT1_PIN 8
//#define RIGHT_MOTOR_HALL_EFFECT_OUT2_PIN 11

#define LEFT_MOTOR_HALL_EFFECT_OUT1_PIN 8
#define LEFT_MOTOR_HALL_EFFECT_OUT2_PIN 13
#define RIGHT_MOTOR_HALL_EFFECT_OUT1_PIN 7
#define RIGHT_MOTOR_HALL_EFFECT_OUT2_PIN 2

#define QUADRATURE true

int32_t get_tick_count_left();
int32_t get_tick_count_right();
void init_hall_effect_sensors();
void motor1_sensor1_edge_change();
void motor1_sensor2_edge_change();

#endif // HALL_EFFECT_SENSOR_H
