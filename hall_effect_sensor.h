#ifndef HALL_EFFECT_SENSOR_H
#define HALL_EFFECT_SENSOR_H

#include <stdint.h>
#include <Arduino.h>

#define LEFT_MOTOR_HALL_EFFECT_OUT1_PIN 2
#define LEFT_MOTOR_HALL_EFFECT_OUT2_PIN 7
#define RIGHT_MOTOR_HALL_EFFECT_OUT1_PIN A0
#define RIGHT_MOTOR_HALL_EFFECT_OUT2_PIN A5

int get_tick_count_left();
int get_tick_count_right();
void init_hall_effect_sensors();
void motor1_sensor1_edge_change();
void motor1_sensor2_edge_change();

#endif // HALL_EFFECT_SENSOR_H
