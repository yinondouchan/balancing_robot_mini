#ifndef HALL_EFFECT_SENSOR_H
#define HALL_EFFECT_SENSOR_H

#include <stdint.h>
#include <Arduino.h>

#define MOTOR1_HALL_EFFECT_OUT1_PIN 7
#define MOTOR1_HALL_EFFECT_OUT2_PIN 8
#define MOTOR2_HALL_EFFECT_OUT1_PIN 0
#define MOTOR2_HALL_EFFECT_OUT2_PIN 0

int get_tick_count();
void init_hall_effect_sensors();
void motor1_sensor1_edge_change();
void motor1_sensor2_edge_change();

#endif // HALL_EFFECT_SENSOR_H
