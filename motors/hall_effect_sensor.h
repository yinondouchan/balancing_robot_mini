#ifndef HALL_EFFECT_SENSOR_H
#define HALL_EFFECT_SENSOR_H

#include <stdint.h>
#include <Arduino.h>

#define MOTOR1_HALL_EFFECT_OUT1_PIN 2
#define MOTOR1_HALL_EFFECT_OUT2_PIN 7
#define MOTOR2_HALL_EFFECT_OUT1_PIN A0
#define MOTOR2_HALL_EFFECT_OUT2_PIN A5

inline unsigned char  digitalPinToInterrupt(unsigned char Interrupt_pin) { return Interrupt_pin; } //This isn't included in the stm32duino libs (yet)
#define portOutputRegister(port) (volatile byte *)( &(port->regs->ODR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#define portInputRegister(port) (volatile byte *)( &(port->regs->IDR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value

int get_tick_count1();
int get_tick_count2();
void init_hall_effect_sensors();
void motor1_sensor1_edge_change();
void motor1_sensor2_edge_change();

#endif // HALL_EFFECT_SENSOR_H
