#ifndef IR_H
#define IR_H

#include <Arduino.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <boarddefs.h>
#include <IRremoteInt.h>
#include <IRremote.h>

#define IR_PIN 4

extern volatile int32_t ir_desired_vel;
extern volatile int32_t ir_ctrl_vel_diff;

void ir_init();

bool ir_read();

void ir_control();

#endif // IR_H
