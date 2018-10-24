#include "ir.h"

#define POWER 0x10EFD827 
#define A 0x10EFF807 
#define B 0x10EF7887
#define C 0x10EF58A7
#define UP 0x10EFA05F
#define DOWN 0x10EF00FF
#define LEFT 0x10EF10EF
#define RIGHT 0x10EF807F
#define SELECT 0x10EF20DF
#define IR_REPEAT 0xFFFFFFFF

volatile int32_t ir_desired_vel;
volatile int32_t ir_ctrl_vel_diff;
volatile int32_t ir_absolute_vel;
volatile int32_t last_cmd;

IRrecv ir_receiver(IR_PIN);
decode_results ir_results;

void ir_init()
{
    ir_desired_vel = 0;
    ir_ctrl_vel_diff = 0;
    ir_absolute_vel = 200;
    ir_receiver.enableIRIn(); // Start the receiver
}

bool ir_read()
{
    if (ir_receiver.decode(&ir_results))
    {
        return true;
    }

    return false;
}

void ir_control()
{   
    if (ir_read())
    {
        int32_t value = ir_results.value;
        if (value == IR_REPEAT)
        {
            value = last_cmd;
        }
        if (value == POWER) 
        {  
        }
        if (value == A) 
        {   
            ir_absolute_vel += 100;
        }
        if (value == B) 
        {
            ir_absolute_vel -= 100;
            ir_absolute_vel = ir_absolute_vel < 0 ? 0 : ir_absolute_vel;
        }
        if (value == C) 
        {
            ir_ctrl_vel_diff = 0;
        }
        if (value == UP) 
        {
            ir_desired_vel += ir_absolute_vel;
        }
        if (value == DOWN) 
        {
            ir_desired_vel -= ir_absolute_vel;
        }
        if (value == LEFT) 
        {
            ir_ctrl_vel_diff -= 100;
        }
        if (value == RIGHT) 
        {
            ir_ctrl_vel_diff += 100;
        }
        if (value == SELECT) 
        {
            ir_desired_vel = 0;
            ir_ctrl_vel_diff = 0;
        }

        last_cmd = value;

        ir_receiver.resume();
    }
}
