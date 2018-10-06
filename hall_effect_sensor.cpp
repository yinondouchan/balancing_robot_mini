#include "hall_effect_sensor.h"

// hall effect sensor states
volatile uint8_t hall_effect1_output1_state;
volatile uint8_t hall_effect1_output2_state;
volatile uint8_t hall_effect2_output1_state;
volatile uint8_t hall_effect2_output2_state;

volatile int tick_count_left;
volatile int tick_count_right;

ISR (PCINT0_vect) // handle pin change interrupt for pins A0 to A5
{   
     if(digitalRead(RIGHT_MOTOR_HALL_EFFECT_OUT1_PIN) != hall_effect2_output1_state)
    {
        // motor 2 sensor 1 changed
        hall_effect2_output1_state = !hall_effect2_output1_state;
        tick_count_right += hall_effect2_output1_state == hall_effect2_output2_state ? 1 : -1;
    }
     
    if(digitalRead(RIGHT_MOTOR_HALL_EFFECT_OUT2_PIN) != hall_effect2_output2_state)
    {
        // motor 2 sensor 2 changed
        hall_effect2_output2_state = !hall_effect2_output2_state;
        if (QUADRATURE) tick_count_right += hall_effect2_output1_state != hall_effect2_output2_state ? 1 : -1;
    }  
} 

ISR (PCINT2_vect) // handle pin change interrupt for pins D0 to D7
{   
    if(digitalRead(LEFT_MOTOR_HALL_EFFECT_OUT1_PIN) != hall_effect1_output1_state)
    {
        // motor 1 sensor 1 changed
        hall_effect1_output1_state = !hall_effect1_output1_state;
        tick_count_left += hall_effect1_output1_state == hall_effect1_output2_state ? 1 : -1;
    }
     
    if(digitalRead(LEFT_MOTOR_HALL_EFFECT_OUT2_PIN) != hall_effect1_output2_state)
    {
        // motor 1 sensor 2 changed
        hall_effect1_output2_state = !hall_effect1_output2_state;
        if (QUADRATURE) tick_count_left += hall_effect1_output1_state != hall_effect1_output2_state ? 1 : -1;
    }
} 

int32_t get_tick_count_left()
{
	return tick_count_left;
}

int32_t get_tick_count_right()
{
	return tick_count_right;
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void init_hall_effect_sensors()
{
        Serial.println("Initializing Hall Effect sensors");
	hall_effect1_output1_state = 0;
	hall_effect1_output2_state = 0;
	hall_effect2_output1_state = 0;
	hall_effect2_output2_state = 0;
	tick_count_left = 0;
	tick_count_right = 0;
	
	// set output pin to pullup
	pinMode(LEFT_MOTOR_HALL_EFFECT_OUT1_PIN, INPUT); 
	pinMode(LEFT_MOTOR_HALL_EFFECT_OUT2_PIN, INPUT); 
	pinMode(RIGHT_MOTOR_HALL_EFFECT_OUT1_PIN, INPUT); 
	pinMode(RIGHT_MOTOR_HALL_EFFECT_OUT2_PIN, INPUT); 
	
	// set pin change interrupts
        pciSetup(LEFT_MOTOR_HALL_EFFECT_OUT1_PIN);
        pciSetup(LEFT_MOTOR_HALL_EFFECT_OUT2_PIN);
        pciSetup(RIGHT_MOTOR_HALL_EFFECT_OUT1_PIN);
        pciSetup(RIGHT_MOTOR_HALL_EFFECT_OUT2_PIN);
}
