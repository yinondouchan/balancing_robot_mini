#include "hall_effect_sensor.h"

// hall effect sensor states
volatile uint8_t hall_effect1_output1_state;
volatile uint8_t hall_effect1_output2_state;
volatile uint8_t hall_effect2_output1_state;
volatile uint8_t hall_effect2_output2_state;

volatile int tick_count1;
volatile int tick_count2;

ISR (PCINT1_vect) // handle pin change interrupt for D0 to D7
{   
    /*if(digitalRead(MOTOR1_HALL_EFFECT_OUT1_PIN) != hall_effect1_output1_state)
    {
        // motor 1 sensor 1 changed
        hall_effect1_output1_state = !hall_effect1_output1_state;
        tick_count1 += hall_effect1_output1_state == hall_effect1_output2_state ? 1 : -1;
    }
     
    if(digitalRead(MOTOR1_HALL_EFFECT_OUT2_PIN) != hall_effect1_output2_state)
    {
        // motor 1 sensor 2 changed
        hall_effect1_output2_state = !hall_effect1_output2_state;
    }*/
     
    if(digitalRead(MOTOR2_HALL_EFFECT_OUT1_PIN) != hall_effect2_output1_state)
    {
        // motor 2 sensor 1 changed
        hall_effect2_output1_state = hall_effect2_output1_state;
        tick_count2 += hall_effect2_output1_state == hall_effect2_output2_state ? 1 : -1;
    }
     
    if(digitalRead(MOTOR2_HALL_EFFECT_OUT2_PIN) != hall_effect2_output2_state)
    {
        // motor 2 sensor 2 changed
        hall_effect2_output2_state = !hall_effect2_output2_state;
    }
     
} 

ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7
{   
    if(digitalRead(MOTOR1_HALL_EFFECT_OUT1_PIN) != hall_effect1_output1_state)
    {
        // motor 1 sensor 1 changed
        hall_effect1_output1_state = !hall_effect1_output1_state;
        tick_count1 += hall_effect1_output1_state == hall_effect1_output2_state ? 1 : -1;
    }
     
    if(digitalRead(MOTOR1_HALL_EFFECT_OUT2_PIN) != hall_effect1_output2_state)
    {
        // motor 1 sensor 2 changed
        hall_effect1_output2_state = !hall_effect1_output2_state;
    }
     
    /*if(digitalRead(MOTOR2_HALL_EFFECT_OUT1_PIN) != hall_effect2_output1_state)
    {
        // motor 2 sensor 1 changed
        hall_effect2_output1_state = hall_effect2_output1_state;
        tick_count2 += hall_effect2_output1_state == hall_effect2_output2_state ? 1 : -1;
    }
     
    if(digitalRead(MOTOR2_HALL_EFFECT_OUT2_PIN) != hall_effect2_output2_state)
    {
        // motor 2 sensor 2 changed
        hall_effect2_output2_state = !hall_effect2_output2_state;
    }*/
     
} 

int get_tick_count1()
{
	return tick_count1;
}

int get_tick_count2()
{
	return tick_count2;
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
	tick_count1 = 0;
	tick_count2 = 0;
	
	// set output pin to pullup
	pinMode(MOTOR1_HALL_EFFECT_OUT1_PIN, INPUT_PULLUP); 
	pinMode(MOTOR1_HALL_EFFECT_OUT2_PIN, INPUT_PULLUP); 
	pinMode(MOTOR2_HALL_EFFECT_OUT1_PIN, INPUT_PULLUP); 
	pinMode(MOTOR2_HALL_EFFECT_OUT2_PIN, INPUT_PULLUP); 
	
	// set pin change interrupts
        pciSetup(MOTOR1_HALL_EFFECT_OUT1_PIN);
        pciSetup(MOTOR1_HALL_EFFECT_OUT2_PIN);
        pciSetup(MOTOR2_HALL_EFFECT_OUT1_PIN);
        pciSetup(MOTOR2_HALL_EFFECT_OUT2_PIN);
}
