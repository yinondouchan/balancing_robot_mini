#include "hall_effect_sensor.h"

uint8_t hall_effect_output1_state;
uint8_t hall_effect_output2_state;
int tick_count;

void sensor1_edge_change()
{
	hall_effect_output1_state = digitalRead(MOTOR1_HALL_EFFECT_OUT1_PIN);
	tick_count += hall_effect_output1_state == hall_effect_output2_state ? 1 : -1;
}

void sensor2_edge_change()
{
	hall_effect_output2_state = digitalRead(MOTOR1_HALL_EFFECT_OUT2_PIN);
}

int get_tick_count()
{
	return tick_count;
}

void init_hall_effect_sensors()
{
	hall_effect_output1_state = 0;
	hall_effect_output2_state = 0;
	tick_count = 0;
	
	// set output pin to pullup
	pinMode(MOTOR1_HALL_EFFECT_OUT1_PIN, INPUT_PULLUP); 
	pinMode(MOTOR1_HALL_EFFECT_OUT2_PIN, INPUT_PULLUP); 
	pinMode(MOTOR2_HALL_EFFECT_OUT1_PIN, INPUT_PULLUP); 
	pinMode(MOTOR2_HALL_EFFECT_OUT2_PIN, INPUT_PULLUP); 
	
	// set falling and rising edge interrupts on this pin
	/*wiringPiISR(hall_effect_output1_pin, INT_EDGE_BOTH, sensor1_edge_change);
	wiringPiISR(hall_effect_output2_pin, INT_EDGE_BOTH, sensor2_edge_change);*/
}
