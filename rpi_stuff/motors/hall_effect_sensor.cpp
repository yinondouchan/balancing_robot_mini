#include <wiringPi.h>
#include <stdio.h>

#include "hall_effect_sensor.h"

uint8_t hall_effect_output1_pin;
uint8_t hall_effect_output2_pin;
uint8_t hall_effect_output1_state;
uint8_t hall_effect_output2_state;
int tick_count;

void sensor1_edge_change()
{
	hall_effect_output1_state = digitalRead(hall_effect_output1_pin);
	tick_count += hall_effect_output1_state == hall_effect_output2_state ? 1 : -1;
	/*if(hall_effect_output1_state == 0)
		printf("falling edge 1, counter: %d\n", tick_count);
	else
		printf("rising edge 1, counter: %d\n", tick_count);*/
}

void sensor2_edge_change()
{
	hall_effect_output2_state = digitalRead(hall_effect_output2_pin);
	/*if(output2_state == 0)
		printf("falling edge 2, counter: %d\n", tick_count);
	else
		printf("rising edge 2, counter: %d\n", tick_count);*/
}

int get_tick_count()
{
	return tick_count;
}

void hall_effect_init()
{
	hall_effect_output1_pin = 4;
	hall_effect_output2_pin = 5;
	hall_effect_output1_state = 0;
	hall_effect_output2_state = 0;
	tick_count = 0;
	
	// set output pin to pullup
	pinMode(hall_effect_output1_pin, INPUT); 
	pinMode(hall_effect_output2_pin, INPUT); 
	pullUpDnControl(hall_effect_output1_pin, PUD_UP);
	pullUpDnControl(hall_effect_output2_pin, PUD_UP);
	
	// set falling and rising edge interrupts on this pin
	wiringPiISR(hall_effect_output1_pin, INT_EDGE_BOTH, sensor1_edge_change);
	wiringPiISR(hall_effect_output2_pin, INT_EDGE_BOTH, sensor2_edge_change);
}

/*int main()
{
	hall_effect_output1_pin = 4;
	hall_effect_output2_pin = 5;
	hall_effect_output1_state = 0;
	hall_effect_output2_state = 0;
	tick_count = 0;
	
	wiringPiSetup();
	
	// set output pin to pullup
	pinMode(hall_effect_output1_pin, INPUT); 
	pinMode(hall_effect_output2_pin, INPUT); 
	pullUpDnControl(hall_effect_output1_pin, PUD_UP);
	pullUpDnControl(hall_effect_output2_pin, PUD_UP);
	
	// set falling and rising edge interrupts on this pin
	wiringPiISR(hall_effect_output1_pin, INT_EDGE_BOTH, sensor1_edge_change);
	wiringPiISR(hall_effect_output2_pin, INT_EDGE_BOTH, sensor2_edge_change);
	
	while(1)
	{
		delay(100);
		fflush(stdout);
	}
	
	return 0;
}*/
