#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "move_motor.h"
#include "hall_effect_sensor.h"

// PID control for position in ticks
void position_control(int ticks)
{
	int tick_count = get_tick_count();
	float error = (ticks - tick_count);
	
	if((error <= 10) && (error >= -10))
		error_integral += error;
	else
		error_integral = 0;
		
	error_difference = error - prev_error;
	
	float p = POSITION_P * error;
	float i = POSITION_I * error_integral;
	float d = POSITION_D * error_difference;
	
	// clip i to provide up to 10% difference in power
	i = i > 10 ? 10 : i;
	i = i < -10 ? -10 : i;
	
	float power = p + i + d;
	printf("tick count: %d, p: %f, i: %f, d: %f, output: %f\n", tick_count, p, i, d, power);
	
	
	
	// clip power to range -100 to 100
	power = power > 100 ? 100 : power;
	power = power < -100 ? -100 : power;
	if(abs(error) <= ERROR_TOLERANCE_TICKS)
		power = 0;
	control_motor(power);
	
	prev_error = error;
}

// control with power from -100 to 100
void control_motor(float power)
{
	if(power > 0)
	{
		// PWM gets values from 0 to 1024. motor starts moving from around 550
		int pwm_power = 512 + (1024 - 512) * power / 100;
		pwmWrite (MOTOR1_PIN2, pwm_power) ;
		pwmWrite(MOTOR1_PIN1, 0);
	}
	else if(power < 0)
	{
		int pwm_power = 512 + (1024 - 512) * -power / 100;
		pwmWrite (MOTOR1_PIN1, pwm_power) ;
		pwmWrite(MOTOR1_PIN2, 0);
	}
	else // power == 0
	{
		pwmWrite (MOTOR1_PIN1, 0) ;
		pwmWrite(MOTOR1_PIN2, 0);
	}
}

void init_motor()
{
	prev_error = 0;
	error_integral = 0;
	error_difference = 0;
	
	pwmSetMode(PWM_MODE_MS);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MOTOR1_PIN1, PWM_OUTPUT);
    pinMode(MOTOR1_PIN2, PWM_OUTPUT);
}

int main(void)
{
    wiringPiSetup();
	hall_effect_init();
	init_motor();

    digitalWrite(ENABLE_PIN, HIGH);
	
	int i, j;
	for(i = 1; i <= 1000; i++)
	{
		/*for(j = 0; j <= 100; j++)
		{
			position_control(i * 200);
			delay(10);
		}*/
		
		position_control(1500);
		delay(10);
	}
	
	//control_motor(0.1);
    //delay(5000);

    digitalWrite(ENABLE_PIN, LOW);
    pwmWrite(MOTOR1_PIN1, 0);
    pwmWrite(MOTOR1_PIN2, 0);
}
