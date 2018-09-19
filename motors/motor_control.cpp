#include "motor_control.h"
#include "hall_effect_sensor.h"

void enable_motors()
{
  digitalWrite(MOTOR1_ENABLE_PIN, HIGH);
}

// control with power from -100 to 100
void control_motor(uint8_t power)
{
	if(power > 0)
	{
		// PWM gets values from 0 to 1024. motor starts moving from around 128
		int pwm_power = 128 + (255 - 128) * power / 255;
                Serial.println(pwm_power);
		analogWrite(MOTOR1_PIN2, pwm_power) ;
		analogWrite(MOTOR1_PIN1, 0);
	}
	else if(power < 0)
	{
		int pwm_power = 128 + (255 - 128) * -power / 255;
		analogWrite (MOTOR1_PIN1, pwm_power) ;
		analogWrite(MOTOR1_PIN2, 0);
	}
	else // power == 0
	{
		analogWrite (MOTOR1_PIN1, 0) ;
		analogWrite(MOTOR1_PIN2, 0);
	}
}

void init_motors()
{
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
}
