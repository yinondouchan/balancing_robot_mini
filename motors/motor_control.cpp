#include "motor_control.h"
#include "hall_effect_sensor.h"

#define ZERO_MOTION_POWER 32

// control with power from -100 to 100
void control_motor(uint8_t dir, uint8_t power)
{
    int pwm_power = ZERO_MOTION_POWER + (255 - ZERO_MOTION_POWER) * power / 100;
    Serial.println(pwm_power);
    if(power == 0) pwm_power = 0;
    
    analogWrite(MOTOR1_ENABLE_PIN, pwm_power);
    //analogWrite(MOTOR1_ENABLE_PIN, pwm_power);
    
    if(dir == MOTOR_BACKWARD)
    {
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR1_PIN1, LOW);
    }
    else // MOTOR_FORWARD
    {
        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
    }
}

void init_motors()
{
  Serial.println("Initializing motor controls");
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
}
