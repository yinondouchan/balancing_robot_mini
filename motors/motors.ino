#include "hall_effect_sensor.h"
#include "motor_control.h"

void setup()
{
  Serial.begin(9600);
  init_motors();
  enable_motors();
  init_hall_effect_sensors();
}

void loop()
{
  delay(10);
}
