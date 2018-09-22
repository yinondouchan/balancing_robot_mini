#include <stdio.h>

#include "hall_effect_sensor.h"
#include "motor_control.h"

void setup()
{
  Serial.begin(9600);
  interrupts();
  init_motors();
  init_hall_effect_sensors();
}

void loop()
{
//  control_motor(MOTOR_FORWARD, 100);
  delay(100);
  char msg[50];
  snprintf(msg, sizeof(msg), "%d, %d", get_tick_count1(), get_tick_count2());
  Serial.println(msg);
}
