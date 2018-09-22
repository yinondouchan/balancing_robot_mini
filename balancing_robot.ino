#include <Wire.h>
#include "LSM6.h"
#include "compl_filter.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"

LSM6 imu;

char report[150];

void setup()
{
  init_hall_effect_sensors();
  init_motors();
  
  Serial.begin(9600);
  Wire.begin();
  
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
  int16_t x, y, z;
  compl_filter_read_angle(&imu, &x, &y, &z);

  snprintf(report, sizeof(report), "Angle: %6d %6d %6d",
   x, y, z);
  Serial.println(report);

  delay(100);
}
