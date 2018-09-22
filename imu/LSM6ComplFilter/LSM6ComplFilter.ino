#include <Wire.h>
#include <LSM6.h>
#include "compl_filter.h"

LSM6 imu;

char report[150];

void setup()
{
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
  imu.read();

  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z,
    imu.g.x, imu.g.y, imu.g.z);
  snprintf(report, sizeof(report), "Angle: %6d %6d %6d",
   x, y, z);
  Serial.println(report);

  delay(100);
}
