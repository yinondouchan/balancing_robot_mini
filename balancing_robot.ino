#include <Wire.h>
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"

LSM6 imu;

char report[150];
int32_t x, y, z, wx, wy, wz;
unsigned long timestamp;

bool initially_stable;

void setup()
{ 
  initially_stable = false;
  
  init_hall_effect_sensors();
  init_motors();

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000L);
  
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  
  compl_filter_init();
  Serial.println("Calibrating gyroscope");
  calibrate_gyro(&imu);
  Serial.println("Done calibrating gyroscope");
  timestamp = millis();
}

void loop()
{
  compl_filter_read(&imu);
  x = cf_angle_x - 90000;
  
  int32_t imu_gx = imu.g.x;
  wx = imu_gx - gyro_bias_x;
  
  int32_t t_right = get_tick_count_right();
  int32_t t_left = get_tick_count_left();

  read_velocities();

  if (!initially_stable)
  {
      initially_stable = (x > -30000) && (x < 30000);
      return;
  }
  
  balance_point_control(x, wx, (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2, (t_right + t_left)/2);
}
