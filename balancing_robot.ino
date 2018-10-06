#include <Wire.h>
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"

LSM6 imu;

char report[150];
int32_t x, y, z, wx, wy, wz, prev_g;

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
  
  compl_filter_init();
  Serial.println("Calibrating gyroscope");
  calibrate_gyro(&imu);
  Serial.println("Done calibrating gyroscope");
}

void loop()
{
  
  compl_filter_read(&imu);
  x = cf_angle_x - 90000;
  y = cf_angle_y;
  z = cf_angle_z;
  
  int32_t imu_gx = imu.g.x;
  wx = imu_gx - gyro_bias_x;
  prev_g = imu_gx;
  
  int32_t t_right = get_tick_count_right();
  int32_t t_left = get_tick_count_left();

  read_velocities();
  //velocity_control(0, (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2, x);
  balance_point_control(x, wx, (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2);
  
//  angle_control(-12500, x);
  delay(2);
}
