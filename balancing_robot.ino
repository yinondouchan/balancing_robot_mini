#include <Wire.h>
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"
#include "ir.h"

LSM6 imu;

int32_t x, y, z, wx, wy, wz;
unsigned long timestamp, time_diff;

bool upright;

void setup()
{ 
  upright = false;

  //IR
  ir_init();

  // motors and encoders
  init_hall_effect_sensors();
  init_motors();

  // serial and i2c
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000L);

  // imu
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  delay(200);
  
  // complementary filter
  compl_filter_init();

  // gyro calibration
  Serial.println("Calibrating gyroscope");
  calibrate_gyro(&imu);
  Serial.println("Done calibrating gyroscope");
  
  timestamp = millis();
}

void loop()
{
  //timestamp = micros();
  // get tilt angle
  compl_filter_read(&imu);
  x = cf_angle_x - 90000;

  // get tilt angular velocity
  int32_t imu_gx = imu.g.x;
  wx = imu_gx - gyro_bias_x;

  // get tick count from motor encoders
  int32_t t_right = get_tick_count_right();
  int32_t t_left = get_tick_count_left();

  // get motor velocities in ticks per second
  read_velocities();
  ir_control();

  if ((x < -60000) || (x > 60000)) upright = false;

  // make sure the robot starts balancing itself only when it is upright
  if (!upright)
  {
      control_motor_phase_en(LEFT_MOTOR, 0, STOP_MODE_BRAKE);
      control_motor_phase_en(RIGHT_MOTOR, 0, STOP_MODE_BRAKE);
      upright = (x > -15000) && (x < 15000);
      return;
  }

  // balance the robot
  balance_point_control(x, wx, (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2 - ir_ctrl_vel, ir_ctrl_vel_diff);
}
