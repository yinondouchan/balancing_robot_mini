#include <Wire.h>
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"
#include "ir.h"

#define ACCEL_LPF_TC 600000.0

LSM6 imu;

int32_t x, y, z, wx, wy, wz, ctrl_vel, ctrl_pos;
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
  
  timestamp = micros();
}

void loop()
{
  unsigned long now = micros();
  unsigned long dt = now - timestamp;
  timestamp = now;
  
  // get tilt angle
  compl_filter_read(&imu);
  x = cf_angle_x - 90000;

  // get motor velocities in ticks per second
  read_velocities();
  ir_control_read();

  if ((x < -60000) || (x > 60000)) upright = false;

  // make sure the robot starts balancing itself only when it is upright
  if (!upright)
  {
      control_motor_phase_en(LEFT_MOTOR, 0, STOP_MODE_BRAKE);
      control_motor_phase_en(RIGHT_MOTOR, 0, STOP_MODE_BRAKE);
      upright = (x > (BALANCE_ANGLE - 20000)) && (x < (BALANCE_ANGLE + 20000));
      return;
  }

  // put a low pass filter on the desired velocity in order to smoothen it
  ctrl_vel = ACCEL_LPF_TC / (ACCEL_LPF_TC + dt) * ctrl_vel + dt / (ACCEL_LPF_TC + dt) * ir_desired_vel;
  
  // balance the robot
  //position_control(&imu, ir_desired_vel, 2000, ir_ctrl_vel_diff);
  balance_point_control(&imu, ctrl_vel, ir_ctrl_vel_diff);
}
