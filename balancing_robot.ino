#include <Wire.h>
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"
#include "ir.h"
#include "bluetooth.h"

#define ACCEL_LPF_TC 000000.0

LSM6 imu;

int32_t x, y, z, wx, wy, wz, ctrl_vel, ctrl_pos;
unsigned long timestamp, time_diff;

bool upright;
bool estop;

void on_estop()
{
    estop = !estop;
}

void setup()
{ 
  upright = false;
  estop = false;

  //IR
  //ir_init();

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

  bt_init();
  bt_set_trim_callback(set_trimming);
  bt_set_estop_callback(on_estop);
  
  timestamp = micros();
}

void loop()
{
  unsigned long now = micros();
  unsigned long dt = now - timestamp;
  timestamp = now;

  // get tilt angle
  compl_filter_read(&imu);
  x = cf_angle_x;

  // get motor velocities in ticks per second
  read_velocities();
  //ir_control_read();
  if ((x < -60000) || (x > 60000)) upright = false;
  return;
  
  bt_read_joystick_control();

  // make sure the robot starts balancing itself only when it is upright. Also don't do anything if e-stop is on
  if (!upright || estop)
  {
      motor_ctrl_reset();
      control_motor(LEFT_MOTOR, 0, STOP_MODE_COAST);
      control_motor(RIGHT_MOTOR, 0, STOP_MODE_COAST);
      upright = (x > ((int32_t)BALANCE_ANGLE - 20000)) && (x < ((int32_t)BALANCE_ANGLE + 20000));
      return;
  }

  if (bt_desired_vel == 0) calibrate_balance_angle();

  // put a low pass filter on the desired velocity in order to smoothen it
  //ctrl_vel = ACCEL_LPF_TC / (ACCEL_LPF_TC + dt) * ctrl_vel + dt / (ACCEL_LPF_TC + dt) * ir_desired_vel;

  // balance the robot
  //position_control(&imu, ir_desired_vel, 2000, ir_ctrl_vel_diff);
  balance_point_control(&imu, bt_desired_vel, bt_desired_vel_diff);
  //simple_pid_control(&imu, ctrl_vel, ir_ctrl_vel_diff);
  delay(5);
}
