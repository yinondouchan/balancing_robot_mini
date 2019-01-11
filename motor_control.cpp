#include "ir.h"
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"

#define ACCEL_LPF_TC 600000.0
#define CLIP(x, val_low, val_high) x = x > val_high ? val_high : x < val_low ? val_low : x;

// left and right motor velocities in ticks per second
int32_t motor_ctrl_right_velocity, motor_ctrl_left_velocity;

int32_t last_left_tick_count, last_right_tick_count;

int32_t vel_p_left, vel_p_right;
int32_t vel_i_left, vel_i_right;
int32_t prev_vel, prev_angle;
int32_t prev_pos_error;

float vel_lpf;
float angle_lpf;
unsigned long vel_timestamp;
unsigned long vel_ctrl_timestamp;
bool read_vel_ctrl_once;
bool read_bp_once;
bool read_vel_once;

float balance_point_power, prev_bp_power;
unsigned long balance_point_timestamp;

float bp_i;

unsigned long pos_timestamp;
int32_t pos_output_vel, pos_output_vel_lpf;
bool read_pos_once;
int32_t pos_error_sum;

int trimming;
unsigned long calib_timestamp;
bool read_calib_timestamp_once;

// control a motor with power from -100 to 100
void control_motor(uint8_t motor, int8_t power, bool stop_mode)
{
    uint8_t dir = power < 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
    power = power < 0 ? -power : power;
    
    int pwm_power = ZERO_MOTION_POWER + (255 - ZERO_MOTION_POWER) * power / 100;
    uint8_t enable_pin = motor == LEFT_MOTOR ? LEFT_MOTOR_ENABLE_PIN : RIGHT_MOTOR_ENABLE_PIN;
    uint8_t pin1 = motor == LEFT_MOTOR ? LEFT_MOTOR_PIN1 : RIGHT_MOTOR_PIN1;
    uint8_t pin2 = motor == LEFT_MOTOR ? LEFT_MOTOR_PIN2 : RIGHT_MOTOR_PIN2;
    
    if(power == 0) pwm_power = 0;
    
    if (stop_mode == STOP_MODE_COAST) analogWrite(enable_pin, pwm_power);
    else digitalWrite(enable_pin, HIGH);  // STOP_MODE_BRAKE
    
    if(dir == MOTOR_BACKWARD)
    {
        if (stop_mode == STOP_MODE_BRAKE)
        {
            analogWrite(pin1, pwm_power);
            digitalWrite(pin2, LOW);
        }
        else  // STOP_MODE_COAST
        {
            digitalWrite(pin2, HIGH);
            digitalWrite(pin1, LOW);
        }
    }
    else // MOTOR_FORWARD
    {
        if (stop_mode == STOP_MODE_BRAKE)
        {
            digitalWrite(pin1, LOW);
            analogWrite(pin2, pwm_power);
        }
        else  // STOP_MODE_COAST
        {
            digitalWrite(pin1, HIGH);
            digitalWrite(pin2, LOW);
        }
    }
}

// control a motor with power from -100 to 100
void control_motor_phase_en(uint8_t motor, int8_t power, bool stop_mode)
{
    uint8_t dir = power < 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
    power = power < 0 ? -power : power;
    
    int pwm_power = ZERO_MOTION_POWER + (255 - ZERO_MOTION_POWER) * power / 100;
    uint8_t enable_pin = motor == LEFT_MOTOR ? LEFT_MOTOR_PE_ENABLE : RIGHT_MOTOR_PE_ENABLE;
    uint8_t phase_pin = motor == LEFT_MOTOR ? LEFT_MOTOR_PHASE : RIGHT_MOTOR_PHASE;
    uint8_t nsleep_pin = motor == LEFT_MOTOR ? LEFT_MOTOR_NSLEEP : RIGHT_MOTOR_NSLEEP;
    
    if(power == 0) pwm_power = 0;

    if (stop_mode == STOP_MODE_BRAKE)
    {
      analogWrite(enable_pin, pwm_power);
      digitalWrite(nsleep_pin, HIGH);
    }
    else // stop_mode == STOP_MODE_COAST
    {
      analogWrite(nsleep_pin, pwm_power);
      digitalWrite(enable_pin, HIGH);
    }
    
    if(dir == MOTOR_BACKWARD)
    {
        digitalWrite(phase_pin, LOW);
    }
    else // MOTOR_FORWARD
    {
        digitalWrite(phase_pin, HIGH);
    }
}

void motor_ctrl_vel_diff(int32_t desired_vel, int32_t desired_diff, bool stop_mode)
{
    unsigned long dt = micros() - vel_ctrl_timestamp;
    vel_ctrl_timestamp = micros();

    if (!read_vel_ctrl_once)
    {
        read_vel_ctrl_once = true;
        return;
    }
    
    // velocities
    int32_t r_vel = motor_ctrl_right_velocity;
    int32_t l_vel = motor_ctrl_left_velocity;

    // velocity errors
    int32_t error_left = desired_vel - l_vel + desired_diff;
    int32_t error_right = desired_vel - r_vel - desired_diff;

    // D components
    int32_t vel_d_left = 1000*(vel_p_left - error_left)/dt;
    int32_t vel_d_right = 1000*(vel_p_right - error_right)/dt;

    // P components
    vel_p_left = error_left;
    vel_p_right = error_right;

    // I components
    vel_i_left += vel_p_left * dt;
    vel_i_right += vel_p_right * dt;

    CLIP(vel_i_left, -50 / VELOCITY_I, 50 / VELOCITY_I);
    CLIP(vel_i_right, -50 / VELOCITY_I, 50 / VELOCITY_I);

    int32_t power_left = VELOCITY_P * vel_p_left + VELOCITY_I * vel_i_left + VELOCITY_D * vel_d_left;
    int32_t power_right = VELOCITY_P * vel_p_right + VELOCITY_I * vel_i_right + VELOCITY_D * vel_d_right;

    CLIP(power_left, -100, 100);
    CLIP(power_right, -100, 100);

    control_motor(LEFT_MOTOR, power_left, stop_mode);
    control_motor(RIGHT_MOTOR, power_right, stop_mode);
}

void read_velocities()
{
    // measure time delta from last read
    unsigned long now = micros();
    unsigned long dt = now - vel_timestamp;
    vel_timestamp = now;
    
    if (!read_vel_once)
    {
        read_vel_once = true;
        return;
    }
    
    // get current tick count for both motors
    int64_t tick_count_left = get_tick_count_left();
    int64_t tick_count_right = get_tick_count_right();
    
    // calculate velocity in ticks per second by calculating delta_ticks / delta_t (sec)
    motor_ctrl_right_velocity = 1000000 * (tick_count_right - last_right_tick_count) / dt;
    motor_ctrl_left_velocity = 1000000 * (tick_count_left - last_left_tick_count) / dt;
    
    // save tick counts for next iteration
    last_left_tick_count = tick_count_left;
    last_right_tick_count = tick_count_right;
}

void balance_point_control_old(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel)
{
    // measure time
    unsigned long dt = micros() - balance_point_timestamp;
    balance_point_timestamp = micros();

    // get angle, angular velocity and velocity from IMU and encoders
    int32_t angle = cf_angle_x;
    int32_t ang_vel = imu->g.x - gyro_bias_x;
    int32_t vel = desired_vel - (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2;

    // do not control if wasn't here at least once in order for dt to be measured correctly
    if (!read_bp_once)
    {
        read_bp_once = true;
        return;
    }

    // low pass filter on given linear velocity
    vel_lpf = VEL_LPF_TC / (VEL_LPF_TC + dt) * vel_lpf + dt / (VEL_LPF_TC + dt) * vel;
    //angle_lpf = ANGLE_LPF_TC / (ANGLE_LPF_TC + dt) * angle_lpf + dt / (ANGLE_LPF_TC + dt) * angle;

    int32_t balance_angle = angle - BALANCE_ANGLE - trimming;
    int32_t rising_angle_offset = ang_vel * ANGLE_RATE_RATIO + balance_angle + 1.0 * (angle - prev_angle) / dt;

    /*float angle_response = max(0.001 - abs(ANGLE_RESPONSE * rising_angle_offset - BP_SPEED_I * vel) / 10000.0, ANGLE_RESPONSE);
    float speed_respose = BP_SPEED_I + (0.02 - BP_SPEED_I) * ((angle_response - ANGLE_RESPONSE) / (0.001 - ANGLE_RESPONSE));
    Serial.println(10000*angle_response);*/
    // add to power the tilt-angle element and the I element of the speed
    balance_point_power += (ANGLE_RESPONSE * rising_angle_offset - BP_SPEED_I * vel) * dt / 1000;
    CLIP(balance_point_power, -10000, 10000);

    // add to power the P and D elements for speed
    float power = balance_point_power + BP_SPEED_P * vel_lpf + (BP_SPEED_D  / dt) * (vel_lpf - prev_vel);
    CLIP(power, -10000, 10000);

    // control motor velocities
    motor_ctrl_vel_diff(power, desired_ang_vel, STOP_MODE_BRAKE);

    prev_vel = vel_lpf;
    prev_angle = angle_lpf;
}

void balance_point_control(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel)
{
    // measure time
    unsigned long dt = micros() - balance_point_timestamp;
    balance_point_timestamp = micros();

    // get angle, angular velocity and velocity from IMU and encoders
    int32_t angle = cf_angle_x;
    int32_t ang_vel = imu->g.x - gyro_bias_x;
    int32_t vel = desired_vel - (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2;

    // do not control if wasn't here at least once in order for dt to be measured correctly
    if (!read_bp_once)
    {
        read_bp_once = true;
        return;
    }

    // low pass filter on given linear velocity
    vel_lpf = VEL_LPF_TC2 / (VEL_LPF_TC2 + dt) * vel_lpf + dt / (VEL_LPF_TC2 + dt) * vel;

    bp_i += 0.00001 * vel;
    CLIP(bp_i, -90000, 90000);
    int32_t bp_angle_offset = 20.0 * vel + bp_i * dt + (7500000.0 / dt) * (vel_lpf - prev_vel);
    CLIP(bp_angle_offset, -90000, 90000);
    
    int32_t rising_angle_offset = ang_vel * ANGLE_RATE_RATIO + angle - BALANCE_ANGLE - bp_angle_offset;

    // add to power the tilt-angle element and the I element of the speed
    balance_point_power += (ANGLE_RESPONSE * rising_angle_offset) * dt / 1000;
    CLIP(balance_point_power, -10000, 10000);

    // added a D element which eliminates oscillations
    float power = balance_point_power + (BP_SPEED_D  / dt) * (vel_lpf - prev_vel);
    CLIP(power, -10000, 10000);

    // control motor velocities
    motor_ctrl_vel_diff(power, desired_ang_vel, STOP_MODE_BRAKE);

    prev_vel = vel_lpf;
}

void simple_pid_control(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel)
{
    // measure time
    unsigned long dt = micros() - balance_point_timestamp;
    balance_point_timestamp = micros();

    // get angle, angular velocity and velocity from IMU and encoders
    int32_t angle = cf_angle_x - 90000;
    int32_t ang_vel = imu->g.x - gyro_bias_x;
    int32_t vel = desired_vel - (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2;

    // do not control if wasn't here at least once in order for dt to be measured correctly
    if (!read_bp_once)
    {
        read_bp_once = true;
        return;
    }

    // low pass filter on given linear velocity
    vel_lpf = VEL_LPF_TC2 / (VEL_LPF_TC2 + dt) * vel_lpf + dt / (VEL_LPF_TC2 + dt) * vel;
    
    int32_t balance_angle = angle - BALANCE_ANGLE;
    angle_lpf = ANGLE_LPF_TC2 / (ANGLE_LPF_TC2 + dt) * angle_lpf + dt / (ANGLE_LPF_TC2 + dt) * balance_angle;


    bp_i += 0.02 * vel;
    CLIP(bp_i, -90000, 90000);
    int32_t bp_angle_offset = 0.0; //3.0 * vel_lpf + bp_i + (2000000.0 / dt) * (vel_lpf - prev_vel);
    CLIP(bp_angle_offset, -90000, 90000);

    int32_t angle_error = bp_angle_offset - balance_angle;
    int32_t angle_pid = -0.1 * angle_error - (100000.0 / dt) * (angle_lpf - prev_angle);

    CLIP(angle_pid, -10000, 10000);

    // control motor velocities
    motor_ctrl_vel_diff(angle_pid, desired_ang_vel, STOP_MODE_COAST);

    prev_vel = vel_lpf;
    prev_angle = angle_lpf;
}

void position_control(LSM6 *imu, int32_t desired_position, int32_t max_velocity, int32_t desired_ang_vel)
{
    int32_t t_right = get_tick_count_right();
    int32_t t_left = get_tick_count_left();
    int32_t actual_position = (t_right + t_left) / 2;
    int32_t pos_error = desired_position - actual_position;
    unsigned long now = micros();
    unsigned long dt = now - pos_timestamp;

    pos_error_sum += pos_error * dt;
    CLIP(pos_error_sum, -10000, 10000);
    pos_output_vel = 1.0 * pos_error + 0.0 * pos_error_sum + 2000000.0 * (pos_error - prev_pos_error) / dt;

    pos_timestamp = now;
    
    // do not control if wasn't here at least once in order for dt to be measured correctly
    if (!read_pos_once)
    {
        read_pos_once = true;
        return;
    }
    
    CLIP(pos_output_vel, -max_velocity, max_velocity);
    pos_output_vel_lpf = 600000.0 / (600000.0 + dt) * pos_output_vel_lpf + dt / (600000.0 + dt) * pos_output_vel;

    balance_point_control(imu, pos_output_vel_lpf, desired_ang_vel);

    prev_pos_error = pos_error;
}

void set_trimming(int trimming_val)
{
    trimming = trimming_val;
}

void calibrate_balance_angle()
{
    unsigned long now = micros();
    unsigned long dt = now - calib_timestamp;
    calib_timestamp = now;
    if (!read_calib_timestamp_once) 
    {
        read_calib_timestamp_once = true;
        return;
    }
    
    int vel = (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2;
    trimming -= 0.00001 * vel * dt;
    CLIP(trimming, -5000, 5000);
}

void motor_ctrl_reset()
{
  balance_point_power = 0;
  vel_lpf = 0;
  angle_lpf = 0;
  prev_vel = 0;
  prev_angle = 0;
  read_bp_once = false;

  vel_p_left = 0;
  vel_p_right = 0;
  vel_i_left = 0;
  vel_i_right = 0;
  read_vel_ctrl_once = false;

  bp_i = 0;
}

void init_motors()
{
  Serial.println("Initializing motor controls");
  motor_ctrl_right_velocity = 0;
  motor_ctrl_left_velocity = 0;
  last_left_tick_count = 0;
  last_right_tick_count = 0;
  vel_i_left = 0;
  vel_i_right = 0;
  balance_point_power = 0.0;
  prev_bp_power = 0.0;
  vel_p_left = 0;
  vel_p_right = 0;
  vel_i_left = 0;
  vel_i_right = 0;
  prev_vel = 0;
  prev_angle = 0;
  prev_pos_error = 0;
  pos_error_sum = 0;
  vel_lpf = 0;
  angle_lpf = 0;
  read_vel_ctrl_once = false;
  read_bp_once = false;
  read_pos_once = false;
  read_calib_timestamp_once = false;
  pos_output_vel_lpf = 0;
  bp_i = 0;
  trimming = 0;
  
  vel_ctrl_timestamp = micros();
  balance_point_timestamp = micros();
  
  read_vel_once = false;
}
