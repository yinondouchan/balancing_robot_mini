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
    uint8_t enable_value, pin1_value, pin2_value;
    power = power < 0 ? -power : power;
    
    int pwm_power = ZERO_MOTION_POWER + (255 - ZERO_MOTION_POWER) * power / 100;
    
    if(power == 0) pwm_power = 0;
    
    if (stop_mode == STOP_MODE_COAST) enable_value = pwm_power;
    else enable_value = PWM_HIGH;  // STOP_MODE_BRAKE
    
    if(dir == MOTOR_BACKWARD)
    {
        if (stop_mode == STOP_MODE_BRAKE)
        {
            pin1_value = pwm_power;
            pin2_value = PWM_LOW;
        }
        else  // STOP_MODE_COAST
        {
            pin2_value = PWM_HIGH;
            pin1_value = PWM_LOW;
        }
    }
    else // MOTOR_FORWARD
    {
        if (stop_mode == STOP_MODE_BRAKE)
        {
            pin1_value = PWM_LOW;
            pin2_value = pwm_power;
        }
        else  // STOP_MODE_COAST
        {
            pin1_value = PWM_HIGH;
            pin2_value = PWM_LOW;
        }
    }

    if (motor == LEFT_MOTOR)
    {
        LEFT_ENABLE_PWM_PIN = enable_value;
        LEFT_PWM_PIN1 = pin1_value;
        LEFT_PWM_PIN2 = pin2_value;
    }
    else // motor == RIGHT_MOTOR
    {
        RIGHT_ENABLE_PWM_PIN = enable_value;
        RIGHT_PWM_PIN1 = pin1_value;
        RIGHT_PWM_PIN2 = pin2_value;
    }
}

void motor_ctrl_vel_diff(int32_t desired_vel, int32_t desired_diff, bool stop_mode, int32_t dt_micros)
{   
    // velocities
    int32_t r_vel = motor_ctrl_right_velocity;
    int32_t l_vel = motor_ctrl_left_velocity;

    // velocity errors
    int32_t error_left = desired_vel - l_vel + desired_diff;
    int32_t error_right = desired_vel - r_vel - desired_diff;

    // D components
    int32_t vel_d_left = 1000*(vel_p_left - error_left)/dt_micros;
    int32_t vel_d_right = 1000*(vel_p_right - error_right)/dt_micros;

    // P components
    vel_p_left = error_left;
    vel_p_right = error_right;

    // I components
    vel_i_left += vel_p_left * dt_micros;
    vel_i_right += vel_p_right * dt_micros;

    CLIP(vel_i_left, -50 / VELOCITY_I, 50 / VELOCITY_I);
    CLIP(vel_i_right, -50 / VELOCITY_I, 50 / VELOCITY_I);

    int32_t power_left = VELOCITY_P * vel_p_left + VELOCITY_I * vel_i_left + VELOCITY_D * vel_d_left;
    int32_t power_right = VELOCITY_P * vel_p_right + VELOCITY_I * vel_i_right + VELOCITY_D * vel_d_right;

    CLIP(power_left, -100, 100);
    CLIP(power_right, -100, 100);

    control_motor(LEFT_MOTOR, power_left, stop_mode);
    control_motor(RIGHT_MOTOR, power_right, stop_mode);
}

void read_velocities(uint32_t dt_micros)
{
    // get current tick count for both motors
    int64_t tick_count_left = get_tick_count_left();
    int64_t tick_count_right = get_tick_count_right();
    
    // calculate velocity in ticks per second by calculating delta_ticks / delta_t (sec)
    motor_ctrl_right_velocity = 1000000 * (tick_count_right - last_right_tick_count) / dt_micros;
    motor_ctrl_left_velocity = 1000000 * (tick_count_left - last_left_tick_count) / dt_micros;
    
    // save tick counts for next iteration
    last_left_tick_count = tick_count_left;
    last_right_tick_count = tick_count_right;
}

void balance_point_control(LSM6 *imu, int32_t desired_vel, int32_t desired_ang_vel, int32_t dt_micros)
{
    // get angle, angular velocity and velocity from IMU and encoders
    int32_t angle = cf_angle_x;
    int32_t ang_vel = imu->g.x - gyro_bias_x;
    int32_t vel = desired_vel - (motor_ctrl_right_velocity + motor_ctrl_left_velocity)/2;

    // low pass filter on given linear velocity
    vel_lpf = VEL_LPF_TC2 / (VEL_LPF_TC2 + dt_micros) * vel_lpf + dt_micros / (VEL_LPF_TC2 + dt_micros) * vel;

    bp_i += 0.00001 * vel;
    CLIP(bp_i, -90000, 90000);
    int32_t bp_angle_offset = 20.0 * vel + bp_i * dt_micros + (7500000.0 / dt_micros) * (vel_lpf - prev_vel);
    CLIP(bp_angle_offset, -90000, 90000);
    
    int32_t rising_angle_offset = ang_vel * ANGLE_RATE_RATIO + angle - BALANCE_ANGLE - bp_angle_offset;

    // add to power the tilt-angle element and the I element of the speed
    balance_point_power += (ANGLE_RESPONSE * rising_angle_offset) * dt_micros / 1000;
    CLIP(balance_point_power, -10000, 10000);

    // added a D element which eliminates oscillations
    float power = balance_point_power + (BP_SPEED_D  / dt_micros) * (vel_lpf - prev_vel);
    CLIP(power, -10000, 10000);

    // control motor velocities
    motor_ctrl_vel_diff(power, desired_ang_vel, STOP_MODE_BRAKE, dt_micros);

    prev_vel = vel_lpf;
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

    balance_point_control(imu, pos_output_vel_lpf, desired_ang_vel, dt);

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

    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
  
    vel_ctrl_timestamp = micros();
    balance_point_timestamp = micros();
    
    read_vel_once = false;
}
