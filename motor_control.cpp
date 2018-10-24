#include "motor_control.h"
#include "hall_effect_sensor.h"

#define CLIP(x, val_low, val_high) x = x > val_high ? val_high : x < val_low ? val_low : x;

// left and right motor velocities in ticks per second
int32_t motor_ctrl_right_velocity, motor_ctrl_left_velocity;

int32_t last_left_tick_count, last_right_tick_count;

int32_t vel_p_left, vel_p_right;
int32_t vel_i_left, vel_i_right;
int32_t prev_vel;
int32_t prev_angle;
float vel_lpf;
unsigned long vel_timestamp;
unsigned long vel_ctrl_timestamp;
bool read_vel_ctrl_once;
bool read_bp_once;
bool read_vel_once;

float balance_point_power, prev_bp_power;
unsigned long balance_point_timestamp;

// control a motor with power from -100 to 100
void control_motor(uint8_t motor, int8_t power)
{
    uint8_t dir = power < 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
    power = power < 0 ? -power : power;
    
    int pwm_power = ZERO_MOTION_POWER + (255 - ZERO_MOTION_POWER) * power / 100;
    uint8_t enable_pin = motor == LEFT_MOTOR ? LEFT_MOTOR_ENABLE_PIN : RIGHT_MOTOR_ENABLE_PIN;
    uint8_t pin1 = motor == LEFT_MOTOR ? LEFT_MOTOR_PIN1 : RIGHT_MOTOR_PIN1;
    uint8_t pin2 = motor == LEFT_MOTOR ? LEFT_MOTOR_PIN2 : RIGHT_MOTOR_PIN2;
    
    if(power == 0) pwm_power = 0;
    
    analogWrite(enable_pin, pwm_power);
    
    if(dir == MOTOR_BACKWARD)
    {
        digitalWrite(pin2, HIGH);
        digitalWrite(pin1, LOW);
    }
    else // MOTOR_FORWARD
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
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

    CLIP(vel_i_left, -100 / VELOCITY_I, 100 / VELOCITY_I);
    CLIP(vel_i_right, -100 / VELOCITY_I, 100 / VELOCITY_I);

    int32_t power_left = VELOCITY_P * vel_p_left + VELOCITY_I * vel_i_left + VELOCITY_D * vel_d_left;
    int32_t power_right = VELOCITY_P * vel_p_right + VELOCITY_I * vel_i_right + VELOCITY_D * vel_d_right;

    CLIP(power_left, -100, 100);
    CLIP(power_right, -100, 100);

    control_motor_phase_en(LEFT_MOTOR, power_left, stop_mode);
    control_motor_phase_en(RIGHT_MOTOR, power_right, stop_mode);
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

void balance_point_control(int32_t angle, int32_t ang_vel, int32_t vel, int32_t vel_diff, int32_t pos)
{
    unsigned long dt = micros() - balance_point_timestamp;
    balance_point_timestamp = micros();

    if (!read_bp_once)
    {
        read_bp_once = true;
        return;
    }

    vel_lpf = VEL_LPF_TC / (VEL_LPF_TC + dt) * vel_lpf + dt / (VEL_LPF_TC + dt) * vel;
    
    int32_t balance_angle = angle - BALANCE_ANGLE;
    //bool falling = ((balance_angle >= 0) && (ang_vel >= 0)) || ((balance_angle < 0) && (ang_vel < 0));
    
    int32_t rising_angle_offset = ang_vel * ANGLE_RATE_RATIO + angle - BALANCE_ANGLE;

    balance_point_power += (ANGLE_RESPONSE * rising_angle_offset + SPEED_RESPONSE * vel) * dt / 1000;
    CLIP(balance_point_power, -10000, 10000);

    float power = balance_point_power + 1.2 * vel_lpf - (100000.0  / dt) * (vel_lpf - prev_vel);
    CLIP(power, -10000, 10000);

    motor_ctrl_vel_diff(power, vel_diff, STOP_MODE_COAST);
    motor_ctrl_vel_diff(power, vel_diff, STOP_MODE_COAST);

    prev_vel = vel_lpf;
    //prev_angle = balance_angle;
}

void position_control(int32_t angle, int32_t ang_vel, int32_t max_vel, int32_t vel_diff, int32_t pos)
{
    int32_t output_vel = pos;
    CLIP(output_vel, -max_vel, max_vel);
    
    balance_point_control(angle, ang_vel, output_vel, vel_diff, pos);
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
  vel_lpf = 0;
  read_vel_ctrl_once = false;
  read_bp_once = false;
  
  vel_ctrl_timestamp = micros();
  balance_point_timestamp = micros();
  
  read_vel_once = false;
}
