#include "motor_control.h"
#include "hall_effect_sensor.h"

#define ZERO_MOTION_POWER 0
#define BALANCE_ANGLE -12500  // measured angle where robot is balanced

#define CLIP(x, val_low, val_high) x = x > val_high ? val_high : x < val_low ? val_low : x;

// left and right motor velocities in ticks per second
int motor_ctrl_right_velocity, motor_ctrl_left_velocity;

int last_left_tick_count, last_right_tick_count;
long vel_timestamp;
long vel_ctrl_timestamp;
float vel_ctrl_error_sum;
long angle_timestamp;
bool read_vel_once;

bool read_angle_once;
float prev_error;
float error_sum;
float balance_point;

float balance_point_power;

int16_t prev_vel_error;
int16_t prev_desired_angle;

char msg[100];

void velocity_control(int32_t desired_vel, int32_t current_vel, int32_t current_angle)
{
    long now = millis();
    long dt = now - vel_ctrl_timestamp;
    vel_ctrl_timestamp = now;
    
    /*boolean within_setpoint = ((prev_desired_angle - current_angle) <= 30) && ((prev_desired_angle - current_angle) >= -30);
    if (!within_setpoint)
    {
        angle_control(prev_desired_angle, current_angle);
        return;
    }*/
  
    int16_t vel_error = desired_vel - current_vel;
    vel_ctrl_error_sum += vel_error * dt;
    
    // anti integral-windup
    vel_ctrl_error_sum = vel_ctrl_error_sum > 10000 / VELOCITY_I ? 10000 / VELOCITY_I : vel_ctrl_error_sum;
    vel_ctrl_error_sum = vel_ctrl_error_sum < -10000 / VELOCITY_I ? -10000  / VELOCITY_I : vel_ctrl_error_sum;
    
    int16_t desired_angle = VELOCITY_P * vel_error + VELOCITY_I * vel_ctrl_error_sum + VELOCITY_D * (vel_error - prev_vel_error) + BALANCE_ANGLE;
    prev_desired_angle = desired_angle;
    angle_control(desired_angle, current_angle);
    Serial.print(desired_angle);
    Serial.print(" ");
    Serial.println(current_angle);
    
    prev_vel_error = vel_error;
}

void angle_control(int32_t desired_angle, int32_t current_angle)
{;
    // P component
    int32_t error = desired_angle - current_angle;
  
    long now = millis();
    long dt = now - angle_timestamp;
    angle_timestamp = now;
    if (!read_angle_once)
    {
        read_angle_once = true;
        return;
    }
    
    // I component
    error_sum += error * dt;
    
    // clip I in order to prevent integral windup
    error_sum = error_sum > 100 / ANGLE_I ? 100 / ANGLE_I : error_sum;
    error_sum = error_sum < -100 / ANGLE_I ? -100 / ANGLE_I : error_sum;
    
    // D component
    int32_t error_diff = (error - prev_error) / dt;
    
    int32_t power = ANGLE_P * -error + ANGLE_I * -error_sum + ANGLE_D * -error_diff + balance_point;
    power = power > 100 ? 100 : power;
    power = power < -100 ? -100 : power;
    
    control_motor(LEFT_MOTOR, power);
    control_motor(RIGHT_MOTOR, power);
    
    //Serial.print(desired_angle); Serial.print(" "); Serial.print(current_angle); Serial.print(" "); Serial.println(ANGLE_D * -error_diff);
    
    prev_error = error;
}

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

void read_velocities()
{
    // measure time delta from last read
    long now = millis();
    long dt = now - vel_timestamp;
    vel_timestamp = now;
    
    if (!read_vel_once)
    {
        read_vel_once = true;
        return;
    }
    
    // get current tick count for both motors
    int32_t tick_count_left = get_tick_count_left();
    int32_t tick_count_right = get_tick_count_right();
    
    // calculate velocity in ticks per second by calculating delta_ticks / delta_t (sec)
    motor_ctrl_right_velocity = 1000 * (tick_count_right - last_right_tick_count) / dt;
    motor_ctrl_left_velocity = 1000 * (tick_count_left - last_left_tick_count) / dt;
    
    // save tick counts for next iteration
    last_left_tick_count = tick_count_left;
    last_right_tick_count = tick_count_right;
}

void balance_point_control(int32_t angle, int32_t ang_vel, int32_t vel)
{
    int32_t rising_angle_offset = ang_vel * ANGLE_RATE_RATIO + angle - BALANCE_ANGLE;

    balance_point_power += ANGLE_RESPONSE * rising_angle_offset + SPEED_RESPONSE * vel;
    CLIP(balance_point_power, -100, 100);
    
    control_motor(LEFT_MOTOR, balance_point_power);
    control_motor(RIGHT_MOTOR, balance_point_power);
}

void init_motors()
{
  Serial.println("Initializing motor controls");
  
  motor_ctrl_right_velocity = 0;
  motor_ctrl_left_velocity = 0;
  last_left_tick_count = 0;
  last_right_tick_count = 0;
  prev_error = 0.0;
  prev_vel_error = 0.0;
  prev_desired_angle = BALANCE_ANGLE;
  error_sum = 0.0;
  vel_ctrl_error_sum = 0.0;
  balance_point = 0.0;
  balance_point_power = 0.0;
  
  read_vel_once = false;
  read_angle_once = false;
  
  pinMode(LEFT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);

  pinMode(RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
}
