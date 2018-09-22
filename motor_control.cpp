#include "motor_control.h"
#include "hall_effect_sensor.h"

#define ZERO_MOTION_POWER 32

// left and right motor velocities in ticks per second
int motor_ctrl_right_velocity, motor_ctrl_left_velocity;

int last_left_tick_count, last_right_tick_count;
long vel_timestamp;
bool read_vel_once;

void angle_control(int16_t desired_angle, int16_t current_angle)
{
    int error = desired_angle - current_angle;
    int power = ANGLE_P * error;
    
    control_motor(LEFT_MOTOR, power);
    control_motor(RIGHT_MOTOR, power);
}

// control a motor with power from -100 to 100
void control_motor(uint8_t motor, uint8_t power)
{
    uint8_t dir = power < 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
    power = power < 0 ? -power : power;
    
    int pwm_power = ZERO_MOTION_POWER + (255 - ZERO_MOTION_POWER) * power / 100;
    uint8_t enable_pin = motor == LEFT_MOTOR ? LEFT_MOTOR_ENABLE_PIN : RIGHT_MOTOR_ENABLE_PIN;
    uint8_t pin1 = motor == LEFT_MOTOR ? LEFT_MOTOR_PIN1 : RIGHT_MOTOR_PIN1;
    uint8_t pin2 = motor == LEFT_MOTOR ? LEFT_MOTOR_PIN2 : RIGHT_MOTOR_PIN2;
    
    Serial.println(pwm_power);
    if(power == 0) pwm_power = 0;
    
    analogWrite(enable_pin, pwm_power);
    //analogWrite(MOTOR1_ENABLE_PIN, pwm_power);
    
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
    int tick_count_left = get_tick_count_left();
    int tick_count_right = get_tick_count_right();
    
    // calculate velocity in ticks per second by calculating delta_ticks / delta_t (sec)
    motor_ctrl_right_velocity = 1000 * (tick_count_right - last_right_tick_count) / dt;
    motor_ctrl_left_velocity = 1000 * (tick_count_left - last_left_tick_count) / dt;
    
    // save tick counts for next iteration
    last_left_tick_count = tick_count_left;
    last_right_tick_count = tick_count_right;
}

void init_motors()
{
  Serial.println("Initializing motor controls");
  
  motor_ctrl_right_velocity = 0;
  motor_ctrl_left_velocity = 0;
  last_left_tick_count = 0;
  last_right_tick_count = 0;
  
  read_vel_once = false;
  
  pinMode(LEFT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);

  pinMode(RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
}
