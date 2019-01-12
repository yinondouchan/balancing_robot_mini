#include <Arduino.h>
#include <Wire.h>
#include "LSM6.h"
#include "filters.h"
#include "motor_control.h"
#include "hall_effect_sensor.h"
#include "bluetooth.h"

LSM6 imu;

int32_t x, y, z, wx, wy, wz, ctrl_vel, ctrl_pos;
uint8_t counter;
unsigned long timestamp, time_diff, prev_timestamp;

bool upright;
bool estop;
bool main_loop_triggered;

ISR(TIMER1_OVF_vect)
{
    // 8192 Hz frequency
    counter++;

    // main loop frequency: 256 Hz
    if (counter >= MAIN_LOOP_DIVISOR)
    {
        counter = 0;
    }
}

void on_estop()
{
    estop = !estop;
}

void setup_timers()
{
    cli();         // disable global interrupts
    
    // initialize Timer0
    TCCR0A = 0;    // set entire TCCR1A register to 0
    TCCR0B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    // Set to fast pwm mode (8-bit) with a prescaler of 8
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B |= (1 << CS01);


    // initialize Timer1
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    // Set to fast pwm mode (8-bit) with a prescaler of 8
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B |= (1 << CS11) | _BV(WGM12);

    // enable Timer1 overflow interrupt:
    TIMSK1 |= (1 << TOIE1);


    // initialize Timer2
    TCCR2A = 0;    // set entire TCCR1A register to 0
    TCCR2B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    // Set to fast pwm mode (8-bit) with a prescaler of 8
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
    TCCR2B |= (1 << CS21);
    
    // enable global interrupts:
    sei();
}

void setup()
{ 
    Serial.begin(9600);
    upright = false;
    estop = false;
    counter = 0;
    main_loop_triggered = false;
    setup_timers();
    init_motors();
    init_hall_effect_sensors();

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
    
    /*
    // serial and i2c
    Serial.begin(9600);
    
    timestamp = micros();*/
}

void loop()
{
    // triggered every 4096 microseconds (~250 Hz)
    if (!main_loop_triggered && (counter == 0))
    {
        uint8_t loop_start_counter = counter;

        // get motor velocities in ticks per second
        read_velocities(CYCLE_TIME_MICROS);
        
        compl_filter_read(&imu, CYCLE_TIME_MICROS);
        x = cf_angle_x;

        if ((x < -60000) || (x > 60000)) upright = false;

        // make sure the robot starts balancing itself only when it is upright. Also don't do anything if e-stop is on
        if (!upright || estop)
        {
            motor_ctrl_reset();
            control_motor(LEFT_MOTOR, 0, STOP_MODE_COAST);
            control_motor(RIGHT_MOTOR, 0, STOP_MODE_COAST);
            upright = (x > ((int32_t)BALANCE_ANGLE - 20000)) && (x < ((int32_t)BALANCE_ANGLE + 20000));
            return;
        }
        // balance the robot
        balance_point_control(&imu, bt_desired_vel, bt_desired_vel_diff, CYCLE_TIME_MICROS);

        bt_read_joystick_control();

        //if ((counter - prev_counter) > 1) Serial.println("Main loop missed a timer interrupt. Yikes!");
        main_loop_triggered = true;
    }
    else if (counter != 0)
    {
        main_loop_triggered = false;
    }
  /*unsigned long now = micros();
  unsigned long dt = now - timestamp;
  timestamp = now;

  // get tilt angle
  compl_filter_read(&imu);
  x = cf_angle_x;

  // get motor velocities in ticks per second
  read_velocities();
  
  if ((x < -60000) || (x > 60000)) upright = false;
  
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
  // balance the robot
  balance_point_control(&imu, bt_desired_vel, bt_desired_vel_diff);
  delay(5);*/
}
