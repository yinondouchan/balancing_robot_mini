#include <math.h>

#include "filters.h"

#define CF_CONST 0.99
#define GYRO_CALIBRATION_NSAMPLES 100

bool read_once;
long last_timestamp;

int32_t gyro_bias_x, gyro_bias_y, gyro_bias_z;

int32_t accel_angle_x;
int32_t accel_angle_y;
int32_t accel_angle_z;

int32_t cf_angle_x;
int32_t cf_angle_y;
int32_t cf_angle_z;

int32_t prev_cf_angle_x;
int32_t prev_cf_angle_y;
int32_t prev_cf_angle_z;

int32_t cf_ang_vel_x;
int32_t cf_ang_vel_y;
int32_t cf_ang_vel_z;

void calibrate_gyro(LSM6 *imu)
{
    int i=0;
    
    gyro_bias_x = 0;
    gyro_bias_y = 0;
    gyro_bias_z = 0;
    
    for (i = 0; i < GYRO_CALIBRATION_NSAMPLES; i++)
    {
        imu->read();
        gyro_bias_x += imu->g.x;
        gyro_bias_y += imu->g.y;
        gyro_bias_z += imu->g.z;
        delay(10);
    }
    
    gyro_bias_x /= GYRO_CALIBRATION_NSAMPLES;
    gyro_bias_y /= GYRO_CALIBRATION_NSAMPLES;
    gyro_bias_z /= GYRO_CALIBRATION_NSAMPLES;
}

void compl_filter_init()
{   
    cf_angle_x = 0;
    cf_angle_y = 0;
    cf_angle_z = 0;
    
    read_once = false;
}

void compl_filter_read(LSM6 *imu)
{
    // read accel and gyro
    imu->read();
    long now = millis();
    long dt = now - last_timestamp;
    last_timestamp = now;
    
    // make sure accel and gyro were read at least one time for being able to integrate
    if (!read_once)
    {
        read_once = true;
        return;
    }
    
    // calculate accel angle
    float angle_float_x = atan2(-imu->a_raw.y, imu->a_raw.z) * 180 / M_PI;
    float angle_float_y = atan2(-imu->a_raw.z, imu->a_raw.x) * 180 / M_PI;
    float angle_float_z = atan2(-imu->a_raw.y, imu->a_raw.x) * 180 / M_PI;
    accel_angle_x = angle_float_x * 1000;
    accel_angle_y = angle_float_y * 1000;
    accel_angle_z = angle_float_z * 1000;

    int32_t unbiased_gyro_x = imu->g.x - gyro_bias_x;
    int32_t unbiased_gyro_y = imu->g.y - gyro_bias_y;
    int32_t unbiased_gyro_z = imu->g.z - gyro_bias_z;
    
    // apply complementary filter
    cf_angle_x = CF_CONST*(cf_angle_x + unbiased_gyro_x*dt/1000) + (1.0 - CF_CONST)*accel_angle_x;
    cf_angle_y = CF_CONST*(cf_angle_y + unbiased_gyro_y*dt/1000) + (1.0 - CF_CONST)*accel_angle_y;
    cf_angle_z = CF_CONST*(cf_angle_z + unbiased_gyro_z*dt/1000) + (1.0 - CF_CONST)*accel_angle_z;
    
    cf_ang_vel_x = 1000 * (cf_angle_x - prev_cf_angle_x) / dt;
    cf_ang_vel_y = 1000 * (cf_angle_y - prev_cf_angle_y) / dt;
    cf_ang_vel_z = 1000 * (cf_angle_z - prev_cf_angle_z) / dt;
    
    prev_cf_angle_x = cf_angle_x;
    prev_cf_angle_y = cf_angle_y;
    prev_cf_angle_z = cf_angle_z;
}
