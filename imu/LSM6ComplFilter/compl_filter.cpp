#include <math.h>

#include "compl_filter.h"

#define CF_CONST 0.75

bool read_once;
long last_timestamp;

int16_t accel_angle_x;
int16_t accel_angle_y;
int16_t accel_angle_z;

int32_t cf_angle_x;
int32_t cf_angle_y;
int32_t cf_angle_z;

void compl_filter_init()
{   
    cf_angle_x = 0;
    cf_angle_y = 0;
    cf_angle_z = 0;
    
    read_once = false;
}

void compl_filter_read_angle(LSM6 *imu, int16_t *x, int16_t *y, int16_t *z)
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
    accel_angle_x = 10 * angle_float_x;
    accel_angle_y = 10 * angle_float_y;
    accel_angle_z = 10 * angle_float_z;
    
    // apply complementary filter
    cf_angle_x = (1 - CF_CONST)*(cf_angle_x + (int32_t)imu->g.x*dt) + CF_CONST*(int32_t)accel_angle_x*1000;
    cf_angle_y = (1 - CF_CONST)*(cf_angle_y + (int32_t)imu->g.y*dt) + CF_CONST*(int32_t)accel_angle_y*1000;
    cf_angle_z = (1 - CF_CONST)*(cf_angle_z + (int32_t)imu->g.z*dt) + CF_CONST*(int32_t)accel_angle_z*1000;
    
    *x = cf_angle_x/1000;
    *y = cf_angle_y/1000;
    *z = cf_angle_z/1000;
}
