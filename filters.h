#ifndef COMPL_FILTER_H
#define COMPL_FILTER_H

#include "LSM6.h"

// gyroscope biases
extern int32_t gyro_bias_x, gyro_bias_y, gyro_bias_z;

// output angles 
extern int32_t cf_angle_x;
extern int32_t cf_angle_y;
extern int32_t cf_angle_z;

// find the measurement biases of the gyroscope by averaging over samples
void calibrate_gyro(LSM6 *imu);

// init
void compl_filter_init();

// read gyro and accelerometer, apply complementary filter
// return angle and angular velocities in millideg and millideg/sec respectively
void compl_filter_read(LSM6 *imu, int32_t dt_micros);

#endif // COMPL_FILTER
