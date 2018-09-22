#ifndef COMPL_FILTER_H
#define COMPL_FILTER_H

#include "LSM6.h"

// init
void compl_filter_init();

// read angle by applying a complementary filter on accelerometer and gyro
// return result in deci-degrees
void compl_filter_read_angle(LSM6 *imu, int16_t *x, int16_t *y, int16_t *z);

#endif // COMPL_FILTER
