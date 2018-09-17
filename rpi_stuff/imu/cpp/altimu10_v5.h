#ifndef ALTIMU10_V5_h
#define ALTIMU10_V5_h

#include "LSM6/LSM6.h"

// complementary filter constant
#define CF_CONST 0.7

// complementary filter high pass and low pass filter time constant (seconds)
#define CF_HPF_LPF_CONST 0.2

class AltImu10V5
{
public:

    enum filter_mode {ACCEL_ONLY, GYRO_ONLY, COMPL_FILTER, KALMAN_FILTER};

    // raw data
    LSM6::vector<int16_t> accel_raw;
    LSM6::vector<int16_t> ang_vel_raw;

    // low pass filter on angle estimated from accelerometer
    LSM6::vector<float> accel_ang_lpf;

    // high pass filter on angle integrated from gyro output
    LSM6::vector<float> gyro_ang_hpf;

    // output data
    LSM6::vector<float> accel;
    LSM6::vector<float> ang_vel;

    LSM6::vector<float> ang_from_accel;   // angle derived from calculating gravity vector angle
    LSM6::vector<float> ang_from_gyro;    // angle derived from integrating gyro data assuming we start at angle 0

    // filtered angle
    LSM6::vector<float> filtered_angle;

    // constructor and destructor
    AltImu10V5(void);
    ~AltImu10V5(void);

    // setup
    void setup_i2c(void);

    void enable_accel_and_gyro(LSM6::accel_resolution accel_res, LSM6::gyro_resolution gyro_res);
    
    // read data and estimate angle
    void read_accel_gyro_and_angle(void);

    // set angle estimation filter mode
    void set_filter_mode(filter_mode filter_mode);

private:

    // file descriptors for the LSM6, LIS3MDL and LPS
    int lsm6_fd;
    int lis3mdl_fd;
    int lps_fd;

    // chip objects
    LSM6 *lsm6;     // gyro and accelerometer

    // output data filtering mode
    filter_mode filter_mode;

    // timestamp in milliseconds
    uint32_t timestamp;
    bool read_gyro_before;
};

#endif // ALTIMU10_V5_h
