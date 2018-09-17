#include "altimu10_v5.h"
#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

AltImu10V5::AltImu10V5(void)
{
    lsm6 = new LSM6();
    filter_mode = GYRO_ONLY;
    ang_from_gyro.x = 0;
    ang_from_gyro.y = 0;
    ang_from_gyro.z = 0;
    accel_ang_lpf.x = 0;
    accel_ang_lpf.y = 0;
    accel_ang_lpf.z = 0;
    gyro_ang_hpf.x = 0;
    gyro_ang_hpf.y = 0;
    gyro_ang_hpf.z = 0;
    filtered_angle.x = 0;
    filtered_angle.y = 0;
    filtered_angle.z = 0;
    read_gyro_before = FALSE;
}

AltImu10V5::~AltImu10V5(void)
{
    delete lsm6;
}

void AltImu10V5::set_filter_mode(enum filter_mode filter_mode)
{
    this->filter_mode = filter_mode;
}

void AltImu10V5::enable_accel_and_gyro(LSM6::accel_resolution accel_res, LSM6::gyro_resolution gyro_res)
{
    lsm6->enable(accel_res, gyro_res);
    //lsm6->enableDefault();
}

void AltImu10V5::read_accel_gyro_and_angle(void)
{
    uint32_t current_timestamp;
    uint32_t dt;

    // read accelerometer data in units of g
    lsm6->readAcc();
    accel = lsm6->a;
    accel_raw = lsm6->a_raw;

    // read gyro data in units of deg/sec
    lsm6->readGyro();
    current_timestamp = millis();
    dt = current_timestamp - timestamp;
    timestamp = current_timestamp;
    ang_vel = lsm6->g;
    ang_vel_raw = lsm6->g_raw;

    // angle based on accelerometer readings (degrees)
    ang_from_accel.x = atan2(-accel.y, accel.z) * 180 / M_PI;
    ang_from_accel.y = atan2(accel.z, accel.x) * 180 / M_PI;
    ang_from_accel.z = atan2(accel.y, accel.x) * 180 / M_PI;

    // filter angle using one of the modes 
    switch(filter_mode)
    {
        case ACCEL_ONLY:
            filtered_angle = ang_from_accel;
            break;
        case GYRO_ONLY:
            if (read_gyro_before)
            {
                // integrate only if gyro was read before in order to prevent very large dt and therefore crazy values
                // integrate angular velocity
                ang_from_gyro.x += dt * ang_vel.x / 1000;
                ang_from_gyro.y += dt * ang_vel.y / 1000;
                ang_from_gyro.z += dt * ang_vel.z / 1000;

                filtered_angle = ang_from_gyro;
            }
            filtered_angle = ang_from_gyro;
            break;
        case COMPL_FILTER:
            if (read_gyro_before)
            {
                float alpha = CF_HPF_LPF_CONST / (CF_HPF_LPF_CONST + dt / 1000.0);
                float prev_gyro_ang_x = ang_from_gyro.x;
                float prev_gyro_ang_y = ang_from_gyro.y;
                float prev_gyro_ang_z = ang_from_gyro.z;
                
                // integrate gyro
                ang_from_gyro.x += dt * ang_vel.x / 1000;
                ang_from_gyro.y += dt * ang_vel.y / 1000;
                ang_from_gyro.z += dt * ang_vel.z / 1000;

                // apply low pass filter on accelerometer
                accel_ang_lpf.x = alpha * accel_ang_lpf.x + (1 - alpha) * ang_from_accel.x;
                accel_ang_lpf.y = alpha * accel_ang_lpf.y + (1 - alpha) * ang_from_accel.y;
                accel_ang_lpf.z = alpha * accel_ang_lpf.z + (1 - alpha) * ang_from_accel.z;

                // apply high pass filter on gyro
                gyro_ang_hpf.x = (1 - alpha) * (gyro_ang_hpf.x + ang_from_gyro.x - prev_gyro_ang_x);
                gyro_ang_hpf.y = (1 - alpha) * (gyro_ang_hpf.y + ang_from_gyro.y - prev_gyro_ang_y);
                gyro_ang_hpf.z = (1 - alpha) * (gyro_ang_hpf.z + ang_from_gyro.z - prev_gyro_ang_z);

                // calculate complementary filter output angle
                /*filtered_angle.x = CF_CONST*(filtered_angle.x + ang_vel.x*dt / 1000) +
                                 (1 - CF_CONST)*ang_from_accel.x;
                filtered_angle.y = CF_CONST*(filtered_angle.y + ang_vel.y*dt / 1000) +
                                 (1 - CF_CONST)*ang_from_accel.y;
                filtered_angle.z = CF_CONST*(filtered_angle.z + ang_vel.z*dt / 1000) +
                                 (1 - CF_CONST)*ang_from_accel.z;*/
                filtered_angle.x = accel_ang_lpf.x + gyro_ang_hpf.x;
                filtered_angle.y = accel_ang_lpf.y + gyro_ang_hpf.y;
                filtered_angle.z = accel_ang_lpf.z + gyro_ang_hpf.z;
            }
            break;
        case KALMAN_FILTER:
            break;
    }

    read_gyro_before = TRUE;
}

void AltImu10V5::setup_i2c()
{
    // get file descriptors for LSM6, LIS3MDL and LPS
    lsm6_fd = wiringPiI2CSetup(DS33_SA0_HIGH_ADDRESS);
    lsm6->set_file_handle(lsm6_fd);
}

int main()
{
    AltImu10V5 altimu;
    altimu.setup_i2c();
    altimu.enable_accel_and_gyro(LSM6::acc_2g, LSM6::gyro_245dps);
    altimu.set_filter_mode(AltImu10V5::COMPL_FILTER);

    for (int i=0; i < 2000; i++)
    {
        altimu.read_accel_gyro_and_angle();
        delay(100);
        float a_mag = sqrt(altimu.accel.x*altimu.accel.x + altimu.accel.y*altimu.accel.y + altimu.accel.z*altimu.accel.z);

        // data in units
        printf("x: %f, y: %f, z: %f, gx: %f, gy: %f, gz: %f\n", altimu.accel.x, altimu.accel.y, altimu.accel.z,
               altimu.ang_vel.x, altimu.ang_vel.y, altimu.ang_vel.z);

        // raw data
        printf("x_acc: %f, x_cf: %f\n", altimu.ang_from_accel.x, altimu.filtered_angle.x);
        fflush(stdout);
    }

    return 0;
}
