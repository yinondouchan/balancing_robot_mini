#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>

#include "LSM6.h"

// Constructors ////////////////////////////////////////////////////////////////

LSM6::LSM6(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

void LSM6::set_file_handle(int file_handle)
{
    this->file_handle = file_handle;
}

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LSM6::getTimeout()
{
  return io_timeout;
}

bool LSM6::init(deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for LSM6DS33 if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_DS33)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(DS33_SA0_HIGH_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        if (device == device_auto) { device = device_DS33; }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(DS33_SA0_LOW_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        if (device == device_auto) { device = device_DS33; }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_DS33:
      address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
Enables the LSM6's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
  and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
  which the electrical characteristics are specified in the datasheet.)
- Enables automatic increment of register address during multiple byte access
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM6::enableDefault(void)
{
    enable(LSM6::acc_2g, LSM6::gyro_245dps);
}


void LSM6::enable(accel_resolution a_res, gyro_resolution g_res)
{
  //if (_device == device_DS33)
  //{
    // Accelerometer
    
    // set accelerometer and gyroscope value ranges according to chosen resolutions
    accel_range = accel_values[a_res];
    gyro_range = gyro_values[g_res];

    uint8_t ctrl1_xl = 0x80;
    ctrl1_xl |= (uint8_t)a_res << 2;
    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    writeReg(CTRL1_XL, ctrl1_xl);

    // Gyro

    uint8_t ctrl2_g = 0x80;
    if (g_res == LSM6::gyro_125dps)
    {
        ctrl2_g |= 0x10; 
    }
    else
    {
        ctrl2_g |= (uint8_t)g_res << 2;
    }
    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    writeReg(CTRL2_G, ctrl2_g);

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    writeReg(CTRL3_C, 0x04);
  //}
}

void LSM6::writeReg(uint8_t reg, uint8_t value)
{
    int status;
    status = wiringPiI2CWriteReg8(file_handle, reg, value);
    if (status < 0)
    {
        printf("readReg: Error reading register %x. Returned status: %01x\n",
               reg, status);
    }
}

uint8_t LSM6::readReg(uint8_t reg)
{
    int value;
    value = wiringPiI2CReadReg8(file_handle, reg);
    if (value < 0)
    {
        printf("readReg: Error reading register %x. Returned value: %01x\n",
               reg, value);
    }

    return (uint8_t)value;
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6::readAcc(void)
{
    uint8_t xla = readReg(OUTX_L_XL);
    uint8_t xha = readReg(OUTX_H_XL);
    uint8_t yla = readReg(OUTY_L_XL);
    uint8_t yha = readReg(OUTY_H_XL);
    uint8_t zla = readReg(OUTZ_L_XL);
    uint8_t zha = readReg(OUTZ_H_XL);

    // combine high and low bytes
    a_raw.x = (int16_t)(xha << 8 | xla);
    a_raw.y = (int16_t)(yha << 8 | yla);
    a_raw.z = (int16_t)(zha << 8 | zla);

    // calculate accelerometer output in g units
    a.x = -accel_range * float(a_raw.x) / 32768;
    a.y = -accel_range * float(a_raw.y) / 32768;
    a.z = -accel_range * float(a_raw.z) / 32768;
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6::readGyro(void)
{
    uint8_t xlg = readReg(OUTX_L_G);
    uint8_t xhg = readReg(OUTX_H_G);
    uint8_t ylg = readReg(OUTY_L_G);
    uint8_t yhg = readReg(OUTY_H_G);
    uint8_t zlg = readReg(OUTZ_L_G);
    uint8_t zhg = readReg(OUTZ_H_G);

    // combine high and low bytes
    g_raw.x = (int16_t)(xhg << 8 | xlg);
    g_raw.y = (int16_t)(yhg << 8 | ylg);
    g_raw.z = (int16_t)(zhg << 8 | zlg);
    
    // calculate gyro output in deg/sec
    g.x = gyro_range * float(g_raw.x) / 32768;
    g.y = gyro_range * float(g_raw.y) / 32768;
    g.z = gyro_range * float(g_raw.z) / 32768;
}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void LSM6::read(void)
{
    readAcc();
    readGyro();
}

void LSM6::vector_normalize(vector<float> *a)
{
    float mag = sqrt(vector_dot(a, a));
    a->x /= mag;
    a->y /= mag;
    a->z /= mag;
}

// Test reading from a register.
// file_handle: wiringPi file handle
// reg: register address
int16_t LSM6::testReg(int file_handle, regAddr reg)
{
    int16_t read_val = wiringPiI2CReadReg16(file_handle, (uint8_t)reg);
    if (read_val < 0)
    {
        printf("testReg: Error reading register %x in address %x. Returned value: %01x\n",
               address, (uint8_t)reg, read_val);
    }

    return read_val;
}
