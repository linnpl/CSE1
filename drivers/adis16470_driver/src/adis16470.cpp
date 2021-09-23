
#include <iostream>
#include <ostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include "adis16470_driver/adis16470.h"



/**
 * @brief Constructor
 */
Adis16470::Adis16470()
  : fd_(-1)
{
}

/**
 * @brief Open device
 * @param device Device file name (/dev/i2c-1)
 * @retval 0 Success
 * @retval -1 Failure
 */
int Adis16470::openPort(const std::string device)
{
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    perror("openPort");
    return -1;
  }
  ioctl(fd_,I2C_SLAVE, 0x36);

  return 0;
}

/**
 * @brief Closes port.. duh
 */
void Adis16470::closePort()
{
  close(fd_);
}

/**
 * @brief Updates Gyro, accl and temp values from imu.
 */
int Adis16470::update()
{
  unsigned char dataBuffer[21];

  read(fd_, dataBuffer, 21);
    gyro[0] = 0.1 * float(int16_t((dataBuffer[2] << 8) | dataBuffer[3]));
    gyro[1] = 0.1 * float(int16_t((dataBuffer[4] << 8) | dataBuffer[5]));
    gyro[2] = 0.1 * float(int16_t((dataBuffer[6] << 8) | dataBuffer[7]));
    accl[0] = 0.00125 * float(int16_t((dataBuffer[8] << 8) | dataBuffer[9]));
    accl[1] = 0.00125 * float(int16_t((dataBuffer[10] << 8) | dataBuffer[11]));
    accl[2] = 0.00125 * float(int16_t((dataBuffer[12] << 8) | dataBuffer[13]));
    temp = 0.1 * float(int16_t((dataBuffer[13] << 8) | dataBuffer[14]));
  return 0;
}