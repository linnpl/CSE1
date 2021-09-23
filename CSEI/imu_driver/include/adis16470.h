
#include <string>

#ifndef ADIS16470_DRIVER_H
#define ADIS16470_DRIVER_H

class Adis16470
{
public:
    int fd_;
    float gyro[3];
    float accl[3];
    float temp;

    Adis16470();
    int openPort(const std::string device);
    void closePort();
    int update();

};

#endif  // ADIS16470_DRIVER_H