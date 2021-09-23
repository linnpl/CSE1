#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef MX106_DRIVER_H
#define MX106_DRIVER_H


class Mx106
{
public:

    Mx106();
    int initServos(uint8_t ids[], int numberOfServos);
    int16_t getPresentPosition(uint8_t id);
    void getPresentPositions(uint8_t id[], int16_t positions[]);
    void setPosition(int16_t position, uint8_t id);
    void setPositions(int16_t positions[], uint8_t ids[]);
};

#endif  // MX106_DRIVER_H