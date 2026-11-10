#ifndef AS5600_H
#define AS5600_H

#include "main.h"

#define AS5600_ADDR  0x36
#define ANGLE_REG    0x0C

float AS5600_ReadAngle(I2C_HandleTypeDef *hi2c);

#endif