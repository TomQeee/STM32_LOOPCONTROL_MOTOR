#include "as5600.h"

float AS5600_ReadAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t reg = ANGLE_REG;
    uint8_t data[2] = {0};

    HAL_I2C_Master_Transmit(hi2c, (AS5600_ADDR << 1), &reg, 1, 100);
    HAL_I2C_Master_Receive(hi2c, (AS5600_ADDR << 1) | 0x01, data, 2, 100);

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    raw &= 0x0FFF;

    return (float)raw * 360.0f / 4096.0f;
}