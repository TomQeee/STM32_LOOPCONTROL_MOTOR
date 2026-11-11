#include "as5600.h"

static float angle_prev = 0.0f;          // 上一次原始角度（弧度）
static float shaft_angle_continuous = 0.0f; // 连续机械角（弧度，可累加）

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

// 读原始角度（度），带I2C错误返回 -1
static float AS5600_ReadRawDeg(I2C_HandleTypeDef *hi2c)
{
    uint8_t reg = ANGLE_REG;
    uint8_t data[2] = {0};
    if (HAL_I2C_Master_Transmit(hi2c, (AS5600_ADDR << 1), &reg, 1, 100) != HAL_OK)
        return -1.0f;
    if (HAL_I2C_Master_Receive(hi2c, (AS5600_ADDR << 1) | 0x01, data, 2, 100) != HAL_OK)
        return -1.0f;
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    raw &= 0x0FFF;
    return (float)raw * 360.0f / 4096.0f;
}

// 更新连续机械角【主循环调用】
void AS5600_Task(I2C_HandleTypeDef *hi2c)
{
    float deg = AS5600_ReadRawDeg(hi2c);
    if (deg < 0.0f) return; // I2C失败保持旧值

    float ang = deg * 3.14159265f / 180.0f; // 转弧度
    // 最短路径解绕
    float diff = ang - angle_prev;
    if (diff > 3.14159265f)       diff -= 6.2831853f;
    else if (diff < -3.14159265f) diff += 6.2831853f;

    shaft_angle_continuous += diff;
    angle_prev = ang;
}

// 只读全局变量，中断内安全，无I2C
float AS5600_GetContinuousAngle_NoI2C(void)
{
    return shaft_angle_continuous;
}

// 重置解绕状态，上电/R启动时调用
void AS5600_ResetState(I2C_HandleTypeDef *hi2c)
{
    float deg = AS5600_ReadRawDeg(hi2c);
    if(deg >= 0.0f)
    {
        angle_prev = deg * 3.14159265f / 180.0f;
        shaft_angle_continuous = angle_prev;
    }
}