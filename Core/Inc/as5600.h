#ifndef AS5600_H
#define AS5600_H
#include "main.h"

#define AS5600_ADDR  0x36
#define ANGLE_REG    0x0C

// 原始读角度（度）
float AS5600_ReadAngle(I2C_HandleTypeDef *hi2c);
// 更新连续角度任务：【仅在主循环调用，禁止中断调用】
void AS5600_Task(I2C_HandleTypeDef *hi2c);
// 获取已经更新好的连续机械角(rad)，中断安全，只读全局变量
float AS5600_GetContinuousAngle_NoI2C(void);
// 重置编码器解绕状态（启动时调用）
void AS5600_ResetState(I2C_HandleTypeDef *hi2c);

#endif