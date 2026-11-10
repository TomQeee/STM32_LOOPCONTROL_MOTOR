#ifndef BLDC_OPENLOOP_H
#define BLDC_OPENLOOP_H

#include "main.h"

#define POLE_PAIRS         7
#define VOLTAGE_POWER_SUPPLY  12.0f
#define VOLTAGE_LIMIT         2.0f
#define SPEED_KP  0.2f   // 比例系数
#define SPEED_KI  0.05f   // 积分系数


void OpenLoop_Init(void);
void OpenLoop_Run(float target_velocity);
void Speed_Loop_Run(float target_speed, float current_speed);
void Speed_Loop_ResetIntegral(void);
void OpenLoop_RunVoltage(float Uq, float target_velocity);  // 新增
void OpenLoop_SetZeroPwm(void);
#endif