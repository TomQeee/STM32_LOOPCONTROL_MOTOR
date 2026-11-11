#ifndef BLDC_OPENLOOP_H
#define BLDC_OPENLOOP_H
#include "main.h"

#define POLE_PAIRS         7
#define VOLTAGE_POWER_SUPPLY  12.0f
#define VOLTAGE_LIMIT         2.0f

#define SPEED_KP  0.005f
#define SPEED_KI  0.001f

#define voltage_sensor_align 2.0f  //标定用电压，不要太大！

extern float g_zero_electric_angle;
extern int   g_sensor_direction; // +1 或者 -1
int alignSensor(I2C_HandleTypeDef *hi2c);
void OpenLoop_Init(void);
void OpenLoop_SetZeroPwm(void);
// mech_angle：外部传入已经算好的连续机械角rad
void OpenLoop_RunVoltage(float Uq, float mech_angle);
// 速度环：目标速度、当前滤波后速度、外部传入机械角
void Speed_Loop_Run(float target_speed, float current_speed, float mech_angle);
void Speed_Loop_ResetIntegral(void);
void Test_VirtualOpenLoop(float Uq, float elec_speed);
void CalibrateMagnetZero(I2C_HandleTypeDef *hi2c);
#endif