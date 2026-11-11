#include "bldc_openloop.h"
#include "foc_utils.h"
#include "as5600.h"
#include <stdio.h>
#include <math.h>
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

#define PWM_PERIOD 1440.0f
static float speed_integral = 0.0f;
float g_zero_electric_angle = 0.8191f;
int   g_sensor_direction = -1;

void OpenLoop_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

// 重点修复Uq负数bug：负电压 → abs电压 + 电角度偏移PI
static void SetPhaseVoltage(float Uq, float angle_el)
{
    float uq_abs;
    float angle_el_local = angle_el; //已经外部算好最终电角度，不再加偏移！
    if(Uq < 0.0f)
    {
        uq_abs = -Uq;
        angle_el_local += _PI; //反转磁场
    }
    else
    {
        uq_abs = Uq;
    }
    float Uout = uq_abs / VOLTAGE_POWER_SUPPLY;
    //??? 这里不再 + ELECTRICAL_OFFSET！外部已经处理完zero_electric_angle
    angle_el_local = _normalizeAngle(angle_el_local);

    float Ta = 0.5f + 0.5f * _sin(angle_el_local);
    float Tb = 0.5f + 0.5f * _sin(angle_el_local - 2.094395102f);
    float Tc = 0.5f + 0.5f * _sin(angle_el_local + 2.094395102f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));
}

void OpenLoop_RunVoltage(float Uq, float raw_mech_angle)
{
    // raw_mech_angle：AS5600读到原始连续机械角
    float real_mech = g_sensor_direction * raw_mech_angle;
    float el_angle = real_mech * POLE_PAIRS;
    // 等价参考代码：el = raw*dir*pp ? zero_electric_angle
    float el_angle_final = _normalizeAngle( el_angle - g_zero_electric_angle );
    SetPhaseVoltage(Uq, el_angle_final);
}

void Speed_Loop_Run(float target_speed, float current_speed, float mech_angle)
{
    float error = target_speed - current_speed;
    float p_out = SPEED_KP * error;

    // ?正确PI：先累加积分，再积分限幅
    speed_integral += SPEED_KI * error;

    if (speed_integral > VOLTAGE_LIMIT)  speed_integral = VOLTAGE_LIMIT;
    if (speed_integral < -VOLTAGE_LIMIT) speed_integral = -VOLTAGE_LIMIT;

    float voltage_out = p_out + speed_integral;

    //输出限幅
    if (voltage_out > VOLTAGE_LIMIT)  voltage_out = VOLTAGE_LIMIT;
    if (voltage_out < -VOLTAGE_LIMIT) voltage_out = -VOLTAGE_LIMIT;

    OpenLoop_RunVoltage(voltage_out, mech_angle);
}

void Speed_Loop_ResetIntegral(void)
{
    speed_integral = 0.0f;
}

void OpenLoop_SetZeroPwm(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}


// 虚拟开环：不使用编码器，不使用PI，软件生成电角度
void Test_VirtualOpenLoop(float Uq, float elec_speed)
{
    static float el_angle = 0.0f;
    // 固定1ms步进，模拟TIM4 1ms周期
    el_angle += elec_speed * 0.001f;
    el_angle = _normalizeAngle(el_angle);

    float ang_local = _normalizeAngle(el_angle + _PI_2);
    float Uout = Uq / VOLTAGE_POWER_SUPPLY;

    float Ta = 0.5f + 0.5f * _sin(ang_local);
    float Tb = 0.5f + 0.5f * _sin(ang_local - 2.094395102f);
    float Tc = 0.5f + 0.5f * _sin(ang_local + 2.094395102f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));
}

// 返回值：1标定成功，0失败
//int alignSensor(I2C_HandleTypeDef *hi2c)
//{
//    int i;
//    float mid_angle, end_angle;
//    float moved;
//    printf("MOT: Align sensor start\r\n");
//    // 1.电角度向前转一个电周期
//    for(i=0; i<=500; i++)
//    {
//        float angle = _3PI_2 + _2PI * i / 500.0f;
//        //直接输出电压，不经过闭环
//        float ang_local = _normalizeAngle(angle);
//        float Uout = voltage_sensor_align / VOLTAGE_POWER_SUPPLY;
//        float Ta = 0.5f + 0.5f * _sin(ang_local) * Uout;
//        float Tb = 0.5f + 0.5f * _sin(ang_local - 2.094395102f) * Uout;
//        float Tc = 0.5f + 0.5f * _sin(ang_local + 2.094395102f) * Uout;
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));
//        HAL_Delay(2);
//        AS5600_Task(hi2c); //刷新编码器
//    }
//    mid_angle = AS5600_GetContinuousAngle_NoI2C();
//    //2.电角度向后转一个电周期
//    for(i=500; i>=0; i--)
//    {
//        float angle = _3PI_2 + _2PI * i / 500.0f;
//        float ang_local = _normalizeAngle(angle);
//        float Uout = voltage_sensor_align / VOLTAGE_POWER_SUPPLY;
//        float Ta = 0.5f + 0.5f * _sin(ang_local) * Uout;
//        float Tb = 0.5f + 0.5f * _sin(ang_local - 2.094395102f) * Uout;
//        float Tc = 0.5f + 0.5f * _sin(ang_local + 2.094395102f) * Uout;
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));
//        HAL_Delay(2);
//        AS5600_Task(hi2c);
//    }
//    end_angle = AS5600_GetContinuousAngle_NoI2C();
//    OpenLoop_SetZeroPwm();
//    HAL_Delay(200);
//    printf("mid_angle=%.4f, end_angle=%.4f\r\n",mid_angle,end_angle);
//    moved = fabsf(mid_angle - end_angle);
//    if( moved < 0.02f )
//    {
//        printf("MOT: sensor align fail, motor not moving!\r\n");
//        return 0;
//    }
//    //识别传感器方向
//    if(mid_angle < end_angle)
//    {
//        g_sensor_direction = 1;
//        printf("MOT: sensor_direction = +1 (CCW)\r\n");
//    }else{
//        g_sensor_direction = -1;
//        printf("MOT: sensor_direction = -1 (CW)\r\n");
//    }
//    //固定d轴电压，把转子吸附到A相磁极
//    {
//        float angle = _3PI_2;
//        float ang_local = _normalizeAngle(angle);
//        float Uout = voltage_sensor_align / VOLTAGE_POWER_SUPPLY;
//        float Ta = 0.5f + 0.5f * _sin(ang_local) * Uout;
//        float Tb = 0.5f + 0.5f * _sin(ang_local - 2.094395102f) * Uout;
//        float Tc = 0.5f + 0.5f * _sin(ang_local + 2.094395102f) * Uout;
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));
//    }
//    HAL_Delay(700);
//    AS5600_Task(hi2c);
//    float raw_mech_now = AS5600_GetContinuousAngle_NoI2C();
//    //计算zero_electric_angle，和参考代码公式完全一致
//    g_zero_electric_angle = _normalizeAngle( (g_sensor_direction * raw_mech_now) * POLE_PAIRS );
//    OpenLoop_SetZeroPwm();
//    HAL_Delay(200);
//    printf("MOT: zero_electric_angle = %.4f rad\r\n", g_zero_electric_angle);
//    return 1;
//}
void CalibrateMagnetZero(I2C_HandleTypeDef *hi2c)
{
    uint32_t t;
    for(t=0; t < 700U; t++)
    {
        float ang_local = _normalizeAngle(_3PI_2);
        float Uout = 2.0f / VOLTAGE_POWER_SUPPLY;
        float Ta = 0.5f + 0.5f * _sin(ang_local) * Uout;
        float Tb = 0.5f + 0.5f * _sin(ang_local - 2.094395102f) * Uout;
        float Tc = 0.5f + 0.5f * _sin(ang_local + 2.094395102f) * Uout;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));

        AS5600_Task(hi2c);
        HAL_Delay(1);
    }
    OpenLoop_SetZeroPwm();
    float raw_mech_now = AS5600_GetContinuousAngle_NoI2C();

    float zero_plus1  = _normalizeAngle(  1.0f * raw_mech_now * POLE_PAIRS );
    float zero_minus1 = _normalizeAngle( -1.0f * raw_mech_now * POLE_PAIRS );

    printf("\r\n====CALIB RESULT====\r\n");
    printf("raw_mech_now = %.4f\r\n", raw_mech_now);
    printf("g_sensor_direction= 1 , zero=%.4f\r\n", zero_plus1);
    printf("g_sensor_direction=-1 , zero=%.4f\r\n", zero_minus1);
    printf("====================\r\n");
}