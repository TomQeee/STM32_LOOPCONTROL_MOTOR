#include "bldc_openloop.h"
#include "foc_utils.h"

extern TIM_HandleTypeDef htim2;

#define PWM_PERIOD 1440.0f


static float shaft_angle = 0.0f;
static float speed_integral = 0.0f;  // 积分累计值

void OpenLoop_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

static void SetPhaseVoltage(float Uq, float angle_el)
{
    float Uout = Uq / VOLTAGE_POWER_SUPPLY;
    angle_el = _normalizeAngle(angle_el + _PI_2);

    float Ta = 0.5f + 0.5f * _sin(angle_el);
    float Tb = 0.5f + 0.5f * _sin(angle_el - 2.094395102f); // -120°
    float Tc = 0.5f + 0.5f * _sin(angle_el + 2.094395102f); // +120°

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(Ta * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(Tb * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(Tc * PWM_PERIOD));
}

void OpenLoop_Run(float target_velocity)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) * 0.001f;
    if(dt <= 0.0f) dt = 0.001f;
    last_tick = now;

    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * dt);

    float el_angle = shaft_angle * POLE_PAIRS;
    SetPhaseVoltage(VOLTAGE_LIMIT, el_angle);
}

void OpenLoop_RunVoltage(float Uq, float target_velocity)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) * 0.001f;
    if(dt <= 0.0f) dt = 0.001f;
    last_tick = now;

    // 用目标速度推进电角度（保证旋转方向正确）
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * dt);

    float el_angle = shaft_angle * POLE_PAIRS;
    SetPhaseVoltage(Uq, el_angle);   // 使用传入的电压，而不是固定 VOLTAGE_LIMIT
}





void Speed_Loop_Run(float target_speed, float current_speed)
{
    float error = target_speed - current_speed;
    
    // 比例项
    float p_out = SPEED_KP * error;
    
    // 积分项（带抗饱和：当输出达到限幅时，停止正向积分）
    float output_before_limit = p_out + speed_integral;
    if(output_before_limit < VOLTAGE_LIMIT && output_before_limit > 0.0f)
    {
        speed_integral += SPEED_KI * error;
    }
    // 积分限幅到电压范围
    if(speed_integral > VOLTAGE_LIMIT)  speed_integral = VOLTAGE_LIMIT;
    if(speed_integral < -VOLTAGE_LIMIT) speed_integral = -VOLTAGE_LIMIT;
    
    // 总输出 = 电压指令，限幅到 [0, VOLTAGE_LIMIT]（单象限，只正转）
    float voltage_out = p_out + speed_integral;
    if(voltage_out > VOLTAGE_LIMIT)  voltage_out = VOLTAGE_LIMIT;
    if(voltage_out < 0.0f)           voltage_out = 0.0f;
    
    // 使用电压驱动，同时传递目标速度用于角度推进
    OpenLoop_RunVoltage(voltage_out, target_speed);
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

