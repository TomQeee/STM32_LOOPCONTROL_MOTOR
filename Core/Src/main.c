/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldc_openloop.h"
#include <stdio.h>
#include "as5600.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
// 提供 strlen 函数
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile float target_velocity = 3.0f;
volatile float current_speed = 0.0f;
char cmd_buf[20];
uint8_t cmd_idx = 0;
uint8_t rx_temp;
volatile uint8_t motor_running = 1;

#define SPEED_FILTER_ALPHA 0.02f   //一阶低通滤波系数 0.05~0.2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
OpenLoop_Init();
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

AS5600_ResetState(&hi2c1);

//==== 删除这里的 int g_sensor_direction / float g_zero_electric_angle！不要在这里赋值！====
//CalibrateMagnetZero(&hi2c1);

motor_running = 0;  //上电默认关闭闭环，等待R命令启动

HAL_TIM_Base_Start_IT(&htim4);

__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_NE);
HAL_UART_Receive_IT(&huart1, &rx_temp, 1);

Speed_Loop_ResetIntegral();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		
//老打印速度
//if(HAL_GetTick() - last_angle_time >= 50)   // 每 50ms 计算一次速度
//{
//    float angle_now = AS5600_ReadAngle(&hi2c1);
//    
//    // 计算角度差，处理 0/360 度过零跳变
//    float delta = angle_now - prev_angle;
//    if(delta > 180.0f)
//        delta -= 360.0f;
//    else if(delta < -180.0f)
//        delta += 360.0f;
//    
//    float dt = (float)(HAL_GetTick() - last_angle_time) / 1000.0f;  // 时间间隔(秒)
//    current_speed = (delta / dt) * (3.14159f / 180.0f);       // 度/秒 转换为 rad/s
//    
//    prev_angle = angle_now;
//    last_angle_time = HAL_GetTick();
//    
//    // 每 200ms 打印一次 (VOFA+ FireWater 格式)
//    static uint32_t last_print = 0;
//    if(HAL_GetTick() - last_print >= 100)
//    {
//       // printf("%.3f,%.3f\n", angle_now, current_speed);
//			  printf("%.3f,\n",-current_speed);
//        last_print = HAL_GetTick();
//    }
//}
//打印速度
    // 每100ms打印一次角度和速度（用于VOFA+观察）
//Test_VirtualOpenLoop(1.0f, 8.0f);

static uint32_t last_enc_tick = 0;
    if(HAL_GetTick() - last_enc_tick >= 2U)
    {
        AS5600_Task(&hi2c1);
        last_enc_tick = HAL_GetTick();
    }

    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print >= 20)
    {
        float mech_angle = AS5600_GetContinuousAngle_NoI2C();

        // Sine of the mechanical angle -> clean sine wave (1 cycle per revolution)
        float sin_mech = sinf(mech_angle);

        // Mechanical angle in degrees, wrapped to 0..360 (sawtooth)
        float ang_deg = mech_angle * 180.0f / 3.14159265f;
        float ang_mod = fmodf(ang_deg, 360.0f);
        if (ang_mod < 0.0f) ang_mod += 360.0f;

        // Speed in mechanical rad/s (same unit as the R/T commands)
        printf("%.3f,%.3f,%.3f,%.3f\n", sin_mech, ang_mod, current_speed, target_velocity);
        last_print = HAL_GetTick();
    }






		
		
		
		

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 1439;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//开环速度更新
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance == TIM4)
//    {
//        OpenLoop_Run(target_velocity);  // 使用全局变量
//    }
//}



//速度环更新
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM4)
//    {
//        static float prev_ang_isr = 0.0f;
//        float mech_ang_now = AS5600_GetContinuousAngle_NoI2C();
//        const float dt = 0.001f;

//        float speed_raw = (mech_ang_now - prev_ang_isr) / dt;
//        prev_ang_isr = mech_ang_now;

//        //一阶低通滤波
//        static float speed_filt = 0.0f;
//        speed_filt = speed_filt * (1.0f - SPEED_FILTER_ALPHA) + SPEED_FILTER_ALPHA * speed_raw;
//        current_speed = speed_filt;

//        if(motor_running == 1)
//        {
//            Speed_Loop_Run(target_velocity, current_speed, mech_ang_now);
//        }
//    }
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        static uint16_t loop_cnt = 0;
        static float prev_ang_isr = 0.0f;
        static float speed_filt = 0.0f;
        float mech_ang_now = AS5600_GetContinuousAngle_NoI2C();

        if(++loop_cnt >= 2)
        {
            loop_cnt = 0;
            const float dt = 0.002f;
            // Mechanical speed, flipped into the same frame as commutation
            float speed_mech = (mech_ang_now - prev_ang_isr) / dt * g_sensor_direction;
            prev_ang_isr = mech_ang_now;
            speed_filt = speed_filt * (1.0f - SPEED_FILTER_ALPHA) + SPEED_FILTER_ALPHA * speed_mech;
        }
        // current_speed：机械rad/s
        current_speed = speed_filt;

        if(motor_running == 1)
        {
            // target_velocity 本身就是你T命令设置的【机械rad/s】，直接丢进去，不乘方向！
            Speed_Loop_Run(target_velocity, current_speed, mech_ang_now);
        }
    }
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
        if(rx_temp == '\r' || rx_temp == '\n')
        {
            if(cmd_idx == 0) return;
            cmd_buf[cmd_idx] = '\0';

            if(cmd_buf[0] == 'H' || cmd_buf[0] == 'h')
            {
                const char *reply = "Hello! STM32 ready.\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)reply, strlen(reply), 100);
            }
            else if(cmd_buf[0] == 'T' || cmd_buf[0] == 't')
            {
                float new_speed = atof(&cmd_buf[1]);
                if(new_speed >= 0.0f && new_speed <= 30.0f)
                {
                    target_velocity = new_speed;
                    Speed_Loop_ResetIntegral();
                    char ack[40];
                    sprintf(ack, "Speed set to %.2f rad/s\r\n", new_speed);
                    HAL_UART_Transmit(&huart1, (uint8_t*)ack, strlen(ack), 100);
                }
                else
                {
                    const char *err = "Invalid speed! (0 ~ 30 rad/s)\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), 100);
                }
            }
            else if(cmd_buf[0] == 'S' || cmd_buf[0] == 's')
            {
                motor_running = 0;
                OpenLoop_SetZeroPwm();
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
                Speed_Loop_ResetIntegral();
                const char *reply = "STOP: motor disabled\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)reply, strlen(reply),100);
            }
            else if(cmd_buf[0] == 'R' || cmd_buf[0] == 'r')
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
                motor_running = 1;
                float new_spd = atof(&cmd_buf[1]);
                if(new_spd >= 0 && new_spd <= 30.0f)
                {
                    target_velocity = new_spd;
                }
                else
                {
                    target_velocity = 5.0f;
                }
                Speed_Loop_ResetIntegral();
                AS5600_ResetState(&hi2c1); //重启刷新编码器状态
                char ack[40];
                sprintf(ack,"RUN speed:%.2f rad/s\r\n",target_velocity);
                HAL_UART_Transmit(&huart1,(uint8_t*)ack,strlen(ack),100);
            }
            else if(cmd_buf[0] != '\0')
            {
                const char *unknown = "Unknown command.\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)unknown, strlen(unknown), 100);
            }
            cmd_idx = 0;
        }
        else if(cmd_idx < sizeof(cmd_buf) - 1)
        {
            cmd_buf[cmd_idx++] = (char)rx_temp;
        }
    }
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART1)
//    {
//        HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
//        HAL_UART_Transmit(&huart1, &rx_temp, 1, 10);
//    }
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
