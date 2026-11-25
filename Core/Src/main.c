/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "oled_ssd1306.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 减速比 9.6，基础脉冲 11PPR，4倍频 -> 11 * 4 * 9.6 = 422.4
#define ENCODER_PPR_MOTOR_REV 11.0f /* datasheet: 11PPR per motor revolution */
#define GEAR_RATIO 9.6f
#define ENCODER_COUNTS_PER_OUTPUT_REV (ENCODER_PPR_MOTOR_REV * 4.0f * GEAR_RATIO)
#define SPEED_SAMPLE_MS 200U
#define SPEED_STEP_RPM 50.0f
#define MAX_TARGET_RPM 625.0f /* 6000 rpm motor / 9.6 gearbox */
#define PWM_MIN 10U /* driver minimum allowed duty: 10% */
#define PWM_MAX 99U  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t key1_flag = 0;
uint8_t key2_flag = 0;

// --- 测速与PID相关变量 ---
uint16_t last_encoder_cnt = 0;  // 上一次的编码器值
float measured_rpm = 0.0f;     // 实测转速
float target_rpm = 0.0f;       // 目标转速
float pwm_output = 0.0f;       // PID计算出的PWM输出

// --- PID 参数  ---
float Kp = 0.12f;
float Ki = 0.08f;
float integral = 0.0f;
float filtered_rpm = 0.0f;
uint32_t motor_start_tick = 0;   

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Motor_SetSpeed(uint16_t duty);
void VOFA_SendSpeed(float actual, float target);
float PID_Calc(float target, float actual);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 自定义串口打印 (替代 printf)
void VOFA_SendSpeed(float actual, float target)
{
    static uint8_t packet[12];
    float values[2];
    uint32_t marker = 0x7F800000U;
    if (huart1.gState != HAL_UART_STATE_READY) return;
    values[0] = (float)((int32_t)(actual * 10.0f + 0.5f)) / 10.0f;
    values[1] = (float)((int32_t)(target * 10.0f + 0.5f)) / 10.0f;
    memcpy(packet, values, 8);
    memcpy(&packet[8], &marker, 4);
    HAL_UART_Transmit_DMA(&huart1, packet, sizeof(packet));
}

// 电机速度控制函数
void Motor_SetSpeed(uint16_t duty)
{
    if (duty > PWM_MAX) duty = PWM_MAX;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty);
}

// 增量式 PID 控制函数
float PID_Calc(float target, float actual)
{
    float error = target - actual;
    float feed_forward = 10.0f + 0.10f * target;
    integral += Ki * error * ((float)SPEED_SAMPLE_MS / 1000.0f);
    if (integral > 35.0f) integral = 35.0f;
    if (integral < -20.0f) integral = -20.0f;
    float out = feed_forward + Kp * error + integral;
    if (out > PWM_MAX) out = PWM_MAX;
    if (out < PWM_MIN) out = PWM_MIN;
    return out;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  OLED_Init();

  /* USER CODE BEGIN 2 */
  // 1. 启动 PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  
  // 2. 启动编码器
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  last_encoder_cnt = 0;
  
  // 3. 初始化电机停止
  Motor_SetSpeed(0);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  uint32_t last_sample_tick = HAL_GetTick();
  while (1)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
      if (!key1_flag) { HAL_Delay(20); if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) { key1_flag=1; target_rpm+=SPEED_STEP_RPM; if(target_rpm>MAX_TARGET_RPM) target_rpm=MAX_TARGET_RPM; HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0); } }
    } else key1_flag=0;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) {
      if (!key2_flag) { HAL_Delay(20); if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) { key2_flag=1; target_rpm-=SPEED_STEP_RPM; if(target_rpm<0.0f) target_rpm=0.0f; } }
    } else key2_flag=0;
    if ((HAL_GetTick()-last_sample_tick)>=SPEED_SAMPLE_MS) {
      last_sample_tick+=SPEED_SAMPLE_MS;
      uint16_t current_cnt=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
      int16_t diff_cnt=(int16_t)(current_cnt-last_encoder_cnt); last_encoder_cnt=current_cnt;
      measured_rpm=((float)diff_cnt*60000.0f)/(ENCODER_COUNTS_PER_OUTPUT_REV*SPEED_SAMPLE_MS);
      if(measured_rpm<0) measured_rpm=-measured_rpm;
      filtered_rpm=0.55f*measured_rpm+0.45f*filtered_rpm;
      if(target_rpm>0.0f){ if(motor_start_tick==0U) motor_start_tick=HAL_GetTick(); float new_pwm=PID_Calc(target_rpm,filtered_rpm); if((HAL_GetTick()-motor_start_tick)<300U&&new_pwm<25.0f)new_pwm=25.0f; pwm_output=new_pwm; Motor_SetSpeed((uint16_t)new_pwm); }
      else { Motor_SetSpeed(0); integral=0; filtered_rpm=0; motor_start_tick=0; pwm_output=0; }
      VOFA_SendSpeed(measured_rpm, target_rpm);
      OLED_ShowStatus(target_rpm,measured_rpm,pwm_output);
    }
    HAL_Delay(5);
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

// TIM2 PWM 初始化
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  
  HAL_TIM_MspPostInit(&htim2);
}

// TIM3 编码器初始化
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) { Error_Handler(); }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

// USART1 初始化
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

// GPIO 初始化
static void MX_I2C1_Init(void)
{
  hi2c1.Instance=I2C1; hi2c1.Init.ClockSpeed=100000; hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1=0; hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2=0;
  hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  if(HAL_I2C_Init(&hi2c1)!=HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
	};
}
