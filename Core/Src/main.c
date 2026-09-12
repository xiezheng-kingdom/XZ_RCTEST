/* USER CODE BEGIN Header */
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "gimbal.h"
#include <stdio.h>

/* 二自由度云台控制板
 *
 *   输入: 2 个电位器 (PC2/PC3, ADC3) 或 MPU6050 (I2C2, PB10/PB11)
 *   切换: PE3 按键 (按下接地), 指示灯 PC5 (高电平亮 = MPU6050 模式)
 *   输出: 目标角度文本 "偏航,俯仰" → printf → USART3 (PD8), 115200
 *
 *   本步只算角度并上报, 不输出舵机 PWM。
 *   下一步接舵机时: 调 MX_TIM4_Init() (tim.c 里已配好 50Hz, PD14/PD15 两路)
 *   + HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3 / TIM_CHANNEL_4)。
 *
 *   注意: 控制节拍放在主循环而不是 SysTick 里 —— 一次 MPU6050 的 I2C 读
 *   约 1.5ms, 放进 1ms 的 SysTick 会把 HAL_IncTick() 饿死。
 */

void SystemClock_Config(void);
void MPU_Config(void);

int main(void)
{
  MPU_Config(); SCB_EnableICache(); SCB_EnableDCache(); HAL_Init(); SystemClock_Config();

  /* 只初始化本任务用到的外设。
   * MX_TIM1/TIM2/TIM3 是编码器与电机 PWM, 已随 motor 模块一起移除。 */
  MX_GPIO_Init(); MX_ADC3_Init(); MX_I2C2_Init();
  MX_USART2_UART_Init(); MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  Gimbal_Init();              /* 内含 MPU6050_Init(), 有约 1s 静止校准 */
  printf("GIMBAL READY\r\n"); /* 给上位机一个明确的起点 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_tick = 0;
    static uint32_t last_send = 0;

    if (HAL_GetTick() - last_tick >= GIMBAL_TICK_MS) {
      last_tick = HAL_GetTick();
      Gimbal_Update();                                   /* 按键 + 采样 + 算角度 */

      if (HAL_GetTick() - last_send >= GIMBAL_SEND_MS) {
        last_send = HAL_GetTick();
        Gimbal_Report();                                 /* 文本上报 */
      }
    }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef R={0}; RCC_ClkInitTypeDef C={0};
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)){}
  R.OscillatorType=RCC_OSCILLATORTYPE_HSE; R.HSEState=RCC_HSE_ON; R.PLL.PLLState=RCC_PLL_ON; R.PLL.PLLSource=RCC_PLLSOURCE_HSE;
  R.PLL.PLLM=2; R.PLL.PLLN=64; R.PLL.PLLP=2; R.PLL.PLLQ=2; R.PLL.PLLR=2; R.PLL.PLLRGE=RCC_PLL1VCIRANGE_3; R.PLL.PLLVCOSEL=RCC_PLL1VCOWIDE; R.PLL.PLLFRACN=0;
  if(HAL_RCC_OscConfig(&R)!=HAL_OK) Error_Handler();
  C.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  C.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; C.SYSCLKDivider=RCC_SYSCLK_DIV1; C.AHBCLKDivider=RCC_HCLK_DIV2;
  C.APB3CLKDivider=RCC_APB3_DIV2; C.APB1CLKDivider=RCC_APB1_DIV2; C.APB2CLKDivider=RCC_APB2_DIV2; C.APB4CLKDivider=RCC_APB4_DIV2;
  if(HAL_RCC_ClockConfig(&C,FLASH_LATENCY_2)!=HAL_OK) Error_Handler();
}
void MPU_Config(void) {
  MPU_Region_InitTypeDef M={0}; HAL_MPU_Disable();
  M.Enable=MPU_REGION_ENABLE; M.Number=MPU_REGION_NUMBER0; M.BaseAddress=0x0; M.Size=MPU_REGION_SIZE_4GB; M.SubRegionDisable=0x87;
  M.TypeExtField=MPU_TEX_LEVEL0; M.AccessPermission=MPU_REGION_NO_ACCESS; M.DisableExec=MPU_INSTRUCTION_ACCESS_DISABLE;
  M.IsShareable=MPU_ACCESS_SHAREABLE; M.IsCacheable=MPU_ACCESS_NOT_CACHEABLE; M.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&M); HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
void Error_Handler(void) { __disable_irq(); while(1){} }
