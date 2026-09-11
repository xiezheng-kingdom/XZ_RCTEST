/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc3;

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  *
  * PC2 -> ADC3_INP0 (ADC_CHANNEL_0)
  * PC3 -> ADC3_INP1 (ADC_CHANNEL_1)
  */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Common config
     Note: ADC clock is synchronous from AHB clock, divided by 4.
           Keep ADC clock <= 50 MHz. */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /* PC2 -> ADC3_INP0 (rank 1) */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* PC3 -> ADC3_INP1 (rank 2) */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC self-calibration (offset) */
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /**ADC3 GPIO Configuration
    PC2     ------> ADC3_INP0
    PC3     ------> ADC3_INP1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if(adcHandle->Instance==ADC3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PC2     ------> ADC3_INP0
    PC3     ------> ADC3_INP1
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief  读取 ADC3 指定通道的采样值（阻塞式，软件触发单轮扫描）
 * @param  channel: ADC_CHANNEL_0 (PC2/INP0) 或 ADC_CHANNEL_1 (PC3/INP1)
 * @retval 12 位采样值 (0~4095)
 */
uint16_t ADC3_ReadChannel(uint32_t channel)
{
    /* 扫描顺序: rank1 = ADC_CHANNEL_0, rank2 = ADC_CHANNEL_1 */
    uint32_t rank = (channel == ADC_CHANNEL_0) ? 1u : 2u;

    HAL_ADC_Start(&hadc3);                     /* 软件触发，启动一轮扫描 */
    for (uint32_t i = 0; i < rank; i++) {
        HAL_ADC_PollForConversion(&hadc3, 10); /* 等待对应 rank 转换完成 */
    }
    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return value;
}

/**
 * @brief  读取 ADC3 指定通道的电压值
 * @param  channel: ADC_CHANNEL_0 (PC2) 或 ADC_CHANNEL_1 (PC3)
 * @retval 电压值，单位 V（范围 0 ~ ADC_REF_VOLTAGE）
 */
float ADC3_ReadVoltage(uint32_t channel)
{
    return (float)ADC3_ReadChannel(channel) * ADC_REF_VOLTAGE / 4095.0f;
}

/* USER CODE END 1 */
