/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "stdio.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}
/* USART2 init function (HC-06 蓝牙模块: PD5=TX, PD6=RX, 9600bps) */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;   /* HC06 用 */
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /*
   * ================================================================
   * USART2 BRR 注意事项（与 USART3 问题相同）
   * ================================================================
   *
   * HAL_UART_Init 调用 HAL_RCC_GetPCLK1Freq() 获取 PCLK1 频率，
   * 该函数在 STM32H743 上返回错误值（~500kHz），导致 BRR 计算错误。
   * 因此必须在此处用正确的 PCLK1 覆盖 BRR。
   *
   * BRR 随 SystemClock_Config 变化，见下表:
   *
   *   PCLK1      USARTDIV(9600)   BRR      来源
   *   ─────      ──────────────   ───      ────
   *   16MHz      104.17           0x0683   旧 (HSI 64MHz)
   *   32MHz      208.33           0x0D05   旧 (PLL 128MHz)
   *   100MHz     651.04           0x28B1   当前 (PLL 400MHz)
   *
   *   USARTDIV = PCLK1 / (16 × 9600)
   *   100e6 / (16×9600) = 651.04 → mant=651(0x28B) frac=1 → 0x28B1
   *   实际 = 100e6/(16×651.0625) = 9600.1 (误差 0.001%)
   *
   * ⚠️ 修改 SystemClock_Config 后必须重算！
   * ================================================================ */
  USART2->BRR = 0x364;   /* printf: 115200 @ 100MHz */

  /* 确保时钟使能 (APB1LENR bit17) */
  RCC->APB1LENR |= RCC_APB1LENR_USART2EN;

  /* 显式写配置寄存器，防范 HAL 未写全 */
  USART2->CR2 = 0;
  USART2->CR3 = 0;
  USART2->PRESC = 0;

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;   /* printf 用 */
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  /* ================================================================
   * USART3 初始化注意事项（踩坑记录）
   * ================================================================
   *
   * 问题1: 为什么不能调用 HAL_UART_Init()？
   * ----------------------------------------------------------------
   * HAL_UART_Init() 在 STM32H743 上会因 UART_CheckIdleState() 等待
   * TEACK/REACK 超时而返回 HAL_ERROR，导致进入 Error_Handler。
   * 因此采用裸寄存器写入方式，直接调用 HAL_UART_MspInit 完成
   * 时钟/GPIO/NVIC 配置后手动写 BRR/CR1/CR2/CR3/PRESC。
   *
   * 问题2: 波特率计算 —— 时钟频率是变化的！
   * ----------------------------------------------------------------
   * BRR 取决于 PCLK1 (APB1 总线时钟)，它受 SystemClock_Config 影响:
   *
   *   PCLK1 = SYSCLK / D2HPRE / D2PPRE1
   *
   * 不同项目/不同时钟源下 PCLK1 可能不同，BRR 必须随之改变。
   *
   *   时钟源        SYSCLK    D2HPRE  D2PPRE1   PCLK1     BRR(115200)
   *   ─────────    ──────    ──────  ───────   ─────     ──────────
   *   HSI 64MHz    64MHz     /2      /2        16MHz     0x08B
   *   HSE+PLL旧    128MHz    /2      /2        32MHz     0x116  (本注释曾误用)
   *   HSE+PLL当前  400MHz    /2      /2        100MHz    0x364  ← 当前
   *
   * BRR 公式 (OVER8=0):
   *   USARTDIV = PCLK1 / (16 × 波特率)
   *   BRR[15:4] = 整数部分
   *   BRR[3:0]  = round(小数部分 × 16)
   *
   * 当前值: PCLK1=100MHz, baud=115200
   *   USARTDIV = 100e6 / (16×115200) = 54.253
   *   mant=54(0x36), frac=round(0.253×16)=4 → BRR=0x364
   *
   * ⚠️ 修改 SystemClock_Config 后必须重新计算 BRR！
   *    建议通过 OLED 或串口打印 SystemCoreClock 和 D2CFGR，
   *    计算公式: PCLK1 = SystemCoreClock / D2HPRE_div / D2PPRE1_div
   *
   * 问题3: APB1LENR 中 USART3 时钟使能位是 bit 18，不是 bit 4！
   * ----------------------------------------------------------------
   * 不同 H7 子型号的 bit 位置不同，务必使用 HAL 宏:
   *   __HAL_RCC_USART3_CLK_ENABLE()  // 展开为 bit 18 (当前芯片)
   *   RCC_APB1LENR_USART3EN_Pos      // 宏定义位置号
   * 不要硬编码位号 (如 1UL<<4)，它会随芯片型号变化。
   *
   * 问题4: USART_ISR_TXFE vs USART_ISR_TXE_TXFNF
   * ----------------------------------------------------------------
   * TXFE(bit23) 仅在 FIFO 开启时有效。FIFO 关闭时必须用:
   *   USART_ISR_TXE_TXFNF  (bit 7, 发送数据寄存器空)
   * 否则 __io_putchar 会在 while(!TXFE) 死循环。
   *
   * 问题5: 完整寄存器初始化
   * ----------------------------------------------------------------
   * 除 BRR/CR1 外还必须显式配置 CR2/CR3/PRESC，避免残留值。
   * 寄存器写入顺序: 先关 UE → 写 BRR/CR1/CR2/CR3/PRESC → 再开 UE。
   * ================================================================ */
  HAL_UART_MspInit(&huart3);

  /* 确保 USART3 时钟使能 (APB1LENR bit18) — MspInit 已设，双保险 */
  RCC->APB1LENR |= RCC_APB1LENR_USART3EN;

  /* 禁用 USART 后才能修改寄存器 */
  USART3->CR1 &= ~USART_CR1_UE;

  /* BRR: 100MHz/115200 → USARTDIV=54.253 → 0x364
   *
   * ⚠️ 这里原本写的是 0x28B1 (9600bps), 是 HC06 还接在 USART3 的年代留下的。
   *    HC06 模块已删除, USART3 现在只用于 printf 和 VOFA 遥测, 必须按 115200 配。
   *
   *    9600 的后果不是报错而是"静默变慢": 一帧 6 float + 4 字节帧尾 = 28 字节,
   *    在 9600 下要 29ms 才发完 (HAL_UART_Transmit 超时给的 100ms, 所以不会
   *    报错, 只是把主循环拖到 ~29ms 一轮)。10ms 的控制周期被抽稀成 ~34Hz 采样,
   *    欠采样会把振荡采成假波形 —— 之前用 200ms 发送踩过同一个坑。
   *    115200 下 28 字节只需 2.4ms, 占用主循环 24%, 真正的 100Hz 全采样。 */
  USART3->BRR = 0x364;

  /* CR1: 8-bit, 无校验, 16x过采样, 禁止FIFO */
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE;

  /* CR2: 1停止位 */
  USART3->CR2 = 0;

  /* CR3: 禁用流控/DMA */
  USART3->CR3 = 0;

  /* PRESC: 内核时钟不分频 */
  USART3->PRESC = 0;

  /* 使能 USART */
  USART3->CR1 |= USART_CR1_UE;

  huart3.gState = HAL_UART_STATE_READY;
  huart3.RxState = HAL_UART_STATE_READY;
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART7 interrupt Init */
    HAL_NVIC_SetPriority(UART7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

    /* USART2 already configured shared USART28SEL clock source.
     * Only enable USART3 peripheral clock here. */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8);

    /* UART7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);  /* printf→PD8 */
    return ch;
}

/* USER CODE END 1 */

