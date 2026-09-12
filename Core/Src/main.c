/* USER CODE BEGIN Header */
/************************************************************************************************
* 程序版本：V1.0 (完美修复版)
* 修复内容：补齐大括号、修复死循环逻辑、重构上位机3参数解析
************************************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------------- VOFA+ JustFloat 协议 ---------------- */
#define CH_COUNT 2                      /* 通道数：占空比(%) + 当前角度(°) */
typedef struct {
    float         fdata[CH_COUNT];
    unsigned char tail[4];              /* 00 00 80 7F = +Inf，帧尾标识 */
} VofaFrame;

/* 结构体必须紧凑成 4*CH_COUNT+4 字节，否则帧尾位置会错、VOFA+ 解析不出 */
_Static_assert(sizeof(VofaFrame) == sizeof(float) * CH_COUNT + 4U,
               "VofaFrame has unexpected padding");

/* ---------------- 舵机参数 (PD15 = TIM4_CH4) ---------------- */
#define SERVO_ANGLE_MIN   (-90.0f)      /* 最小角，对应 0.5ms 脉宽 */
#define SERVO_ANGLE_MAX   ( 90.0f)      /* 最大角，对应 2.5ms 脉宽 */
#define SERVO_PWM_ARR     (20000.0f)    /* ARR+1，用于算占空比 */
#define SERVO_SLEW_STEP   (1.0f)        /* 每周期转过多少度 -> 50°/s */
#define CTRL_PERIOD_MS    (20U)         /* 主循环周期，与 PWM 周期一致 */

uint8_t PC_RXbuff[50];    // 上位机接收缓冲区 (USART2)
uint8_t Car_RXbuff[50];   // 小车接收缓冲区 (UART5)

/* VOFA+ 下发的目标角度：中断里收行，主循环里解析 */
static volatile uint8_t rx_line_ready = 0;
static char             rx_line[32];

/* 舵机状态 */
static float target_angle  = 0.0f;      /* VOFA+ 下发的目标角度 */
static float current_angle = 0.0f;      /* 当前输出角度，按斜坡跟随目标 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  角度 -> 定时器比较值
  * @note   与 servo.c 的 translate_angle_to_pulse 同一套约定：
  *         -90° -> 0.5ms(500)，0° -> 1.5ms(1500)，+90° -> 2.5ms(2500)
  *         1MHz 计数、20ms 周期，故脉宽数值即微秒数
  */
static uint32_t servo_angle_to_pulse(float angle)
{
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
    return (uint32_t)(((angle + 90.0f) / 90.0f + 0.5f) * 1000.0f);
}

/**
  * @brief  角度 -> 占空比百分比
  * @note   脉宽 500~2500 / 周期 20000 -> 2.5% ~ 12.5%
  */
static float servo_angle_to_duty(float angle)
{
    return (float)servo_angle_to_pulse(angle) / SERVO_PWM_ARR * 100.0f;
}

/**
  * @brief  向 VOFA+ 回传一帧 JustFloat 数据
  * @note   帧结构：float fdata[2] + 00 00 80 7F，共 12 字节，小端
  */
static void vofa_send(float duty, float angle)
{
    VofaFrame frame;

    frame.fdata[0] = duty;
    frame.fdata[1] = angle;
    frame.tail[0] = 0x00;
    frame.tail[1] = 0x00;
    frame.tail[2] = 0x80;
    frame.tail[3] = 0x7f;

    HAL_UART_Transmit(&huart2, (uint8_t *)&frame, sizeof(frame), 20);
}

/**
  * @brief  串口空闲中断回调：收全一行就置标志，解析交给主循环
  * @note   中断里不做浮点解析，也不打印，避免阻塞
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        uint16_t n = Size;
        if (n > sizeof(rx_line) - 1U) n = sizeof(rx_line) - 1U;
        memcpy(rx_line, PC_RXbuff, n);
        rx_line[n] = '\0';
        rx_line_ready = 1;
        /* 重新武装，否则只能收到第一帧 */
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, PC_RXbuff, sizeof(PC_RXbuff));
    }
    else if (huart->Instance == UART5) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, Car_RXbuff, sizeof(Car_RXbuff));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)      HAL_UARTEx_ReceiveToIdle_IT(&huart2, PC_RXbuff, sizeof(PC_RXbuff));
    else if (huart->Instance == UART5)  HAL_UARTEx_ReceiveToIdle_IT(&huart5, Car_RXbuff, sizeof(Car_RXbuff));
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_UART7_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  /* 不再调用 servo.c 的 pwm_start()/servo_reset_begin()：
     servo.c 会往 TIM4_CH4 (PD15) 写 pulse_6，与本文件的 VOFA 控制抢同一个通道 */

  /* PD15 (TIM4_CH4) 舵机 PWM 启动，初始停在中位 */
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, servo_angle_to_pulse(current_angle));
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* 开启两路接收 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, PC_RXbuff, sizeof(PC_RXbuff));
  HAL_UARTEx_ReceiveToIdle_IT(&huart5, Car_RXbuff, sizeof(Car_RXbuff));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t tick = HAL_GetTick();

    /* --- 1. 解析 VOFA+ 下发的目标角度（ASCII 文本，如 "45.5"） --- */
    if (rx_line_ready)
    {
        char snapshot[sizeof(rx_line)];

        __disable_irq();
        memcpy(snapshot, rx_line, sizeof(snapshot));
        rx_line_ready = 0;
        __enable_irq();

        char *end = NULL;
        const float value = strtof(snapshot, &end);

        if (end != snapshot)                    /* 至少解出一个数才认 */
        {
            if (value < SERVO_ANGLE_MIN)      target_angle = SERVO_ANGLE_MIN;
            else if (value > SERVO_ANGLE_MAX) target_angle = SERVO_ANGLE_MAX;
            else                              target_angle = value;
        }
    }

    /* --- 2. 非阻塞斜坡：当前角度逐周期逼近目标 --- */
    if (current_angle < target_angle)
    {
        current_angle += SERVO_SLEW_STEP;
        if (current_angle > target_angle) current_angle = target_angle;
    }
    else if (current_angle > target_angle)
    {
        current_angle -= SERVO_SLEW_STEP;
        if (current_angle < target_angle) current_angle = target_angle;
    }

    /* --- 3. 更新 PD15 的 PWM 比较值 --- */
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, servo_angle_to_pulse(current_angle));

    /* --- 4. 回传 JustFloat 帧：[占空比%, 当前角度] --- */
    vofa_send(servo_angle_to_duty(current_angle), current_angle);

    /* --- 5. 对齐到固定周期，保证 VOFA+ 采样率稳定 --- */
    while ((HAL_GetTick() - tick) < CTRL_PERIOD_MS) { }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */




/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
