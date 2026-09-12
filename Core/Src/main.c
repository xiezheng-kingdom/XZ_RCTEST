/* USER CODE BEGIN Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "oled.h"
#include "motor.h"
#include "adc.h"
#include "motor_control.h"

/* ---- 调试帧: N 个 float + 帧尾 00 00 80 7F, 经 huart3(PD8) 发送 ----
 * 这就是 VOFA+ 的 JustFloat 协议, VOFA 里选 JustFloat、通道数填 CH_COUNT 即可。
 * 波特率 115200 (见 usart.c 的 USART3->BRR)。 */
#define CH_COUNT 6   /* 通道: pos, vel, vel_ref, duty, integral, target_pos */

typedef struct {
    float         fdata[CH_COUNT];
    unsigned char tail[4];  /* 帧尾 0x00 0x00 0x80 0x7f = +Inf(little-endian float), 作帧同步 */
} Frame;

void SystemClock_Config(void);
void MPU_Config(void);

int main(void)
{
  MPU_Config(); SCB_EnableICache(); SCB_EnableDCache(); HAL_Init(); SystemClock_Config();
  /* MX_TIM4_Init() 已移除: TIM4(CH3/CH4=PD14/PD15) 全工程无任何应用代码引用,
   * 只有 tim.c 自己的初始化。需要时加回来即可。 */
  MX_GPIO_Init(); MX_TIM1_Init(); MX_TIM2_Init(); MX_TIM3_Init();
  MX_USART2_UART_Init(); MX_USART3_UART_Init(); MX_I2C1_Init(); MX_I2C2_Init();
  MX_ADC3_Init();
  OLED_Init(); HAL_Delay(200); OLED_Clear(); OLED_Refresh();
  Motor_Init(); Encoder_Start();
  MotorControl_Init();
  /* 目标 = 当前 + 1792×18 = 32256 counts = 车轮一圈 */
  MotorControl_SetTargetPosition(Encoder_GetRightPosition() + (ENCODER_PPR * 4 * GEAR_RATIO));
  /* 已移除 HC06_Init() / Cargo_Init() / MPU6050_Init() / IR_Init() 四行调用。
   *
   * 原因: 整条业务链没有入口 —— HC06_Process()(hc06.c:150) 和 Cargo_Run()
   * (cargo.c:220) 全工程都没有调用点, 所以这些初始化做完之后永远不会被读到。
   *   HC06 收字节 → 环形缓冲 → [HC06_Process 缺] → Cargo_FeedByte
   *   Cargo 状态机 → [Cargo_Run 缺] → Nav_xxx() → IR_Update()
   *
   * ★ 将来接回业务链时, 除了把这四行加回来, 还必须在上面的 while(1) 里补上
   *   HC06_Process() 和 Cargo_Run() —— 光加初始化是没用的。
   *
   * 附带收益: HC06_Init() 里有个 HAL_Delay(1000), 删掉后启动快 1 秒。
   * 另外 MPU6050 芯片本身已损坏(见 road_map.h), IR_Init() 只被 nav.c 读取。 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 控制由 SysTick 中断驱动 (10ms)，主循环同步以 10ms 发送二进制调试帧。
     * ★ 必须与控制同节拍: 早期的 200ms 发送把几十 ms 的极限环混叠成了
     *   看不出真实形状的锯齿, 导致误判。 */
    {
      static uint32_t last_print = 0;
      if (HAL_GetTick() - last_print >= 10) {
        last_print = HAL_GetTick();
        Frame frame;
        frame.fdata[0] = (float)Encoder_GetRightPosition();       /* 位置 */
        frame.fdata[1] = (float)MotorControl_GetVelocity();       /* 实测速度(滤波) */
        frame.fdata[2] = MotorControl_GetVelRef();                /* 速度参考 */
        frame.fdata[3] = (float)MotorControl_GetDuty();           /* 实际 duty */
        frame.fdata[4] = MotorControl_GetIntegral();              /* 速度环积分 */
        /* 目标位置: 调位置环必须能直接看到"目标 vs 实际"两条线, 否则只能反推误差 */
        frame.fdata[5] = (float)MotorControl_GetTargetPosition(); /* 目标位置 */
        frame.tail[0] = 0x00; frame.tail[1] = 0x00;
        frame.tail[2] = 0x80; frame.tail[3] = 0x7f;
        HAL_UART_Transmit(&huart3, (uint8_t *)&frame, sizeof(frame), 100);
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
