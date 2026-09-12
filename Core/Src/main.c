/* USER CODE BEGIN Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include "adc.h"
#include "motor_control.h"
#include "can.h"
#include "can1.h"
#include "can2.h"
#include <stdio.h>

void SystemClock_Config(void);
void MPU_Config(void);

/* ── 主从角色仲裁 ────────────────────────────────────────────────────────────
 * 两块板烧同一份固件, 不靠跳线区分身份:
 *   上电默认都是从机; 哪块板按下自己的 KEY1(PE3), 哪块就成为主机,
 *   并在升级的这一下把 0x11 发出去, 对面从机收到后立刻回 0x22。
 *   角色一旦确定就保持, 不会自己变回去。
 *
 * ⚠️ 所以上电后要先去"想当主机"的那块板上按一下 KEY1。
 *    想上电即为主机, 按住 KEY1 再上电即可 (见 can1.c 里的说明)。 */
typedef enum {
  ROLE_SLAVE  = 0,
  ROLE_MASTER = 1
} Role_t;

/* 初始化骨架。业务逻辑请挂在 SysTick 或主循环里, CAN 收发用 can.h 的接口。
 *
 * ★ 相对清空前的两处取舍 (原始版本见 git 历史):
 *   1. 去掉 MX_I2C1_Init() —— I2C1 的 SCL/SDA 是 PB8/PB9, 已让给 FDCAN1。
 *      连带去掉 OLED_Init()/OLED_Clear()/OLED_Refresh(), OLED 屏不再可用。
 *      i2c.c 里 hi2c1 的定义和 oled.c 都还在, 只是没有任何调用点了。
 *   2. 去掉 MotorControl_SetTargetPosition() —— 那行是上电自走一圈的测试代码,
 *      不属于初始化。目标位置现在保持为 0, 电机上电后原地锁死。
 *      ★ 现在没有任何东西会驱动电机, 要动起来得由 CAN 或其它逻辑下发目标。 */
int main(void)
{
  MPU_Config(); SCB_EnableICache(); SCB_EnableDCache(); HAL_Init(); SystemClock_Config();

  MX_GPIO_Init(); MX_TIM1_Init(); MX_TIM2_Init(); MX_TIM3_Init();
  MX_USART2_UART_Init(); MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_ADC3_Init();

  /* CAN_Init() 放在时钟配置之后: FDCAN 的内核时钟用的是 HSE, 得等 HSE 起振。
   * (曾经用过 PLL1Q, 但这条路径在本板子上不出时钟, 见 can.c 的说明。) */
  CAN_Init();

  /* 上电自检: 环回发一帧再收回来。这一路不经过 TJA1050, 所以
   *   打印 PASS -> MCU/FDCAN 侧没问题, 后面通信不顺只可能是收发器或总线;
   *   打印 FAIL -> 先别怀疑 TJA1050, MCU 侧(时钟/位时序/消息 RAM)还没通。
   *   FAIL 时会额外打印现场: TEST.bit4(LBCK) 必须是 1, 否则"环回"根本没生效。 */
  CAN_SelfTestInfoTypeDef st;
  uint8_t selfOk = CAN_SelfTest(&st);

  printf("[CAN] self-test (loopback, transceiver bypassed): %s\r\n",
         (selfOk != 0U) ? "PASS" : "FAIL");

  if (selfOk == 0U)
  {
    printf("[CAN]   tried=%lu  CCCR=0x%08lX  TEST=0x%08lX (LBCK=%lu)  LEC=%lu\r\n",
           (unsigned long)st.tries, (unsigned long)st.cccr, (unsigned long)st.test,
           (unsigned long)((st.test & FDCAN_TEST_LBCK) ? 1U : 0U),
           (unsigned long)st.lec);
  }

  Motor_Init(); Encoder_Start();
  MotorControl_Init();

  /* 两个角色模块都初始化: KEY1 输入、PA1 输出低。谁当主机要运行时才定,
   * 所以两边的引脚都先配好, 未用到的那个不影响任何东西。 */
  CAN1_Init();
  CAN2_Init();

  Role_t role = ROLE_SLAVE;
  printf("\r\n[CAN] Boot role = SLAVE. Press KEY1 on this board to become MASTER.\r\n");

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ── CAN 看门狗: 从 bus-off 里把外设捞回来 ─────────────────────────────
     * M_CAN 一旦 bus-off 就把自己锁在 CCCR.INIT=1 上停机, **硬件不会自愈**:
     * 之后入队的帧全都发不出去, 表现就是"按一次 KEY1 之后再也发不出来"。
     * bus-off 的门槛低得吓人 —— 只要总线上没有第二个节点回 ACK(对面没上电/
     * 没接线/波特率不一致/只有一块板), 32 次重发约 2ms 内 TEC 就满 256 了。
     * 所以这一步是必需品, 不是调试脚手架。 */
    CAN_FaultInfoTypeDef fault;
    if (CAN_RecoverIfBusOff(&fault) != 0U)
    {
      /* ★ 别把这行当成"确定是 bus-off": 判据是 CCCR.INIT=1, 而 HAL_FDCAN_Init
       *   自己退出时也把 INIT 留在 1(靠随后的 HAL_FDCAN_Start 清)。
       *   只有 IR.bit25(BO)=1 才真的证明进过 bus-off —— IR 是写 1 才清,
       *   不像 PSR.BO 会在一百多个隐性位后自己消失。 */
      printf("[CAN] engine halted, CCCR.INIT=1! CCCR=0x%08lX PSR=0x%08lX ECR=0x%08lX TXFQS=0x%08lX IR=0x%08lX -> re-init (%s)\r\n",
             (unsigned long)fault.cccr, (unsigned long)fault.psr,
             (unsigned long)fault.ecr,  (unsigned long)fault.txfqs,
             (unsigned long)fault.ir,
             ((fault.ir & FDCAN_IR_BO) != 0U) ? "real bus-off"
                                              : "NOT bus-off - check the INIT path");
    }

    if (role == ROLE_MASTER)
    {
      /* 主机: 按 KEY1 -> 发 0x11; 收到 0x22 -> 串口打印 */
      CAN1_Process();
    }
    else
    {
      /* 从机: 收到 0x11 -> 回 0x22 并翻转 PA1 */
      CAN2_Process();

      /* 顺带看本板 KEY1 有没有被按下 —— 按了就升级成主机。
       * 注意取事件这个动作本身有副作用(清零), 所以只在从机分支里调一次。 */
      if (CAN1_KeyTakePressed() != 0U)
      {
        role = ROLE_MASTER;
        printf("[CAN] KEY1 pressed -> this board is now MASTER\r\n");
        CAN1_SendRequest();   /* 升级的这一下也把 0x11 发出去 */
      }
    }
    /* USER CODE END 3 */
  }
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
