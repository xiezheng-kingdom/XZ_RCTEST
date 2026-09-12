/**
  ******************************************************************************
  * @file    can1.c
  * @brief   CAN 主机 (Master) 应用层
  ******************************************************************************
  */

#include "can1.h"
#include "can.h"
#include <stdio.h>

/* 按键消抖时间: 电平必须连续稳定这么久才认账 */
#define KEY_DEBOUNCE_MS     20U

/* 按键状态机 (全部非阻塞, 靠 HAL_GetTick 计时, 不在循环里 HAL_Delay) */
static uint8_t  s_raw_last  = 0U;   /* 上一次采样到的原始电平, 1 = 按下 */
static uint8_t  s_stable    = 0U;   /* 已确认稳定的电平 */
static uint8_t  s_pressed   = 0U;   /* 待取走的"按下"事件 */
static uint32_t s_last_edge = 0U;   /* 原始电平上一次变化的时刻 */

void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();   /* gpio.c 已开过 GPIOE, 重复使能无害, 双保险 */

    /* 复位状态机。s_stable 初始为 0(未按下), 所以如果上电时就按住 KEY1,
     * 第一次轮询会看到 raw=1 != s_stable=0, 稳定 20ms 后照样判定为一次按下 ——
     * 即"上电按住 KEY1"也能让本板成为主机。 */
    s_raw_last  = 0U;
    s_stable    = 0U;
    s_pressed   = 0U;
    s_last_edge = HAL_GetTick();

    GPIO_InitStruct.Pin   = CAN1_KEY_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;   /* 按键另一端接 GND, 所以内部上拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN1_KEY_PORT, &GPIO_InitStruct);
}

/* 轮询一次按键, 消抖后把"按下"沿记成事件 */
static void CAN1_KeyPoll(void)
{
    uint8_t  raw = (HAL_GPIO_ReadPin(CAN1_KEY_PORT, CAN1_KEY_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint32_t now = HAL_GetTick();

    if (raw != s_raw_last)            /* 原始电平刚跳变, 重新开始计时 */
    {
        s_raw_last  = raw;
        s_last_edge = now;
        return;
    }

    if ((uint32_t)(now - s_last_edge) < KEY_DEBOUNCE_MS) return;  /* 抖还没消完 */
    if (raw == s_stable) return;                                  /* 稳定值没变, 无事发生 */

    s_stable  = raw;
    if (raw != 0U) s_pressed = 1U;    /* 只在"按下"这一沿产生事件, 松开不产生 */
}

uint8_t CAN1_KeyTakePressed(void)
{
    CAN1_KeyPoll();

    if (s_pressed == 0U) return 0U;
    s_pressed = 0U;                   /* 取走即清零, 保证一次按下只触发一次 */
    return 1U;
}

void CAN1_SendRequest(void)
{
    uint8_t payload[1] = { (uint8_t)CAN_ID_REQ };
    HAL_StatusTypeDef st = CAN_SendStd(CAN_ID_REQ, payload, 1U);

    /* ★★ 顺序很关键: 必须先抓 LEC, 再打印。★☆
     *
     * 下面那句 printf 是阻塞串口 (见 usart.c 的 __io_putchar), 115200 下
     * 约 45 个字符 = 4ms; 而从"帧入队"到 bus-off 只要约 2ms
     * (32 次重发 × 每帧约 58us)。之前把 printf 写在前头, 等轮到抓 LEC 的时候
     * 协议引擎已经停机、LEC 也被 bus-off 抹成 7 了 —— 于是那一行永远打不出来,
     * 看着就像"总线上一点错都没出"。 */
    uint32_t lec = (st == HAL_OK) ? CAN_CatchFirstLec(3U) : 7U;

    /* ★ 紧跟着抓完 LEC 就把状态寄存器整片拍下来 —— 再晚一点 printf 那几毫秒
     *   足够让 bus-off 把现场抹平。IR.TC 能回答"帧到底发出去没有",
     *   IR.BO 能回答"是不是真进过 bus-off", 这两个不会随读消失。 */
    CAN_DumpBusState("after-tx");

    if (st == HAL_OK)
    {
        printf("[MASTER] KEY1 pressed -> sent ID=0x%02X\r\n", CAN_ID_REQ);

        /* LEC 是唯一能分辨故障原因的东西: M_CAN 的 PSR.LEC 读后即清, bus-off
         * 更会把它一并抹平, 事后读寄存器只能看到 7(NoChange)。
         *
         *     LEC=3 应答错误 -> 帧确实发到总线上了, 只是没人回 ACK
         *                       (对面没上电 / 没接线 / 只有这一块板)
         *     LEC=5 Bit0错误 -> 收发器没把总线拉成显性
         *                       (TJA1050 坏了 / 没供 5V / TXD 断线 / S 脚被拉高)
         *     LEC=1/6        -> 填充/CRC 错误, 两边的波特率或采样点对不上 */
        if (lec != 7U)
        {
            printf("[MASTER] bus error: LEC=%lu (%s)\r\n", (unsigned long)lec,
                   (lec == 1U) ? "stuff error - bit timing mismatch"      :
                   (lec == 2U) ? "form error"                             :
                   (lec == 3U) ? "no ACK - no other node on the bus"      :
                   (lec == 4U) ? "bit1 error"                             :
                   (lec == 5U) ? "transceiver is not driving the bus"     :
                   (lec == 6U) ? "CRC error - bit timing mismatch"        : "?");
        }
        else
        {
            printf("[MASTER] no bus error seen in 3ms window\r\n");
        }
    }
    else
    {
        printf("[MASTER] KEY1 pressed -> TX failed, st=%d (TX FIFO full?)\r\n", (int)st);
    }
}

void CAN1_Process(void)
{
    CAN_FrameTypeDef frame;

    /* 1) 按键 -> 发 0x11 */
    if (CAN1_KeyTakePressed() != 0U)
    {
        CAN1_SendRequest();
    }

    /* 2) 抽干 RX FIFO, 收到 0x22 就打印 */
    while (CAN_Receive(&frame) != 0U)
    {
        if (frame.ext || frame.id != CAN_ID_ACK) continue;   /* 不是应答帧, 丢 */

        printf("[MASTER] RX reply ID=0x%02X, DLC=%u, data[0]=0x%02X\r\n",
               CAN_ID_ACK, frame.len, (frame.len > 0U) ? frame.data[0] : 0x00U);
    }
}
