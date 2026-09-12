/**
  ******************************************************************************
  * @file    can1.h
  * @brief   CAN 主机 (Master) 应用层
  ******************************************************************************
  *   角色取得: 上电默认从机, 本板 KEY1 被按下后升为主机 (仲裁在 main.c)。
  *   主机行为:
  *     每按一次 KEY1 (PE3, 按键一端接 GND, 按下读到低) -> 发一帧 ID = 0x11
  *     每收到一帧 ID = 0x22                            -> USART3(PD8) printf 打印
  *
  *   硬件: FDCAN1 (PB8=RX / PB9=TX) -> TJA1050 -> CANH/CANL
  *         总线两端各接 120Ω 终端电阻, TJA1050 的 S 脚接 GND 进正常模式
  ******************************************************************************
  */

#ifndef __CAN1_H__
#define __CAN1_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* KEY1 引脚。改按键只需改这两行 */
#define CAN1_KEY_PORT       GPIOE
#define CAN1_KEY_PIN        GPIO_PIN_3

/**
  * @brief 把 KEY1 配成上拉输入 (按下接地 -> 读低)
  * @note  主机和从机都要调用 —— 从机靠它检测"本板要不要升级成主机"。
  */
void CAN1_Init(void);

/** @brief 发一帧 ID = 0x11 的请求帧 (1 字节数据, 内容 0x11) */
void CAN1_SendRequest(void);

/**
  * @brief 取走一次"KEY1 被按下"事件 (内部已做 20ms 消抖)
  * @retval 1 = 自上次取走后确实按下过一次; 0 = 没有
  * @note   事件取走即清零, 同一次按下不会被取两次, 不会重复发帧。
  */
uint8_t CAN1_KeyTakePressed(void);

/**
  * @brief 主机主循环体, 需要在 while(1) 里反复调用
  * @note  非阻塞。内部顺序: 处理按键 -> 发 0x11 -> 抽干接收 FIFO -> 收到 0x22 就打印
  */
void CAN1_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN1_H__ */
