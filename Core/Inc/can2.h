/**
  ******************************************************************************
  * @file    can2.h
  * @brief   CAN 从机 (Slave) 应用层
  ******************************************************************************
  *   角色取得: 上电默认就是从机, 不需要任何操作。
  *   从机行为:
  *     每收到一帧 ID = 0x11 -> 回一帧 ID = 0x22, 同时翻转 PA1 电平
  *     PA1 上电初始化为低电平
  *
  *   硬件: 与主机相同 —— FDCAN1 (PB8=RX / PB9=TX) -> TJA1050 -> CANH/CANL
  ******************************************************************************
  */

#ifndef __CAN2_H__
#define __CAN2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 翻转电平的输出脚。改脚位只需改这两行 */
#define CAN2_LED_PORT       GPIOA
#define CAN2_LED_PIN        GPIO_PIN_1

/**
  * @brief 把 PA1 配成推挽输出并置为低电平
  * @note  主机上也会调用 (角色是运行时才定的), 初始低电平无害。
  */
void CAN2_Init(void);

/** @brief 回一帧 ID = 0x22 的应答帧 (1 字节数据, 内容 0x22) */
void CAN2_SendAck(void);

/**
  * @brief 从机主循环体, 需要在 while(1) 里反复调用
  * @note  非阻塞。收到 0x11 -> 回 0x22 + 翻转 PA1 + 串口打印。
  */
void CAN2_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN2_H__ */
