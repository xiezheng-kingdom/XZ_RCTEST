/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   左电机 位置/速度 级联 PID 闭环控制
 *
 * 架构 (串级双环):
 *   目标位置 ─→ [位置环 PID] ─→ 目标速度 ─→ [速度环 PID] ─→ duty ─→ Motor_Left_Drive
 *               ↑ 位置反馈                           ↑ 速度反馈(编码器)
 *
 * 控制周期: 10ms (由 1ms SysTick 分频得到)
 ******************************************************************************
 */
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "main.h"

/* 控制周期 (ms)，与 motor.c 的 ENC_UPDATE_MS 保持一致 */
#define MC_PERIOD_MS   10

void     MotorControl_SetEnabled(uint8_t en);     /* 0=挂起闭环(开环辨识用) */
void     MotorControl_Init(void);
void     MotorControl_Tick(void);                 /* 1ms SysTick 钩子 */
void     MotorControl_SetTargetPosition(int32_t pos);
int32_t  MotorControl_GetTargetPosition(void);
int32_t  MotorControl_GetDuty(void);
uint8_t  MotorControl_IsArrived(void);

/* 调试遥测: 供主循环以 10ms 节拍输出真实波形 */
float    MotorControl_GetVelRef(void);            /* 斜率受限后的速度参考 counts/s */
float    MotorControl_GetIntegral(void);          /* 速度环积分 (duty) */
int32_t  MotorControl_GetVelocity(void);          /* 实测滤波速度 (直读编码器, 闭环挂起时也有效) */

#endif /* __MOTOR_CONTROL_H__ */
