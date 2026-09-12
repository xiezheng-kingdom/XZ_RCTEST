/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID 通用控制器框架
 *
 * 用法:
 *   1. PID_Init(&pid, Kp, Ki, Kd, out_min, out_max);
 *   2. 定时调用 output = PID_Update(&pid, setpoint, measured);
 *
 * 支持场景:
 *   - 巡线位置 PID (外环): 输入=红外位置偏差, 输出=目标速度修正量
 *   - 电机速度 PID (内环): 输入=目标RPM, 测量=编码器RPM, 输出=PWM占空比
 ******************************************************************************
 */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* PID 误差积分限幅方式 */
typedef enum {
    PID_INTEGRAL_CLAMP = 0,   /* 硬限幅: integral 被夹在 ±limit 内 */
    PID_INTEGRAL_SEPARATE,    /* 分离积分: |error|大时清空积分 */
} PID_IntegralMode;

typedef struct {
    /* --- 参数 --- */
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */

    /* --- 限幅 --- */
    float output_min;   /* 输出下限 */
    float output_max;   /* 输出上限 */
    float integral_limit;       /* 积分累加器硬限幅 */
    float separate_threshold;   /* 积分分离阈值 (mode=SEPARATE时有效) */
    PID_IntegralMode integral_mode;

    /* --- 内部状态 (只读) --- */
    float integral;     /* 积分累加值 */
    float last_error;   /* 上一次误差 */
    float output;       /* 最新输出值 */
    uint8_t initialized;
} PID_HandleTypeDef;

/* 初始化 PID 控制器 */
void PID_Init(PID_HandleTypeDef *pid,
              float Kp, float Ki, float Kd,
              float output_min, float output_max);

/* 设置积分限幅模式 */
void PID_SetIntegralLimit(PID_HandleTypeDef *pid, float limit);
void PID_SetIntegralSeparate(PID_HandleTypeDef *pid, float threshold);

/* 重置积分器和历史误差 */
void PID_Reset(PID_HandleTypeDef *pid);

/* 步进一次 PID 计算，返回控制量 */
float PID_Update(PID_HandleTypeDef *pid, float setpoint, float measured);

/* 获取最后一次输出 (调试用) */
float PID_GetOutput(const PID_HandleTypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
