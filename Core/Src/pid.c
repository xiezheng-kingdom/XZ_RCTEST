/**
 ******************************************************************************
 * @file    pid.c
 * @brief   PID 通用控制器实现
 *
 * 核心公式:
 *   output = Kp*error + Ki*integral + Kd*derivative
 *
 * 防积分饱和 (windup) 策略:
 *   CLAMP 模式:   integral 被硬夹在 ±integral_limit
 *   SEPARATE 模式: |error| > threshold 时 integral 清零
 *
 * 实际使用:
 *   位置环: Kp=0.001~0.005, Ki=0.0001~0.001, Kd=0.005~0.03
 *           integral_limit = output_max * 0.3
 *   速度环: Kp=0.2~1.0, Ki=0.01~0.1, Kd=0
 *           integral_limit = output_max * 0.5
 ******************************************************************************
 */
#include "pid.h"
#include <string.h>   /* memset */
#include <math.h>     /* fabsf */

/* ==================== 初始化 ==================== */

void PID_Init(PID_HandleTypeDef *pid,
              float Kp, float Ki, float Kd,
              float output_min, float output_max)
{
    if (pid == NULL) return;
    memset(pid, 0, sizeof(*pid));

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_min = output_min;
    pid->output_max = output_max;

    /* 默认: 硬限幅方式, 积分上限取输出范围的 30% */
    pid->integral_mode   = PID_INTEGRAL_CLAMP;
    pid->integral_limit  = (output_max - output_min) * 0.3f;
    pid->initialized = 1;
}

void PID_SetIntegralLimit(PID_HandleTypeDef *pid, float limit)
{
    if (pid == NULL) return;
    pid->integral_mode  = PID_INTEGRAL_CLAMP;
    pid->integral_limit = limit;
}

void PID_SetIntegralSeparate(PID_HandleTypeDef *pid, float threshold)
{
    if (pid == NULL) return;
    pid->integral_mode       = PID_INTEGRAL_SEPARATE;
    pid->separate_threshold  = threshold;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    if (pid == NULL) return;
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
    pid->output     = 0.0f;
}

/* ==================== PID 步进 ==================== */

float PID_Update(PID_HandleTypeDef *pid, float setpoint, float measured)
{
    if (pid == NULL || !pid->initialized) return 0.0f;

    float error = setpoint - measured;

    /* --- 比例项 --- */
    float p_term = pid->Kp * error;

    /* --- 积分项 (带 anti-windup) --- */
    if (pid->integral_mode == PID_INTEGRAL_SEPARATE) {
        /* 分离积分: 误差大时清零积分, 避免 windup */
        if (fabsf(error) > pid->separate_threshold) {
            pid->integral = 0.0f;
        }
    }
    pid->integral += error;
    /* 硬限幅 (CLAMP 模式始终生效, SEPARATE 作为补充保护) */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_term = pid->Ki * pid->integral;

    /* --- 微分项 (on measurement, 不 on error 避免微分冲击) --- */
    float d_term = pid->Kd * (measured - pid->last_error);  /* = -Kd * d(error)/dt */
    pid->last_error = measured;

    /* --- 合成 --- */
    float output = p_term + i_term - d_term;  /* 注意 d_term 符号 */

    /* --- 输出限幅 + 积分回算 (clamping anti-windup) --- */
    float out_clamped;
    if (output > pid->output_max) {
        out_clamped = pid->output_max;
    } else if (output < pid->output_min) {
        out_clamped = pid->output_min;
    } else {
        out_clamped = output;
        pid->output = output;
        return output;   /* 未限幅, integral 保持 */
    }

    /* 输出被限幅 → 回退积分防止饱和 */
    if (pid->Ki > 0.0f) {
        pid->integral -= error;  /* 撤销本次 error 的积分 */
    }
    pid->output = out_clamped;
    return out_clamped;
}

/* ==================== 查询 ==================== */

float PID_GetOutput(const PID_HandleTypeDef *pid)
{
    return (pid != NULL) ? pid->output : 0.0f;
}
