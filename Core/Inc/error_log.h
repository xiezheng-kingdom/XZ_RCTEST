/**
 ******************************************************************************
 * @file    error_log.h
 * @brief   位置误差记录模块 — Kalman预测 vs IR路口实测 + PID性能
 *
 * 每到达一个路口, 记录:
 *   - 编码器+Kalman预测距离
 *   - 地图实际距离 (路口间已知距离)
 *   - 误差 = 预测 - 实际
 *   - PID 左右轮平均占空比
 *   - MPU6050 加速度均值
 *
 * 数据存入内存缓冲区, 可通过串口导出分析.
 ******************************************************************************
 */
#ifndef __ERROR_LOG_H__
#define __ERROR_LOG_H__

#include "main.h"

#define EL_MAX_ENTRIES  64          /* 最多64条记录 */

typedef struct {
    uint8_t  seg_id;                /* 路段编号 */
    float    pred_cm;               /* Kalman预测距离 (cm) */
    float    actual_cm;             /* 地图实际距离 (cm) */
    float    error_cm;              /* 误差 (cm) */
    float    error_pct;             /* 误差百分比 */
    float    pid_avg_L;             /* 左轮平均PWM */
    float    pid_avg_R;             /* 右轮平均PWM */
    float    speed_avg_L;           /* 左轮平均速度 m/s */
    float    speed_avg_R;           /* 右轮平均速度 m/s */
    float    mpu_ax_avg;            /* MPU6050 平均加速度 */
    uint32_t duration_ms;           /* 本段耗时 ms */
} ErrorLogEntry;

typedef struct {
    ErrorLogEntry entries[EL_MAX_ENTRIES];
    uint8_t       count;
    float         total_error_cm;   /* 累计误差 cm */
    float         total_distance;   /* 累计距离 cm */
} ErrorLogger;

extern ErrorLogger g_elog;

void EL_Init(void);
void EL_BeginSegment(uint8_t seg_id, float actual_dist_cm);
void EL_EndSegment(void);           /* 到达路口时调用, 记录数据 */
void EL_UpdatePID(float L, float R, float vL, float vR);  /* 每循环更新PID均值 */
void EL_UpdateMPU(float ax);        /* 更新加速度均值 */
void EL_PrintSummary(void);         /* 串口输出汇总 */
void EL_Reset(void);

#endif
