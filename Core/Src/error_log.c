/**
 ******************************************************************************
 * @file    error_log.c
 * @brief   误差记录实现 — 内存缓冲区 + 串口导出
 ******************************************************************************
 */
#include "error_log.h"
#include "road_map.h"
#include <stdio.h>
#include <string.h>

ErrorLogger g_elog;

static uint32_t m_seg_start;        /* 路段开始时刻 */
static float    m_pid_sum_L;        /* PID 累加 */
static float    m_pid_sum_R;
static float    m_speed_sum_L;
static float    m_speed_sum_R;
static float    m_mpu_sum;
static uint32_t m_sample_count;

void EL_Init(void)
{
    memset(&g_elog, 0, sizeof(g_elog));
}

void EL_BeginSegment(uint8_t seg_id, float actual_dist_cm)
{
    if (g_elog.count >= EL_MAX_ENTRIES) return;
    ErrorLogEntry *e = &g_elog.entries[g_elog.count];
    memset(e, 0, sizeof(*e));
    e->seg_id    = seg_id;
    e->actual_cm = actual_dist_cm;
    m_seg_start  = HAL_GetTick();
    m_pid_sum_L  = 0;
    m_pid_sum_R  = 0;
    m_speed_sum_L = 0;
    m_speed_sum_R = 0;
    m_mpu_sum    = 0;
    m_sample_count = 0;
}

void EL_EndSegment(void)
{
    if (g_elog.count >= EL_MAX_ENTRIES) return;
    ErrorLogEntry *e = &g_elog.entries[g_elog.count];

    /* 从 road_map 获取 Kalman 预测距离 */
    e->pred_cm   = RoadMap_GetProgress();
    e->error_cm  = e->pred_cm - e->actual_cm;
    if (e->actual_cm > 0.1f)
        e->error_pct = e->error_cm / e->actual_cm * 100.0f;
    e->duration_ms = HAL_GetTick() - m_seg_start;

    if (m_sample_count > 0) {
        e->pid_avg_L   = m_pid_sum_L   / (float)m_sample_count;
        e->pid_avg_R   = m_pid_sum_R   / (float)m_sample_count;
        e->speed_avg_L = m_speed_sum_L / (float)m_sample_count;
        e->speed_avg_R = m_speed_sum_R / (float)m_sample_count;
        e->mpu_ax_avg  = m_mpu_sum     / (float)m_sample_count;
    }

    g_elog.total_error_cm += (e->error_cm > 0 ? e->error_cm : -e->error_cm);
    g_elog.total_distance  += e->actual_cm;
    g_elog.count++;

    printf("ELOG[%d]: pred=%.1f act=%.1f err=%.1fcm(%.1f%%) PID=%.0f/%.0f\r\n",
           e->seg_id, e->pred_cm, e->actual_cm,
           e->error_cm, e->error_pct, e->pid_avg_L, e->pid_avg_R);
}

void EL_UpdatePID(float L, float R, float vL, float vR)
{
    m_pid_sum_L   += L;
    m_pid_sum_R   += R;
    m_speed_sum_L += vL;
    m_speed_sum_R += vR;
    m_sample_count++;
}

void EL_UpdateMPU(float ax)
{
    m_mpu_sum += ax;
}

void EL_PrintSummary(void)
{
    printf("=== ERROR LOG SUMMARY ===\r\n");
    printf("Segments: %d\r\n", g_elog.count);
    for (int i = 0; i < g_elog.count; i++) {
        ErrorLogEntry *e = &g_elog.entries[i];
        printf(" S%d: pred=%.1f act=%.1f err=%+.1fcm(%.1f%%) | PID L=%.0f R=%.0f | vL=%.2f vR=%.2f | %lums\r\n",
               e->seg_id, e->pred_cm, e->actual_cm,
               e->error_cm, e->error_pct,
               e->pid_avg_L, e->pid_avg_R,
               e->speed_avg_L, e->speed_avg_R,
               (unsigned long)e->duration_ms);
    }
    if (g_elog.total_distance > 0)
        printf("TOTAL: dist=%.1fcm err_sum=%.1fcm avg_err=%.1f%%\r\n",
               g_elog.total_distance, g_elog.total_error_cm,
               g_elog.total_error_cm / g_elog.total_distance * 100.0f);
    printf("=== END ===\r\n");
}

void EL_Reset(void)
{
    memset(&g_elog, 0, sizeof(g_elog));
}
