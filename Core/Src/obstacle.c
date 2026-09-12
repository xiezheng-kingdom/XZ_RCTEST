/**
 ******************************************************************************
 * @file    obstacle.c
 * @brief   超声波障碍物避让: 距离 → 速度限制
 *
 * 速度限制策略 (分段线性, 无超调):
 *
 *   speed_limit
 *     ^
 *  max|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\
 *     |                              \
 *     |                               \
 *     |                                \
 *     |                                 \
 *    0|                                  ‾‾‾‾‾‾‾‾‾
 *     +--------+-------------------------+------→ distance_cm
 *     0       STOP                     WARN
 *            (15cm)                    (50cm)
 *
 *   距离 >= WARN:  全速 (speed_limit = max)
 *   STOP < d < WARN: 线性递减
 *   距离 <= STOP:  紧急停车 (speed_limit = 0)
 *   距离 < 0(无效): 保持上次, 标记 TIMEOUT
 *
 * 与巡线 PID 集成:
 *   limit = Obstacle_GetSpeedLimit(&obs);
 *   motor_target = fminf(line_follow_target, limit);
 ******************************************************************************
 */
#include "obstacle.h"

/* 最大允许速度 (m/s) */
#define OBSTACLE_MAX_SPEED_MS   0.5f

static float linear_map(float d)
{
    if (d >= OBSTACLE_WARN_CM)
        return OBSTACLE_MAX_SPEED_MS;

    if (d <= OBSTACLE_STOP_CM)
        return 0.0f;

    /* 线性插值: (d - STOP) / (WARN - STOP) × MAX */
    float ratio = (d - OBSTACLE_STOP_CM) / (OBSTACLE_WARN_CM - OBSTACLE_STOP_CM);
    return OBSTACLE_MAX_SPEED_MS * ratio;
}

/* ====================== API ====================== */

void Obstacle_Init(ObstacleHandle *obs)
{
    if (obs == NULL) return;
    obs->state       = OBSTACLE_CLEAR;
    obs->distance_cm = OBSTACLE_MAX_CM;
    obs->speed_limit = OBSTACLE_MAX_SPEED_MS;

    /* 距离 PID: Kp 将距离误差映射到速度修正量 (备用, 当前用线性映射) */
    PID_Init(&obs->pid, 0.01f, 0.0f, 0.0f, 0.0f, OBSTACLE_MAX_SPEED_MS);
    PID_SetIntegralLimit(&obs->pid, OBSTACLE_MAX_SPEED_MS * 0.3f);
}

void Obstacle_Update(ObstacleHandle *obs, float distance_cm)
{
    if (obs == NULL) return;
    obs->distance_cm = distance_cm;

    /* 超声波无效 (超时/未连接) */
    if (distance_cm < 0.0f) {
        obs->state = OBSTACLE_TIMEOUT;
        /* 保持上次 speed_limit, 不做改变 */
        return;
    }

    /* 超出量程: 视为无障碍 */
    if (distance_cm > OBSTACLE_MAX_CM) {
        obs->state       = OBSTACLE_CLEAR;
        obs->speed_limit = OBSTACLE_MAX_SPEED_MS;
        return;
    }

    /* 紧急停车 */
    if (distance_cm <= OBSTACLE_STOP_CM) {
        obs->state       = OBSTACLE_STOP;
        obs->speed_limit = 0.0f;
        PID_Reset(&obs->pid);  /* 清积分防重启冲击 */
        return;
    }

    /* 预警区: 线性减速 */
    if (distance_cm < OBSTACLE_WARN_CM) {
        obs->state       = OBSTACLE_WARN;
        obs->speed_limit = linear_map(distance_cm);
        return;
    }

    /* 安全距离: 全速 */
    obs->state       = OBSTACLE_CLEAR;
    obs->speed_limit = OBSTACLE_MAX_SPEED_MS;
}

float Obstacle_GetSpeedLimit(const ObstacleHandle *obs)
{
    return (obs != NULL) ? obs->speed_limit : OBSTACLE_MAX_SPEED_MS;
}

ObstacleState Obstacle_GetState(const ObstacleHandle *obs)
{
    return (obs != NULL) ? obs->state : OBSTACLE_TIMEOUT;
}
