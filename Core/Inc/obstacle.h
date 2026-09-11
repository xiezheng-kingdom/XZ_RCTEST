/**
 ******************************************************************************
 * @file    obstacle.h
 * @brief   超声波障碍物检测 + 避让 (距离 PID 仲裁)
 *
 * 策略:
 *   distance > WARN_CM   → 正常行驶, 无限制
 *   WARN_CM >= d > STOP_CM → 减速区, 距离 PID 输出速度上限
 *   d <= STOP_CM         → 紧急停车 speed=0
 *
 * 与巡线 PID 配合:
 *   target_speed = min(line_follow_speed, obstacle_speed_limit)
 *
 * 超声波: HC-SR04, 15Hz 测距, 有效范围 2-400cm
 * ================================================================
 */
#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

/* ================================================================
 * 距离阈值 (cm) — 按实际调
 * ================================================================ */
#define OBSTACLE_WARN_CM    50.0f   /* 进入预警区 */
#define OBSTACLE_STOP_CM    15.0f   /* 紧急停车 */
#define OBSTACLE_MAX_CM     400.0f  /* 超声波有效上限 */

/* ================================================================
 * 状态
 * ================================================================ */
typedef enum {
    OBSTACLE_CLEAR = 0,   /* 无遮挡 */
    OBSTACLE_WARN,        /* 预警: 减速中 */
    OBSTACLE_STOP,        /* 停车: 距离<=STOP_CM */
    OBSTACLE_TIMEOUT,     /* 超声波超时/无效 */
} ObstacleState;

typedef struct {
    ObstacleState state;
    float distance_cm;          /* 最新测距 */
    float speed_limit;          /* 速度上限 (m/s), 供电机控制用 */
    PID_HandleTypeDef pid;      /* 距离 PID: 距离→速度限制 */
} ObstacleHandle;

/* ================================================================
 * API
 * ================================================================ */
void Obstacle_Init(ObstacleHandle *obs);
void Obstacle_Update(ObstacleHandle *obs, float distance_cm);
float Obstacle_GetSpeedLimit(const ObstacleHandle *obs);
ObstacleState Obstacle_GetState(const ObstacleHandle *obs);

#ifdef __cplusplus
}
#endif

#endif /* __OBSTACLE_H__ */
