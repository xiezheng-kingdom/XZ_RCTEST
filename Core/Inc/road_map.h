/**
 ******************************************************************************
 * @file    road_map.h
 * @brief   多传感器融合定位中枢
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │                     ROAD_MAP                           │
 * │  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
 * │  │ ENCODER  │  │ MPU6050  │  │  IR x4   │             │
 * │  │ KALMAN   │  │ (备用)   │  │ JUNCTION │             │
 * │  └────┬─────┘  └────┬─────┘  └────┬─────┘             │
 * │       │里程+速度      │航向角      │循线+路口            │
 * │       └───────────────┼───────────┘                    │
 * │                       ▼                                │
 * │              融合位置 (x, y, θ, node)                  │
 * │                       ▲                                │
 * │       ┌───────────────┘                                │
 * │       │ QR绝对定位                                      │
 * │  ┌────┴──────────┐  ┌──────────┐                      │
 * │  │ LOCALIZATION  │  │  OPENMV  │                      │
 * │  │ (K230 QR码)   │  │ (货物识别)│                      │
 * │  └───────────────┘  └──────────┘                      │
 * └─────────────────────────────────────────────────────────┘
 *                  │ 位置输出
 *                  ▼
 *            ┌──────────┐
 *            │  CARGO   │ → HC06/USART 指令执行
 *            └──────────┘
 *
 * 定位策略:
 *   1. QR码绝对定位: 最高置信度, 直接修正位置 + 确认节点
 *   2. IR路口检测: 确认到达路口节点
 *   3. 编码器+Kalman: 节点间里程计 (DIST_CALIB=8.0标定)
 *   4. IR循线: 保持车体在黑色轨迹上
 *   5. MPU6050: 航向角参考 (芯片损坏, 暂不可用)
 ******************************************************************************
 */
#ifndef __ROAD_MAP_H__
#define __ROAD_MAP_H__

#include "main.h"
#include "ir_sensor.h"
#include "kalman.h"
#include "localization.h"

/* ================================================================
 * 地图节点
 * ================================================================ */
typedef enum {
    NODE_1=1,  NODE_2=2,  NODE_3=3,   /* 储物台 */
    NODE_4=4,                            /* 丁字路口 (5在正下) */
    NODE_5=5,                            /* 出发点 */
    NODE_6=6,                            /* 取货点 */
    NODE_7=7,                            /* 杂货区 */
    NODE_8=8,                            /* 出货点 */
    NODE_9=9,                            /* 左路口 */
    NODE_10=10,                          /* 右路口 */
    NODE_MAX=11
} MapNode;

/* ================================================================
 * 定位置信度
 * ================================================================ */
typedef enum {
    CONF_NONE   = 0,   /* 无定位信息 */
    CONF_ODOM   = 1,   /* 仅里程计 (漂移累积) */
    CONF_LINE   = 2,   /* 循线中 (准确) */
    CONF_JUNC   = 3,   /* 路口确认 (高置信) */
    CONF_QR     = 4,   /* QR绝对定位 (最高置信) */
} PosConfidence;

/* ================================================================
 * 统一位置
 * ================================================================ */
typedef struct {
    float   x_cm;           /* X 坐标 cm (世界系) */
    float   y_cm;           /* Y 坐标 cm */
    float   heading_deg;    /* 航向角 (度, 0=N) */
    MapNode node;           /* 当前所在节点 (到达路口时更新) */
    uint8_t on_line;        /* 1=在黑色循迹线上 */
    PosConfidence conf;     /* 位置置信度 */
} RoadMapPosition;

/* ================================================================
 * 底盘参数
 * ================================================================ */
typedef struct {
    float     wheel_dia_mm;
    float     track_width_mm;
    float     kf_calib;
    uint16_t  duty_min;
    uint16_t  duty_run;
} ChassisParams;

/* ================================================================
 * 运动阶段
 * ================================================================ */
typedef enum {
    PHASE_IDLE = 0,
    PHASE_MOVING,         /* 正在循线直行 */
    PHASE_ROTATING,       /* 正在旋转 */
    PHASE_DONE,           /* 动作完成 */
} MotionPhase;

/* ================================================================
 * 全局变量
 * ================================================================ */
extern RoadMapPosition g_pos;
extern ChassisParams   g_chassis;
extern IR_SensorTypeDef m_ir;
extern uint8_t g_dbg_motor;

/* ================================================================
 * API — 初始化
 * ================================================================ */
void RoadMap_Init(void);

/* ================================================================
 * API — 传感器融合 (每循环调用)
 * ================================================================ */
void RoadMap_Update(void);        /* 读取所有传感器, 融合定位 */

/* ================================================================
 * API — 运动控制
 * ================================================================ */

/* 循线直行到目标节点 */
void RoadMap_GoToNode(MapNode target, uint16_t duty);

/* 沿当前线直行 dist_cm (不使用节点判定) */
void RoadMap_GoStraight(float dist_cm, uint16_t duty);

/* 原地旋转 */
void RoadMap_Rotate(float angle_deg, uint16_t duty);

/* 停止 */
void RoadMap_Stop(void);

/* 每循环推进运动 */
void RoadMap_Run(void);

/* ================================================================
 * API — 位置修正 (外部模块调用)
 * ================================================================ */

/* QR码绝对定位修正 (由 LOCALIZATION 或 CARGO 调用) */
void RoadMap_QRFix(MapNode node, float x_cm, float y_cm, float heading);

/* ================================================================
 * API — 查询
 * ================================================================ */
MotionPhase    RoadMap_GetPhase(void);
uint8_t        RoadMap_IsDone(void);
float          RoadMap_GetProgress(void);
MapNode        RoadMap_GetCurrentNode(void);
MapNode        RoadMap_GetTargetNode(void);
PosConfidence  RoadMap_GetConfidence(void);
uint8_t        RoadMap_IsAtJunction(void);
float          RoadMap_GetSpeedL(void);
float          RoadMap_GetSpeedR(void);

#endif
