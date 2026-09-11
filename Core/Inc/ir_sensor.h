/**
 ******************************************************************************
 * @file    ir_sensor.h
 * @brief   4路红外巡线 + 路口类型判定
 *
 * 探头布局 (俯视, 车头朝上):
 *     S1           S2           S3           S4
 *   [左]        [中左]        [中右]        [右]
 *    -3           -1           +1           +3   ← 权重
 *
 * 路口判定 (基于 4 路同时黑线的持续时间):
 *   十字路口: 4路全黑且持续 → CROSS
 *   T字左支: 左侧(S1)先黑再全黑 → T_LEFT
 *   T字右支: 右侧(S4)先黑再全黑 → T_RIGHT
 *
 * 判据: 小车通过路口约需 200~400ms (线宽1.5cm, 车速~5cm/s)
 *       全黑持续 >= 50ms 且 < 600ms 才认为是路口
 ******************************************************************************
 */
#ifndef __IR_SENSOR_H__
#define __IR_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 * GPIO 引脚
 * ================================================================ */
#define IR_GPIO_PORT        GPIOD
#define IR_S1_PIN           GPIO_PIN_0   /* 左侧 */
#define IR_S2_PIN           GPIO_PIN_1   /* 中左 */
#define IR_S3_PIN           GPIO_PIN_2   /* 中右 */
#define IR_S4_PIN           GPIO_PIN_3   /* 右侧 */

#define IR_PIN_MASK  (IR_S1_PIN | IR_S2_PIN | IR_S3_PIN | IR_S4_PIN)
#define IR_CHANNEL_COUNT  4

/* ================================================================
 * 路口类型
 * ================================================================ */
typedef enum {
    JUNC_NONE   = 0,   /* 无路口 */
    JUNC_CROSS  = 1,   /* 十字/T字(全黑): 节点9,10 */
    JUNC_T_LEFT  = 2,  /* T字左支: 左侧有岔路 */
    JUNC_T_RIGHT = 3,  /* T字右支: 右侧有岔路 */
    JUNC_T_PASS  = 4,  /* 丁字直行(3黑1白): 经过节点4不转向 */
} IR_JunctionType;

/* ================================================================
 * 数据结构
 * ================================================================ */
typedef struct {
    uint8_t raw[IR_CHANNEL_COUNT];  /* 4路布尔值 */
    int8_t  active_count;           /* 检测到线数 (0~4) */
    float   position;               /* 加权位置 [-3,+3] */
    float   error;                  /* = 0 - position */
    uint8_t line_lost;              /* 0=在线 1=脱线 2=宽线/路口 */
    int8_t  last_dir;               /* 最后方向 */

    /* ---- 路口检测 ---- */
    IR_JunctionType junction;       /* 当前检测到的路口类型 */
    uint8_t  junction_confirmed;    /* 1=路口已确认 (可通知上层) */
    uint32_t all_black_start;       /* 四路全黑开始时刻 (ms) */
    uint32_t left_black_start;      /* 仅左侧先黑开始时刻 */
    uint32_t right_black_start;     /* 仅右侧先黑开始时刻 */

    /* 盲区恢复: 线夹在S1-S2或S3-S4之间时自动寻回 */
    uint8_t  last_pattern;          /* 上一次非0000的4路值 (bit3:0=S1~S4) */
    uint8_t  blind_zone;            /* 0=正常 1=线在S1-S2间 2=线在S3-S4间 */
} IR_SensorTypeDef;

/* ================================================================
 * API
 * ================================================================ */
void IR_Init(void);
void IR_Update(IR_SensorTypeDef *ir);

uint8_t IR_GetRaw(uint8_t channel);
int     IR_IsLineLost(const IR_SensorTypeDef *ir);
int     IR_IsJunction(const IR_SensorTypeDef *ir);       /* 是否在路口中心 */
IR_JunctionType IR_GetJunction(const IR_SensorTypeDef *ir); /* 路口类型 */
void    IR_ClearJunction(IR_SensorTypeDef *ir);           /* 离开路口后清除 */

#ifdef __cplusplus
}
#endif

#endif
