/**
 ******************************************************************************
 * @file    cargo.h
 * @brief   仓储物流 — 基于地图节点 + 循线 + QR定位的导航系统
 *
 * 地图拓扑 (黑色循迹线, 每个节点有QR码):
 *
 *          1                    6
 *          |                    |
 *      2───9────4────10────7
 *          |    │            │
 *          3    5            8
 *               ↑
 *             丁字路口(5从下方接入)
 *
 *   水平线: 2─9─4─10─7     4=丁字路口(正下=5), 5=出发
 *   左右对称于 4-5 轴: 1↔6, 2↔7, 3↔8, 9↔10
 *
 *   节点类型:
 *     1,2,3 = 储物台(STORE)      6 = 取货点(PICKUP)
 *     8     = 出货点(DELIVER)    7 = 杂货区(FLEX)
 *     9,10  = T字路口(CROSS)     4 = 丁字路口(T-JUNC)
 *     5     = 出发点(HOME)
 *
 *   邻接边 (双向, 距离 cm 需实测):
 *     左列: 1-2, 2-3    右列: 6-7, 7-8
 *     水平: 2-9, 9-4, 4-10, 10-7
 *     中下: 4-5
 *
 * 导航模式:
 *   1. 循线 (IR PD0~3) 保持车体在黑色轨迹上
 *   2. 到达节点时 QR 二维码定位修正
 *   3. 路口 (节点4) 处按预设方向转向
 *
 * QR定位基准:
 *   每个节点有固定QR码, K230扫描得小车相对QR的XY偏移(cm)
 *   存储时: 记录(节点, OX, OY) 作为该货物的"标准停车位"
 *   取货时: 到达节点→QR扫描当前(OX',OY')→与记录(OX,OY)比对
 *     |Δ| < 阈值 → 机械臂自行补偿
 *     |Δ| > 阈值 → 小车微调修正后重扫
 * 入库: 5→6(取货) → 储物台→QR扫描→记录→机械臂放置
 * 出库: 查记录→储物台→QR比对→修正→机械臂取货→8(出货)
 ******************************************************************************
 */
#ifndef __CARGO_H__
#define __CARGO_H__

#include "main.h"
#include "road_map.h"      /* 统一使用 road_map 的 MapNode 定义 */

typedef enum {
    NT_STORE   = 0,   /* 储物台 (1,2,3) */
    NT_CROSS   = 1,   /* 路口 (4) */
    NT_HOME    = 2,   /* 出发点 (5) */
    NT_PICKUP  = 3,   /* 取货点 (6) */
    NT_FLEX    = 4,   /* 杂货区 (7, 可配置) */
    NT_DELIVER = 5,   /* 出货点 (8) */
} NodeType;

/* ================================================================
 * 邻接边
 * ================================================================ */
typedef struct {
    MapNode  from;
    MapNode  to;
    float    dist_cm;      /* 编码器测距 cm */
    uint8_t  ir_action;    /* 路口转向动作: 0=直行 1=左转 2=右转 3=掉头 */
} MapEdge;

#define MAP_EDGE_MAX  16
extern const MapEdge g_map_edges[];
extern const uint8_t g_map_edge_count;

/* ================================================================
 * 货物记录
 * ================================================================ */
#define CARGO_ID_LEN      12
#define CARGO_COLOR_LEN   8
#define CARGO_DB_MAX      32

/* QR 定位偏差阈值 */
#define QR_ARM_OK_CM      3.0f   /* ≤3cm 机械臂自行补偿 */
#define QR_CAR_FIX_CM     5.0f   /* >5cm 小车微调修正 */
#define CARGO_MATCH_MAX_CM 8.0f  /* K230货物匹配最大距离 */

/* 电池模拟: 初始100%, 每节点间边消耗1%, 阈值 */
#define BAT_PER_EDGE      1      /* 每条边消耗电量 % */
#define BAT_LOW_PCT       10     /* <10% 低电: 拒绝任务 */
#define BAT_MED_PCT       75     /* <75% 中等: 提醒但仍执行 */

typedef struct {
    char     id[CARGO_ID_LEN];
    char     color[CARGO_COLOR_LEN];
    MapNode  shelf_node;        /* 储物台节点号 */
    /* 存储时 K230 模式1 扫描QR 记录小车停车位 (OX0,OY0) */
    float    car_ox, car_oy;
    /* 存储时 K230 模式2 扫描货物 记录货物放置位 (OX0',OY0') */
    float    cargo_ox, cargo_oy;
    /* 不变向量: Δ = (cargo_ox - car_ox, cargo_oy - car_oy)
     * 取货时: QR得(OX1,OY1) → 预期货物 = (OX1+Δx, OY1+Δy)
     *        再扫描货物列表 → 匹配最近 → 确认身份 */
    uint8_t  stored;            /* 1=在库 */
} CargoRecord;

/* ================================================================
 * 系统状态
 * ================================================================ */
typedef enum {
    CS_IDLE = 0,
    CS_NAV_TO,           /* 正在导航去目标节点 */
    CS_AT_NODE,          /* 到达节点, 等待动作 */
    CS_ARM_WAIT,         /* 等待机械臂完成 */
    CS_DONE,             /* 动作完成, 回复HC06 + 启程返航 */
    CS_RETURN,           /* 返航中: 回出发点5 */
    CS_RETURN_DONE,      /* 回到5, 掉头面向4 */
    CS_LOW_BATTERY,      /* 低电量: 退回5等待充电 */
    CS_PENDING,          /* 有待执行指令, 等待充电完成 */
    CS_OBSTACLE,         /* 检测到路障(途中扫到货物) */
    CS_OBSTACLE_TO_7,    /* 转运路障到杂货台7 */
    CS_OBSTACLE_FAIL,    /* 拾取失败, 需人工介入 */
    CS_WAIT_RESUME,      /* 等待继续指令 */
    CS_ERROR,
} CargoState;

/* ================================================================
 * 指令类型
 * ================================================================ */
typedef enum {
    CMD_NONE = 0,
    CMD_STORE,           /* 入库: $ST,id,color,shelf(1/2/3) */
    CMD_RETRIEVE,        /* 出库: $GT,id */
    CMD_GOTO,            /* $GO,node */
    CMD_CHARGE,          /* $CHG,PCT 充电完成 */
    CMD_CONTINUE,        /* $CONT 继续(清障后恢复) */
} CmdType;

/* ================================================================
 * API
 * ================================================================ */
void Cargo_Init(void);
void Cargo_Run(void);
void Cargo_FeedByte(uint8_t byte);

/* 查询 */
CargoState  Cargo_GetState(void);
void        Cargo_GetStateStr(char *buf, uint8_t len);
MapNode     Cargo_GetCurrentNode(void);
MapNode     Cargo_GetTargetNode(void);
uint8_t     Cargo_GetDBCount(void);

/* 电池 (模拟) */
uint8_t     Cargo_GetBatteryPct(void);
uint8_t     Cargo_IsBatteryLow(void);

#endif /* __CARGO_H__ */
