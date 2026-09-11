/**
 ******************************************************************************
 * @file    ir_sensor.c
 * @brief   4路红外巡线 + 路口类型判定
 *
 * 路口判定逻辑:
 *   1. 接近路口: S1(左) 或 S4(右) 先单独检测到黑线分支
 *      → 记录 left_black_start / right_black_start
 *   2. 到达路口中心: 4路全黑
 *      → 计时, 50~600ms 内确认路口
 *   3. 判断类型:
 *      - 左侧先黑 + 全黑 → T_LEFT
 *      - 右侧先黑 + 全黑 → T_RIGHT
 *      - 直接全黑(无左右预警) → CROSS
 *   4. 超时或离开路口 → 清除
 ******************************************************************************
 */
#include "ir_sensor.h"

/* 权重 */
static const int8_t ir_weight[IR_CHANNEL_COUNT] = {-3, -1, 1, 3};
static const uint16_t ir_pin[IR_CHANNEL_COUNT] = {
    IR_S1_PIN, IR_S2_PIN, IR_S3_PIN, IR_S4_PIN
};

/* 路口检测阈值 */
#define JUNC_ALL_BLACK_MIN_MS   50    /* 全黑持续 ≥50ms 才确认路口 */
#define JUNC_ALL_BLACK_MAX_MS   800   /* 全黑 ≥800ms 大概是停机不是路口 */
#define JUNC_SIDE_WARN_MS       30    /* 单侧黑 ≥30ms 记录预警 */

/* ====================== GPIO 初始化 ====================== */
void IR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin  = IR_PIN_MASK;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IR_GPIO_PORT, &GPIO_InitStruct);
}

/* ====================== 数据采集 + 路口判定 ====================== */
void IR_Update(IR_SensorTypeDef *ir)
{
    if (ir == NULL) return;

    uint32_t now = HAL_GetTick();
    uint16_t idr = IR_GPIO_PORT->IDR;
    int32_t weighted_sum = 0;
    uint8_t active_count  = 0;

    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; i++) {
        uint8_t state = (idr & ir_pin[i]) ? 1 : 0;  /* HIGH=黑 */
        ir->raw[i] = state;
        if (state) { weighted_sum += ir_weight[i]; active_count++; }
    }

    ir->active_count = active_count;
    if (active_count > 0)
        ir->position = (float)weighted_sum / (float)active_count;
    else
        ir->position = 0.0f;

    ir->error = 0.0f - ir->position;

    /* ---- 盲区恢复: 直线段线夹在传感器间隙时自动寻回 ---- */
    uint8_t pat = (ir->raw[0]<<3)|(ir->raw[1]<<2)|(ir->raw[2]<<1)|ir->raw[3];
    if (active_count == 0 && ir->last_pattern == 0x4) {  /* 0100→0000: 线在S1-S2间 */
        ir->blind_zone = 1;  /* 需向S1方向(左)转弯寻线 */
    } else if (active_count == 0 && ir->last_pattern == 0x2) {  /* 0010→0000: 线在S3-S4间 */
        ir->blind_zone = 2;  /* 需向S4方向(右)转弯寻线 */
    } else if (active_count > 0) {
        ir->blind_zone = 0;  /* 在线, 清除 */
        ir->last_pattern = pat;
    }

    /* ---- 基本失线判定 ---- */
    if (active_count == 4) {
        ir->line_lost = 2;  /* 全黑 = 路口中心 */
    } else if (active_count == 0) {
        ir->line_lost = 1;  /* 全白 = 脱线 */
    } else {
        ir->line_lost = 0;
        if      (ir->position < -0.5f) ir->last_dir = -1;
        else if (ir->position >  0.5f) ir->last_dir =  1;
        else                           ir->last_dir =  0;
    }

    /* ============================================================
     * 路口状态机
     * ============================================================ */
    uint8_t s1 = ir->raw[0], s2 = ir->raw[1];
    uint8_t s3 = ir->raw[2], s4 = ir->raw[3];
    uint8_t all_black  = (active_count == 4);
    uint8_t black3     = (active_count == 3);  /* 3黑1白 = 经过丁字路口不转 */
    uint8_t left_only  = (s1 && !s4 && active_count <= 3);
    uint8_t right_only = (!s1 && s4 && active_count <= 3);

    /* --- 路口预警: 单侧先看到黑线分支 --- */
    if (!all_black) {
        if (left_only) {
            if (ir->left_black_start == 0)
                ir->left_black_start = now;
        } else {
            ir->left_black_start = 0;
        }
        if (right_only) {
            if (ir->right_black_start == 0)
                ir->right_black_start = now;
        } else {
            ir->right_black_start = 0;
        }
    }

    /* --- 路口中心: 全黑(节点9/10) 或 3黑1白(节点4直行) --- */
    if (all_black || black3) {
        if (ir->all_black_start == 0)
            ir->all_black_start = now;

        uint32_t dur = now - ir->all_black_start;
        if (dur >= JUNC_ALL_BLACK_MIN_MS && !ir->junction_confirmed) {
            ir->junction_confirmed = 1;

            if (all_black) {
                /* 全黑 = 节点9/10 的T字/十字 */
                uint8_t had_left  = (ir->left_black_start  > 0);
                uint8_t had_right = (ir->right_black_start > 0);
                if (had_left && had_right)      ir->junction = JUNC_CROSS;
                else if (had_left)              ir->junction = JUNC_T_LEFT;
                else if (had_right)             ir->junction = JUNC_T_RIGHT;
                else                            ir->junction = JUNC_CROSS;
            } else {
                /* 3黑1白 = 经过节点4(丁字路口)不转向
                 * 白在S4 → 9→4→10方向  白在S1 → 10→4→9方向 */
                if (!s4) ir->junction = JUNC_T_PASS;  /* S4白=右无分支 */
                else     ir->junction = JUNC_T_PASS;  /* S1白=左无分支 */
            }

            ir->left_black_start  = 0;
            ir->right_black_start = 0;
        }
    } else {
        /* 退出全黑状态 */
        if (ir->all_black_start > 0) {
            uint32_t dur = now - ir->all_black_start;
            if (dur >= JUNC_ALL_BLACK_MAX_MS)
                ir->junction = JUNC_NONE;   /* 长时间全黑不是路口 */
        }
        /* 离开路口后重置 */
        if (ir->junction_confirmed) {
            IR_ClearJunction(ir);
        }
        ir->all_black_start = 0;
    }
}

/* ====================== 查询 ====================== */

uint8_t IR_GetRaw(uint8_t channel)
{
    if (channel >= IR_CHANNEL_COUNT) return 0;
    return ((IR_GPIO_PORT->IDR & ir_pin[channel]) == 0) ? 1 : 0;
}

int IR_IsLineLost(const IR_SensorTypeDef *ir)
{
    return (ir != NULL) ? (int)ir->line_lost : 1;
}

int IR_IsJunction(const IR_SensorTypeDef *ir)
{
    return (ir != NULL) ? (int)ir->junction_confirmed : 0;
}

IR_JunctionType IR_GetJunction(const IR_SensorTypeDef *ir)
{
    return (ir != NULL) ? ir->junction : JUNC_NONE;
}

void IR_ClearJunction(IR_SensorTypeDef *ir)
{
    if (ir == NULL) return;
    ir->junction           = JUNC_NONE;
    ir->junction_confirmed = 0;
    ir->all_black_start    = 0;
    ir->left_black_start   = 0;
    ir->right_black_start  = 0;
}
