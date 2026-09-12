#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 * 二自由度云台: 输入源 → 目标角度
 *
 *   电位器模式: PC2 (ADC3_INP0) → 偏航, PC3 (ADC3_INP1) → 俯仰
 *   MPU6050   : 加速度计算俯仰, 陀螺仪 Z 轴积分算偏航
 *   PE3 按键  : 按下(接地)切换输入源, 同时翻转 PC5 指示灯
 *   输出      : 文本 "偏航,俯仰" 经 printf → USART3 (PD8), 115200
 *
 *   符号约定: 正 = 右转(偏航) / 向下俯冲(俯仰)
 * ================================================================ */

/* 两轴角度限幅 (度) */
#define GIMBAL_ANGLE_LIMIT   30.0f

/* 控制节拍 (ms): 采样 + 按键扫描 */
#define GIMBAL_TICK_MS       10u

/* 上报周期 (ms): 20Hz, 终端看着不刷屏, 也够画曲线 */
#define GIMBAL_SEND_MS       50u

/* 按键消抖: 连续 N 次采样一致才认 (N × GIMBAL_TICK_MS = 30ms) */
#define GIMBAL_KEY_DEBOUNCE  3u

/* ----------------------------------------------------------------
 * 极性标定: 上电实测, 哪个轴方向反了就把对应符号改成 -1.0f
 * ---------------------------------------------------------------- */
#define GIMBAL_POT_YAW_SIGN    (+1.0f)
#define GIMBAL_POT_PITCH_SIGN  (+1.0f)
#define GIMBAL_MPU_PITCH_SIGN  (+1.0f)
#define GIMBAL_GYRO_SIGN       (+1.0f)

/* 静止死区 (°/s): 抑制陀螺仪零偏积分漂移 */
#define GIMBAL_GYRO_DEADBAND   (0.5f)

/* 1 = 启动 HC06 蓝牙 (HC06_Init 内含 1s 阻塞延时)
 * 本步输出走上位机 USART3, 不消费蓝牙数据, 所以默认关闭 */
#define GIMBAL_HC06_ENABLE    0

/* 输入源 */
typedef enum {
    GIMBAL_SRC_POT = 0,   /* 两个可调电位器 */
    GIMBAL_SRC_MPU = 1    /* MPU6050 */
} Gimbal_Source;

/* 初始化: 复位状态, 点亮/熄灭指示灯, 初始化 MPU6050(含静止校准) */
void Gimbal_Init(void);

/* 每 GIMBAL_TICK_MS 调用一次: 扫描按键, 采样当前输入源, 更新目标角度 */
void Gimbal_Update(void);

/* 每 GIMBAL_SEND_MS 调用一次: 把目标角度以 "偏航,俯仰" 文本发给上位机 */
void Gimbal_Report(void);

Gimbal_Source Gimbal_GetSource(void);

/* 取当前目标角度 (度), 任一指针可为 NULL */
void Gimbal_GetTarget(float *yaw, float *pitch);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_H__ */
