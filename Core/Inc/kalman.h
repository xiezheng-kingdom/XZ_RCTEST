/**
 ******************************************************************************
 * @file    kalman.h
 * @brief   卡尔曼滤波器: 1D 速度 + 2D 位置-速度
 *
 * Kalman1D — 融合加速度+编码器, 输出速度 (原版, 保持兼容)
 * Kalman2D — 融合加速度+编码器+超声波, 输出位置+速度
 *
 * MPU6050 安装方向 (芯片丝印面朝上, 俯视小车):
 *   ┌──────────────┐      小车前进方向 →
 *   │  ┌──┐        │      ╔══════════════╗
 *   │  │○ │  ← X   │ 车尾 ║   MPU6050    ║ 车头
 *   │  └──┘  ↓     │      ║  X→车后      ║
 *   │         Y    │      ║  Y→车右      ║
 *   │              │      ╚══════════════╝
 *   │   MPU6050    │
 *   └──────────────┘
 *   X轴 = 指向车尾 (后方), Y轴 = 指向车右, Z轴 = 指向天空
 *   前进加速时: 惯性质量后移(+X方向), ax 读正值 → 需要 -ax*g 得前进加速度
 *
 * 2D 状态: [x, v]  (位置, 速度)
 * 预测:   x = x + v*dt          里程积分
 *         v = v + ax*dt          加速度积分 (ax 需预先取反, MPU6050前进时读负值)
 * 更新A:  z = encoder_speed     观测速度 (纠正 v, 同时通过Kx修正x)
 * 更新B:  z = ultrasonic_dist   观测位置 (纠正 x, 消除累积漂移)
 *
 * ⚠️ 实测标定 (2026-07-01, 双电机 JGB37-520, 轮径65mm):
 *   - 仅有加速度+编码器融合(无超声波位置修正)时, KF位置输出约为真实距离的8倍
 *   - 推荐参数: Qx=0.0001, Qv=0.1, R_enc=0.5
 *   - 最低启动 PWM 占空比: 2820~3760 counts (14~19% @ 50Hz, 随电池电压波动)
 *   - 可靠行驶 PWM: >= 3760 counts (19%) 向上取整保证启动成功
 *
 * 调用流程:
 *   Kalman2D_Init(&kf, Qx, Qv, R_enc, R_us);
 *   // 每 10ms:  Kalman2D_Predict(&kf, -accel_x_g * 9.8f, dt);
 *   // 每循环:   Kalman2D_UpdateVel(&kf, encoder_speed_ms);
 *   // 位置可用时: Kalman2D_UpdatePos(&kf, position_m);
 ******************************************************************************
 */
#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 * Kalman1D — 单变量速度滤波器 (保持向后兼容)
 * ================================================================ */
typedef struct {
    float v;     /* 速度估计值 (m/s) */
    float P;     /* 估计协方差 */
    float Q;     /* 过程噪声 */
    float R;     /* 测量噪声 */
    float K;     /* 卡尔曼增益 (只读) */
} Kalman1D;

void Kalman_Init   (Kalman1D *kf, float Q, float R);
void Kalman_Predict(Kalman1D *kf, float ax, float dt);
void Kalman_Update (Kalman1D *kf, float z);

/* ================================================================
 * Kalman2D — 位置-速度滤波器
 * ================================================================ */
typedef struct {
    /* 状态 */
    float x;           /* 位置估计 (m), 标定后需 /8 得真实距离 */
    float v;           /* 速度估计 (m/s) */

    /* 协方差矩阵 [2×2] */
    float p_xx, p_xv;
    float p_vx, p_vv;

    /* 过程噪声 */
    float Q_x;         /* 位置过程噪声 (推荐 0.0001) */
    float Q_v;         /* 速度过程噪声 (推荐 0.1) */

    /* 测量噪声 */
    float R_enc;       /* 编码器速度测量噪声 (推荐 0.5) */
    float R_us;        /* 超声波位置测量噪声 (推荐 0.01, 米制) */

    /* 增益 (只读, 调试用) */
    float K_enc;        /* 编码器更新时的最大增益 */
    float K_us;         /* 超声波更新时的最大增益 */
} Kalman2D;

/* 初始化: Q_x=0.0001, Q_v=0.1, R_enc=0.5, R_us=99(不用超声波则设大值) */
void Kalman2D_Init(Kalman2D *kf,
                   float Q_x,  float Q_v,
                   float R_enc, float R_us);

/* 预测步: ax 单位为 m/s², 调用前需取反 (前进时 ax 为负) */
void Kalman2D_Predict(Kalman2D *kf, float ax, float dt);

/* 更新步A: encoder_v 为编码器测速 (m/s), 双轮取平均 */
void Kalman2D_UpdateVel(Kalman2D *kf, float encoder_v);

/* 更新步B: 超声波/二维码定位位置观测 (m) */
void Kalman2D_UpdatePos(Kalman2D *kf, float position_m);

/* 重置状态, 保留噪声参数 */
void Kalman2D_Reset(Kalman2D *kf);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H__ */
