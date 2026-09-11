/**
 ******************************************************************************
 * @file    kalman.c
 * @brief   卡尔曼滤波器: 1D 速度 + 2D 位置-速度 (矩阵手动展开)
 *
 * MPU6050 安装方向 (芯片丝印面朝上, 俯视小车):
 *   ┌──────────────┐      小车前进方向 →
 *   │  ┌──┐        │      ╔══════════════╗
 *   │  │○ │  ← X   │ 车尾 ║   MPU6050    ║ 车头
 *   │  └──┘  ↓     │      ║  X→车尾      ║
 *   │         Y    │      ║  Y→车右      ║
 *   │              │      ╚══════════════╝
 *   │   MPU6050    │
 *   └──────────────┘
 *   X轴 = 指向车尾, Y轴 = 指向车右, Z轴 = 天空
 *   前进加速: 质量后移 = +X → ax>0 → 前进加速度 = -ax*g
 *
 * ⚠️ 标定记录 (2026-07-01, PZtest 双电机 JGB37-520, 轮径65mm, 电池7.4V):
 *   仅有加速度+编码器融合(无位置观测)时, KF x 输出 ≈ 真实距离×8.
 *   原因是 PPR=448 编码器在电机轴而非输出轴, 实际减速比未计入.
 *   使用时需 kf.x / DIST_CALIB (=8.0) 转换为真实距离 (m).
 *   推荐噪声参数: Qx=0.0001, Qv=0.1, R_enc=0.5
 *
 *   超声波/二维码 UpdatePos 可直接消除此累积误差.
 *
 *   最低启动 PWM: 2820~3760 counts (14~19%, 随电池电压波动)
 *   可靠行驶: >=3760 counts (19%) 保证启动成功
 *
 * 数学模型:
 *   状态: [x, v]^T  (位置 m, 速度 m/s)
 *   状态转移: x_k = x_{k-1} + v_{k-1}·dt
 *            v_k = v_{k-1} + ax·dt
 *   转移矩阵 A = [1, dt; 0, 1]
 *
 *   速度观测: H=[0,1]  → z = encoder_speed (m/s)
 *   位置观测: H=[1,0]  → z = position_m
 *
 *   矩阵运算全部手动展开, 避免依赖外部数学库.
 ******************************************************************************
 */
#include "kalman.h"
#include <string.h>  /* memset */
#include <math.h>    /* fabsf */

/* ================================================================
 * 内部工具
 * ================================================================ */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ================================================================
 * Kalman1D — 单变量速度滤波器 (保持向后兼容)
 * ================================================================ */

void Kalman_Init(Kalman1D *kf, float Q, float R)
{
    if (kf == NULL) return;
    kf->v = 0.0f;
    kf->P = 1.0f;
    kf->Q = Q;
    kf->R = R;
    kf->K = 0.0f;
}

void Kalman_Predict(Kalman1D *kf, float ax, float dt)
{
    if (kf == NULL) return;
    /* 死区: ±30mg 以下视为静止噪声 */
    if (ax > -0.03f && ax < 0.03f) ax = 0.0f;
    kf->v += ax * dt;
    kf->P += kf->Q;
}

void Kalman_Update(Kalman1D *kf, float z)
{
    if (kf == NULL) return;
    kf->K  = kf->P / (kf->P + kf->R);
    kf->v += kf->K * (z - kf->v);
    kf->P *= (1.0f - kf->K);
}

/* ================================================================
 * Kalman2D — 位置-速度 (矩阵手动展开)
 *
 * 初始化参数说明:
 *   Q_x : 位置过程噪声, 静态时 x 漂移速度. 推荐 0.0001
 *   Q_v : 速度过程噪声, ax 积分误差. 推荐 0.1
 *   R_enc: 编码器测速噪声 (m²/s²). 推荐 0.5
 *   R_us : 位置观测噪声 (m²), 推荐 0.01(超声波)或 99(不用)
 * ================================================================ */

void Kalman2D_Init(Kalman2D *kf,
                   float Q_x, float Q_v,
                   float R_enc, float R_us)
{
    if (kf == NULL) return;
    memset(kf, 0, sizeof(*kf));

    /* 初始协方差 = 单位阵 (完全未知, 快速收敛) */
    kf->p_xx = 1.0f;
    kf->p_vv = 1.0f;
    kf->p_xv = 0.0f;
    kf->p_vx = 0.0f;

    kf->Q_x   = Q_x;
    kf->Q_v   = Q_v;
    kf->R_enc = R_enc;
    kf->R_us  = R_us;
}

void Kalman2D_Reset(Kalman2D *kf)
{
    if (kf == NULL) return;
    kf->x = 0.0f;
    kf->v = 0.0f;
    kf->p_xx = 1.0f;
    kf->p_vv = 1.0f;
    kf->p_xv = 0.0f;
    kf->p_vx = 0.0f;
}

/* ==================== 预测 ==================== */

void Kalman2D_Predict(Kalman2D *kf, float ax, float dt)
{
    if (kf == NULL) return;

    /* 死区: ±0.3m/s² 以下视为静止噪声
     * ax 已取反为前进加速度 (m/s²), X→车尾时前进加速 = -ax_raw */
    if (ax > -0.3f && ax < 0.3f) ax = 0.0f;

    /* 状态推进: x += v*dt, v += ax*dt */
    kf->x += kf->v * dt;
    kf->v += ax * dt;

    /* 协方差推进: P = A·P·A^T + Q
     * A = [1, dt; 0, 1]
     * A·P·A^T = [p_xx+2·dt·p_xv+dt²·p_vv,  p_xv+dt·p_vv;
     *            p_vx+dt·p_vv,                p_vv] */
    float dt2 = dt * dt;

    float old_pxv = kf->p_xv;
    float old_pvv = kf->p_vv;

    kf->p_xx = kf->p_xx + 2.0f * dt * old_pxv + dt2 * old_pvv + kf->Q_x;
    kf->p_xv = old_pxv + dt * old_pvv;
    kf->p_vx = kf->p_xv;   /* 对称 */
    kf->p_vv = old_pvv + kf->Q_v;
}

/* ==================== 速度更新 (编码器) ==================== */

void Kalman2D_UpdateVel(Kalman2D *kf, float encoder_v)
{
    if (kf == NULL) return;

    /* H = [0, 1]: 仅观测速度
     * 新息 y = z - v
     * 新息协方差 S = p_vv + R_enc
     * 卡尔曼增益 Kx = p_xv / S (位置修正)
     *              Kv = p_vv / S (速度修正) */
    float S = kf->p_vv + kf->R_enc;
    if (S < 1e-9f) return;

    float Kx = kf->p_xv / S;
    float Kv = kf->p_vv / S;

    float y = encoder_v - kf->v;

    /* 状态修正 */
    kf->x += Kx * y;
    kf->v += Kv * y;

    /* 协方差缩减: P = (I - K·H)·P
     * I-K·H = [1, -Kx; 0, 1-Kv] */
    kf->p_xx -= Kx * kf->p_vx;
    kf->p_xv -= Kx * kf->p_vv;
    kf->p_vx -= Kv * kf->p_vx;
    kf->p_vv -= Kv * kf->p_vv;

    kf->K_enc = Kv;
}

/* ==================== 位置更新 (超声波/二维码) ==================== */

void Kalman2D_UpdatePos(Kalman2D *kf, float position_m)
{
    if (kf == NULL) return;

    /* H = [1, 0]: 仅观测位置
     * 新息 y = z - x
     * 新息协方差 S = p_xx + R_us
     * 卡尔曼增益 Kx = p_xx / S
     *              Kv = p_vx / S */
    float S = kf->p_xx + kf->R_us;
    if (S < 1e-9f) return;

    float Kx = kf->p_xx / S;
    float Kv = kf->p_vx / S;

    float y = position_m - kf->x;

    /* 状态修正 */
    kf->x += Kx * y;
    kf->v += Kv * y;

    /* 协方差缩减 */
    kf->p_xx -= Kx * kf->p_xx;
    kf->p_xv -= Kx * kf->p_xv;
    kf->p_vx -= Kv * kf->p_vx;
    kf->p_vv -= Kv * kf->p_xv;

    /* 强制对称 */
    kf->p_xv = kf->p_vx;

    kf->K_us = Kx;
}
