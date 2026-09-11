/**
 ******************************************************************************
 * @file    ultrasonic.h
 * @brief   HC-SR04 超声波测距模块驱动 (TIM 输入捕获)
 *
 * 工作原理:
 *   1. TRIG 发 >=10μs 高电平脉冲
 *   2. 模块发射 8 个 40kHz 超声波
 *   3. ECHO 引脚拉高, 收到回波后拉低
 *   4. ECHO 高电平时间 × 声速 / 2 = 距离
 *
 * 距离公式:
 *   distance_cm = (echo_us × 0.0343) / 2 = echo_us / 58.3
 *   温度补偿: 声速 ≈ 331.4 + 0.607×T(℃), 0.0343 对应 ~25℃
 *
 * 技术指标:
 *   量程: 2cm - 400cm
 *   精度: ±3mm
 *   测距间隔: >= 60ms
 *
 * 硬件引脚 (可修改):
 *   TRIG: PA8  (GPIO 推挽输出)
 *   ECHO: PA1  (TIM5_CH2 输入捕获)
 *   注意: HC-SR04 需 5V 供电, ECHO 输出 5V, STM32H7 引脚部分 5V 耐受
 ******************************************************************************
 */
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 * 引脚 + 定时器配置 — 按实际接线修改
 * ================================================================ */
#define US_TRIG_PORT        GPIOA
#define US_TRIG_PIN         GPIO_PIN_8

#define US_ECHO_PORT        GPIOA
#define US_ECHO_PIN         GPIO_PIN_1

#define US_TIM              (&htim5)       /* TIM5 句柄 */
#define US_TIM_CHANNEL      TIM_CHANNEL_2  /* PA1 = TIM5_CH2 */
#define US_TIM_ACTIVE_CH    HAL_TIM_ACTIVE_CHANNEL_2

/* ================================================================
 * 测距参数
 * ================================================================ */
/* 声速 (cm/μs) @ 25℃, 除以 2 (往返) */
#define US_SPEED_CM_PER_US   0.01715f    /* = 0.0343 / 2 */

/* 有效距离范围 */
#define US_DIST_MIN_CM       2.0f
#define US_DIST_MAX_CM       400.0f

/* 超时: ECHO 超过 38ms(约 650cm)判定无回波 */
#define US_TIMEOUT_US        38000U

/* 最小测量间隔 */
#define US_TRIG_INTERVAL_MS  60U

/* ================================================================
 * 数据结构
 * ================================================================ */
typedef struct {
    float   distance_cm;        /* 最新测距结果 (cm) */
    uint8_t valid;              /* 1=有效 0=超时/无回波 */
    uint8_t busy;               /* 1=测量进行中 */
    uint32_t last_trig_tick;    /* 上次触发时刻 (ms) */
    uint32_t edge_rising;       /* 上升沿捕获值 */
} UltrasonicTypeDef;

/* ================================================================
 * API
 * ================================================================ */
void US_Init(void);                                   /* 初始化 GPIO + TIM5 */
void US_Trigger(void);                                /* 触发一次测距 */
void US_IRQ_Handler(void);                            /* TIM 捕获中断 (放 stm32h7xx_it.c) */
float US_Read(void);                                  /* 读最新距离 (cm), 无效返回 -1 */
int US_IsValid(void);                                 /* 最新数据是否有效 */
void US_StartMeasurement(void);                       /* 触发 + 等待完成 (阻塞 ~65ms max) */
float US_GetDistance(void);                           /* = US_Read() 别名 */

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H__ */
