/**
 ******************************************************************************
 * @file    motor.h
 * @brief   双电机 TB6612FNG 驱动 + 编码器速度测量
 *
 * TB6612FNG 真值表 (单通道):
 *   IN1  IN2  PWM    模式
 *    0    0    X     停止(滑行)  Motor_Stop()
 *    1    0   PWM    正转(CW)    Motor_Forward()
 *    0    1   PWM    反转(CCW)   Motor_Reverse()
 *    1    1    X     刹车        Motor_Brake() (未实现, 预留)
 *
 * 控制引脚 (双电机, 左右对称):
 *   左(A通道): AIN1=PC1  AIN2=PC0  PWMA=TIM2_CH1(PA0)
 *   右(B通道): BIN1=PA4  BIN2=PA5  PWMB=TIM2_CH2(PB3)
 *
 * ⚠️ PWM 频率验证公式 (修改 tim.c prescaler 前必须验算):
 *   PWM_Hz = TIM2_CLK / (PSC+1) / (ARR+1)
 *
 *   当前实际时钟: SYSCLK=128MHz → APB1=32MHz → TIM2_CLK=64MHz
 *   PSC=63, ARR=19999  →  64MHz / 64 / 20000 = 50Hz  ✓
 *   duty 范围: 0 ~ 19999
 *
 *   历史踩坑: PSC=199 时 PWM=16Hz, 烧毁了 STM32H743VIT6 PA0/PB3 引脚和 TB6612
 *
 * 编码器 (520 电机 JGB37-520):
 *   左: TIM1_CH1(PE9/A相) + TIM1_CH2(PE11/B相)
 *   右: TIM3_CH1(PA6/A相) + TIM3_CH2(PA7/B相)
 *   PPR=448, 4倍频=1792 P/R(电机轴), 减速比=18 → 车轮一圈=32256 counts
 *   轮径=65mm, 每圈距离 = π×65mm ≈ 204.2mm
 *   10cm ≈ 15796 编码器计数 (100/204.2×32256)
 ******************************************************************************
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* ---- 编码器参数 ---- */
#define ENCODER_PPR  448               /* 编码器线数 (CPR, per motor shaft rev) */
/* 4倍频 (TIM TI12 mode): 448×4 = 1792 counts/电机轴圈 */
#define GEAR_RATIO   18                /* 减速比: 车轮一圈 = 1792×18 = 32256 counts */

/* ---- 车轮参数 ---- */
#define WHEEL_DIAMETER_MM  65.0f       /* 轮径 65mm */
/* 轮周长 = π×65 ≈ 204.2mm */

void Motor_Init(void);
void Encoder_Start(void);

/* 左电机 (A通道: AIN1=PC1, AIN2=PC0, PWMA=TIM2_CH1=PA0) */
void Motor_Left_Forward(uint16_t duty);
void Motor_Left_Reverse(uint16_t duty);
void Motor_Left_Stop(void);
void Motor_Left_Brake(void);

/* 右电机 (B通道: BIN1=PA4, BIN2=PA5, PWMB=TIM2_CH2=PB3) */
void Motor_Right_Forward(uint16_t duty);
void Motor_Right_Reverse(uint16_t duty);
void Motor_Right_Stop(void);
void Motor_Right_Brake(void);

/* 组合运动 */
void Move_Forward(uint16_t duty);
void Move_Reverse(uint16_t duty);
void Move_Stop(void);

/* ADC 调速：采样值越高，转速越大 */
void Motor_SetSpeedFromADC(uint16_t adc);

/* 电压调速：电压越高，转速越大 */
void Motor_SetSpeedFromVoltage(float voltage);

/* 编码器 */
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void    Motor_UpdateRPM(void);
float   Motor_GetLeftRPM(void);
float   Motor_GetRightRPM(void);
float   Motor_GetLeftSpeedMS(void);
float   Motor_GetRightSpeedMS(void);

/* 位置/速度闭环接口（控制节拍内调用） */
void    Encoder_Update(void);
int32_t Encoder_GetLeftPosition(void);
int32_t Encoder_GetRightPosition(void);
float   Encoder_GetLeftVelocity(void);
float   Encoder_GetRightVelocity(void);

/* 带符号 duty 驱动（正=正转，负=反转） */
void Motor_Left_Drive(int32_t duty);
void Motor_Right_Drive(int32_t duty);

#endif
