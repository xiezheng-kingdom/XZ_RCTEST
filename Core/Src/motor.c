/**
 ******************************************************************************
 * @file    motor.c
 * @brief   双电机 TB6612FNG 驱动 + 520 编码器 RPM 测量
 *
 * TB6612FNG 真值表 (以左/A通道为例):
 *   AIN1(PC1)  AIN2(PC0)  PWMA(PA0)   效果
 *      0          0          X         停止(滑行)
 *      1          0         PWM        正转(CW / 前进)
 *      0          1         PWM        反转(CCW / 后退)
 *      1          1          X         刹车(短路制动)
 *
 * ⚠️ PWM 参数检查点 — 修改频率前必须按下面公式验算:
 *
 *   PWM_Hz = TIM2_CLK / (PSC+1) / (ARR+1)
 *
 *   时钟链: HSE 25MHz → PLL → SYSCLK 400MHz → HCLK 200MHz → TIMxCLK 200MHz
 *
 *   PSC=0, ARR=7999 → 200MHz / 1 / 8000 = 25kHz  ← 当前正确值 (520电机推荐25~60kHz)
 *
 *   历史错误: PSC=199, ARR=19999 → 200M/200/20000=50Hz (舵机频率, 适合舵机但导致直流电机低频脉动大电流)
 *
 * 编码器参数:
 *   左: TIM1_CH1(PE9) + CH2(PE11), TI12 4倍频模式
 *   右: TIM3_CH1(PA6) + CH2(PA7)
 *   PPR=448, 4×=1792 脉冲/电机轴转
 *   轮径=65mm, 周长≈204.2mm, 10cm≈878 编码器计数
 *
 *   RPM 公式:  delta / (4×PPR) / (dt_sec)
 *   m/s 公式:  RPM × π × D_mm / 60000
 ******************************************************************************
 */
#include "motor.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"

/* ================== 左电机 (TB6612 A通道) ================== */
#define AIN1_PORT   GPIOC
#define AIN1_PIN    GPIO_PIN_1        /* PC1 → AIN1 */
#define AIN2_PORT   GPIOC
#define AIN2_PIN    GPIO_PIN_0        /* PC0 → AIN2 */
#define PWMA_TIM    &htim2
#define PWMA_CH     TIM_CHANNEL_1     /* PA0, AF1 */

/* ================== 右电机 (TB6612 B通道) ================== */
#define BIN1_PORT   GPIOA
#define BIN1_PIN    GPIO_PIN_4        /* PA4 → BIN1 */
#define BIN2_PORT   GPIOA
#define BIN2_PIN    GPIO_PIN_5        /* PA5 → BIN2 */
#define PWMB_TIM    &htim2
#define PWMB_CH     TIM_CHANNEL_2     /* PB3, AF1 (复用JTDO, 需SWD模式) */

/* ================== 编码器 ================== */
#define LEFT_ENC_TIM    &htim1        /* PE9/PE11, AF1 */
#define RIGHT_ENC_TIM   &htim3        /* PA6/PA7, AF2  */

/* RPM 计算: 100ms 间隔 */
#define RPM_INTERVAL_MS  100

static int32_t  last_left_count;
static int32_t  last_right_count;
static uint32_t last_rpm_tick;

static float left_rpm;
static float right_rpm;

/* 位置/速度闭环：32 位累计位置 + 速度 (counts/s, 100ms 窗口测速) */
static int32_t  left_pos32;
static int32_t  right_pos32;
static uint16_t left_last_cnt;
static uint16_t right_last_cnt;
static float    left_vel;
static float    right_vel;

/* ================== 初始化 ================== */
void Motor_Init(void)
{
    /* 全部方向引脚拉低，防止上电瞬间误触发 */
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(PWMA_TIM, PWMA_CH);
    HAL_TIM_PWM_Start(PWMB_TIM, PWMB_CH);
}

/* ================== 编码器 ================== */
void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(LEFT_ENC_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(RIGHT_ENC_TIM, TIM_CHANNEL_ALL);

    last_left_count  = (int32_t)__HAL_TIM_GET_COUNTER(LEFT_ENC_TIM);
    last_right_count = (int32_t)__HAL_TIM_GET_COUNTER(RIGHT_ENC_TIM);
    last_rpm_tick    = HAL_GetTick();

    left_last_cnt  = (uint16_t)__HAL_TIM_GET_COUNTER(LEFT_ENC_TIM);
    right_last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(RIGHT_ENC_TIM);
    left_pos32  = 0;
    right_pos32 = 0;
    left_vel    = 0.0f;
    right_vel   = 0.0f;
}

int32_t Encoder_GetLeftCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(LEFT_ENC_TIM);
}

int32_t Encoder_GetRightCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(RIGHT_ENC_TIM);
}

/* ================== 位置/速度闭环反馈 ================== */
/* 控制节拍内调用（周期 = ENC_UPDATE_MS，与 motor_control 一致） */
#define ENC_UPDATE_MS       10      /* ms (控制周期) */
#define VEL_EMA_A           0.60f   /* 速度低通系数, τ≈17ms (减小反馈滞后, 增加速度环阻尼) */

void Encoder_Update(void)
{
    uint16_t cnt;
    int16_t  delta;

    /* 左: 位置累积 (每 10ms) + 瞬时速度 EMA 低通 */
    cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(LEFT_ENC_TIM);
    delta = (int16_t)(cnt - left_last_cnt);
    left_last_cnt = cnt;
    left_pos32   += delta;
    left_vel     += VEL_EMA_A * ((float)delta * (1000.0f / (float)ENC_UPDATE_MS) - left_vel);

    /* 右 */
    cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(RIGHT_ENC_TIM);
    delta = (int16_t)(cnt - right_last_cnt);
    right_last_cnt = cnt;
    right_pos32   += delta;
    right_vel     += VEL_EMA_A * ((float)delta * (1000.0f / (float)ENC_UPDATE_MS) - right_vel);
}

int32_t Encoder_GetLeftPosition(void)  { return left_pos32; }
int32_t Encoder_GetRightPosition(void) { return right_pos32; }
float   Encoder_GetLeftVelocity(void)  { return left_vel; }
float   Encoder_GetRightVelocity(void) { return right_vel; }

/* ================== 速度计算 ================== */

float Motor_GetLeftRPM(void)  { return left_rpm; }
float Motor_GetRightRPM(void) { return right_rpm; }

/* RPM → m/s: 轮周长 = π×D_mm,  1 m/s = 60000 / (π×D_mm) RPM */
#define RPM_TO_MS(rpm)  ((rpm) * 3.1415926f * WHEEL_DIAMETER_MM / 60000.0f)

float Motor_GetLeftSpeedMS(void)  { return RPM_TO_MS(left_rpm); }
float Motor_GetRightSpeedMS(void) { return RPM_TO_MS(right_rpm); }

void Motor_UpdateRPM(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - last_rpm_tick;
    if (dt < RPM_INTERVAL_MS) return;

    /* OLED 刷新可能阻塞主循环, >800ms 跳过避免 dt 溢出 */
    if (dt > 800) {
        last_left_count  = (int32_t)__HAL_TIM_GET_COUNTER(LEFT_ENC_TIM);
        last_right_count = (int32_t)__HAL_TIM_GET_COUNTER(RIGHT_ENC_TIM);
        last_rpm_tick = now;
        return;
    }

    int32_t cur_left  = Encoder_GetLeftCount();
    int32_t cur_right = Encoder_GetRightCount();

    /* 16位差值自动处理溢出 (65535→0 回绕) */
    int16_t delta_left  = (int16_t)((uint16_t)cur_left  - (uint16_t)last_left_count);
    int16_t delta_right = (int16_t)((uint16_t)cur_right - (uint16_t)last_right_count);

    /* RPM = delta / (4×PPR) / (dt/60000)
     *     = delta × factor,  factor = 60000 / (4×PPR×dt_ms) */
    float factor = 60000.0f / (4.0f * ENCODER_PPR * (float)dt);
    float raw_l  = (float)delta_left  * factor;
    float raw_r  = (float)delta_right * factor;

    /* EMA 低通滤波, 抑制编码器高频毛刺 */
    #define EMA_A 0.3f
    left_rpm  = EMA_A * raw_l + (1.0f - EMA_A) * left_rpm;
    right_rpm = EMA_A * raw_r + (1.0f - EMA_A) * right_rpm;

    last_left_count  = cur_left;
    last_right_count = cur_right;
    last_rpm_tick    = now;
}

/* ================== 左电机 ================== */

void Motor_Left_Forward(uint16_t duty)
{
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET); /* IN2=0 */
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);   /* IN1=1 → CW */
    __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_CH, duty);
}

void Motor_Left_Reverse(uint16_t duty)
{
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET); /* IN1=0 */
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);   /* IN2=1 → CCW */
    __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_CH, duty);
}

void Motor_Left_Stop(void)
{
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET); /* IN1=0 */
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET); /* IN2=0 → 滑行 */
    __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_CH, 0);
}

void Motor_Left_Brake(void)
{
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET); /* IN1=1 */
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET); /* IN2=1 → 短路制动 */
    __HAL_TIM_SET_COMPARE(PWMA_TIM, PWMA_CH, 0);
}

/* 带符号 duty 驱动：正=正转，负=反转，|duty|<=死区=停止 */
#define MOTOR_DEADZONE  50

void Motor_Left_Drive(int32_t duty)
{
    int32_t max = (int32_t)__HAL_TIM_GET_AUTORELOAD(PWMA_TIM);
    if (duty >  max) duty =  max;
    if (duty < -max) duty = -max;

    if (duty > MOTOR_DEADZONE) {
        Motor_Left_Forward((uint16_t)duty);
    } else if (duty < -MOTOR_DEADZONE) {
        Motor_Left_Reverse((uint16_t)(-duty));
    } else {
        Motor_Left_Stop();
    }
}

/* ================== 右电机 ================== */

void Motor_Right_Forward(uint16_t duty)
{
    /* 右电机接线反相, Forward=IN1低/IN2高 */
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET); /* IN1=0 */
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);   /* IN2=1 */
    __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_CH, duty);
}

void Motor_Right_Reverse(uint16_t duty)
{
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET); /* IN2=0 */
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);   /* IN1=1 */
    __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_CH, duty);
}

void Motor_Right_Stop(void)
{
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_CH, 0);
}

void Motor_Right_Brake(void)
{
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET); /* IN1=1 */
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET); /* IN2=1 → 短路制动 */
    __HAL_TIM_SET_COMPARE(PWMB_TIM, PWMB_CH, 0);
}

void Motor_Right_Drive(int32_t duty)
{
    int32_t max = (int32_t)__HAL_TIM_GET_AUTORELOAD(PWMB_TIM);
    if (duty >  max) duty =  max;
    if (duty < -max) duty = -max;

    if (duty > MOTOR_DEADZONE) {
        Motor_Right_Forward((uint16_t)duty);
    } else if (duty < -MOTOR_DEADZONE) {
        Motor_Right_Reverse((uint16_t)(-duty));
    } else {
        Motor_Right_Stop();
    }
}

/* ================== 组合运动 ================== */

void Move_Forward(uint16_t duty)
{
    Motor_Left_Forward(duty);
    Motor_Right_Forward(duty);
}

void Move_Reverse(uint16_t duty)
{
    Motor_Left_Reverse(duty);
    Motor_Right_Reverse(duty);
}

void Move_Stop(void)
{
    Motor_Left_Stop();
    Motor_Right_Stop();
}

/* ================== ADC 调速 ================== */

/* 采样值低于该阈值视为松开油门，直接停车，避免 ADC 噪声让电机微动 */
#define ADC_IDLE_THRESHOLD  50

/**
 * @brief 根据 ADC 采样值设定电机转速（采样值越高，转速越大）
 * @param adc 12 位采样值 (0~4095)，来自 PC2(ADC3_INP0) 或 PC3(ADC3_INP1)
 *
 * 映射: duty = adc × ARR / 4095
 *   adc = 0    → 停止
 *   adc = 4095 → 满占空比 (ARR = TIM2 Period = 7999)
 */
void Motor_SetSpeedFromADC(uint16_t adc)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWMA_TIM);

    if (adc < ADC_IDLE_THRESHOLD) {
        Move_Stop();
        return;
    }

    uint32_t duty = (uint32_t)adc * arr / 4095u;
    if (duty > arr) {
        duty = arr;
    }

    Move_Forward((uint16_t)duty);
}

/**
 * @brief 根据 ADC 电压值设定电机转速（电压越高，转速越大）
 * @param voltage ADC 电压 (V)，范围 0 ~ ADC_REF_VOLTAGE(3.3V)
 */
void Motor_SetSpeedFromVoltage(float voltage)
{
    if (voltage < 0.0f)            voltage = 0.0f;
    if (voltage > ADC_REF_VOLTAGE) voltage = ADC_REF_VOLTAGE;

    uint16_t adc = (uint16_t)(voltage * 4095.0f / ADC_REF_VOLTAGE);
    Motor_SetSpeedFromADC(adc);
}
