#include "gimbal.h"
#include "adc.h"
#include "gpio.h"
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

#if GIMBAL_HC06_ENABLE
#include "hc06.h"
#endif

#define RAD2DEG  57.29578f

/* ---- 状态 ---- */
static Gimbal_Source s_src = GIMBAL_SRC_POT;
static float s_yaw   = 0.0f;
static float s_pitch = 0.0f;

/* ---- 按键消抖: 1 = 松开, 0 = 按下 ---- */
static uint8_t s_key_stable = 1;
static uint8_t s_key_count  = 0;

/* ----------------------------------------------------------------
 * 工具
 * ---------------------------------------------------------------- */

static float clamp_angle(float deg)
{
    if (deg >  GIMBAL_ANGLE_LIMIT) return  GIMBAL_ANGLE_LIMIT;
    if (deg < -GIMBAL_ANGLE_LIMIT) return -GIMBAL_ANGLE_LIMIT;
    return deg;
}

/* 12 位 ADC (0~4095) 线性映射到 -30~+30, 中点 2048 对应 0° */
static float pot_to_deg(uint16_t adc)
{
    return ((float)adc - 2048.0f) * (2.0f * GIMBAL_ANGLE_LIMIT) / 4095.0f;
}

/* 把 PC5 设成当前模式对应的状态: 亮 = MPU6050 模式 */
static void led_apply(void)
{
    HAL_GPIO_WritePin(GIMBAL_LED_PORT, GIMBAL_LED_PIN,
                      (s_src == GIMBAL_SRC_MPU) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------
 * 按键 PE3: 消抖 + 按下沿切换模式
 * ---------------------------------------------------------------- */
static void key_scan(void)
{
    /* 上拉输入, 按下接地 → 读到 RESET(0) 即按下 */
    uint8_t raw = (HAL_GPIO_ReadPin(GIMBAL_KEY_PORT, GIMBAL_KEY_PIN) == GPIO_PIN_RESET) ? 0u : 1u;

    if (raw != s_key_stable) {
        if (++s_key_count >= GIMBAL_KEY_DEBOUNCE) {
            s_key_stable = raw;
            s_key_count  = 0;

            if (s_key_stable == 0) {          /* 按下沿 */
                s_src = (s_src == GIMBAL_SRC_POT) ? GIMBAL_SRC_MPU : GIMBAL_SRC_POT;
                led_apply();

                /* 进入 MPU 模式时重置偏航基准: 陀螺仪积分是相对角,
                 * 不重置会把切换前累积的漂移一起带进来 */
                if (s_src == GIMBAL_SRC_MPU) {
                    s_yaw = 0.0f;
                }
            }
        }
    } else {
        s_key_count = 0;                      /* 抖动回弹, 计数清零 */
    }
}

/* ----------------------------------------------------------------
 * 电位器输入
 * ---------------------------------------------------------------- */
static void pot_update(void)
{
    s_yaw   = clamp_angle(GIMBAL_POT_YAW_SIGN   * pot_to_deg(ADC3_ReadChannel(ADC_CHANNEL_0)));
    s_pitch = clamp_angle(GIMBAL_POT_PITCH_SIGN * pot_to_deg(ADC3_ReadChannel(ADC_CHANNEL_1)));
}

/* ----------------------------------------------------------------
 * MPU6050 输入
 * ---------------------------------------------------------------- */
static void mpu_update(void)
{
    MPU6050_DataUpdate();

    if (!MPU6050_IsOK()) {                    /* 传感器不响应: 保持 0,0 不阻塞 */
        s_yaw   = 0.0f;
        s_pitch = 0.0f;
        return;
    }

    /* --- 俯仰: 加速度计测重力方向 ---
     * 模块 X=机头, Y=右, Z=上。机头下俯 θ 时 ax=-sinθ, az=cosθ,
     * 所以 atan2(-ax, ...) 给出的正是"下俯为正"。
     * 用 sqrt(ay²+az²) 而非 az, 是为了大横滚时仍然成立。 */
    float ax = MPU6050_GetAx();
    float ay = MPU6050_GetAy();
    float az = MPU6050_GetAz();
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    s_pitch = clamp_angle(GIMBAL_MPU_PITCH_SIGN * pitch);

    /* --- 偏航: 陀螺仪 Z 轴积分 ---
     * 机头右转 = 偏航角变正(约定), 本板上对应 g_z 为负, 极性由
     * GIMBAL_GYRO_SIGN 兜住。
     * 陀螺仪积分是相对角, 会缓慢漂移 —— 这是本方案的固有代价
     * (加速度计测不出绕重力轴的转角), 靠死区 + 限幅兜住。 */
    float gz = MPU6050_GetGz() * GIMBAL_GYRO_SIGN;
    if (fabsf(gz) < GIMBAL_GYRO_DEADBAND) {
        gz = 0.0f;
    }
    s_yaw = clamp_angle(s_yaw + gz * ((float)GIMBAL_TICK_MS / 1000.0f));
}

/* ----------------------------------------------------------------
 * 对外接口
 * ---------------------------------------------------------------- */

void Gimbal_Init(void)
{
    s_src       = GIMBAL_SRC_POT;
    s_yaw       = 0.0f;
    s_pitch     = 0.0f;
    s_key_stable = 1;
    s_key_count  = 0;

    led_apply();                              /* 上电: 电位器模式, 灯灭 */

#if GIMBAL_HC06_ENABLE
    HC06_Init();                              /* 内含 1s 延时 */
#endif

    MPU6050_Init();                           /* 内含约 1s 静止零偏校准, 会打印信息 */
}

void Gimbal_Update(void)
{
    key_scan();

    if (s_src == GIMBAL_SRC_POT) {
        pot_update();
    } else {
        mpu_update();
    }
}

void Gimbal_Report(void)
{
    printf("%.1f,%.1f\r\n", (double)s_yaw, (double)s_pitch);
}

Gimbal_Source Gimbal_GetSource(void)
{
    return s_src;
}

void Gimbal_GetTarget(float *yaw, float *pitch)
{
    if (yaw   != NULL) *yaw   = s_yaw;
    if (pitch != NULL) *pitch = s_pitch;
}
