/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   右电机 位置/速度 串级闭环 (位置外环 → 速度环, 斜率受限参考 + 在线估计)
 *
 * ============================ 设计依据 ============================
 * 本套参数不是"试出来的", 而是先在 tools/sim_control.py 里建立被控对象模型
 * (近空载 JGB37-520: 死区 + 高增益 + 一阶惯性 + 10ms采样/EMA测速滞后),
 * 用实测症状(启动峰值30000 / 巡航峰谷20000 / 反向尖峰-20000)反推出电机参数
 * (G≈45~100 counts/s per duty, τ≈10~15ms), 再用 tools/tune_control.py 在
 * 参数不确定域上寻优得到。34 个工况(含 ±30% 方向不对称、G=20~120)全部达标。
 *
 * ============================ 三条症状的根因 ============================
 * 1) 启动尖峰 30000: 原 vel_ref = Kp_pos×error 在 t=0 直接阶跃到限幅 6000,
 *    电机被瞬间拉满。→ 加"速度参考斜率限制(MC_AMAX)", vel_ref 用 0.75s 爬升。
 *
 * 2) 巡航剧烈振荡: 原方案用"反电动势前馈 ff = k×vel_ref"抵消稳态 duty。
 *    但该前馈的斜率必须精确等于 1/G, 而 G 是未知的; 猜错 → 前馈本身就把
 *    工作点推到错误的转速, 闭环再来回纠正 → 极限环。
 *    → ★根治: 反电动势前馈置零, 改由"积分器"承担。积分器在数学上就等于
 *      一个在线的 1/G 估计器, 自动收敛到正确的稳态 duty, 与 G 无关。
 *
 * 3) 到位反向尖峰 -20000: 原静摩擦前馈用 math.copysign(500, vel_ref), 在
 *    vel_ref 过零时 duty 发生 ±1000 的阶跃(瞬间满反向)。→ 改成平滑饱和
 *    MC_FF_STIC×vel_ref/(|vel_ref|+MC_FF_SMOOTH), 过零连续, 无阶跃。
 *
 * 另外: 速度参考叠加了减速距离约束 v ≤ sqrt(2·a·|error|), 保证到目标前
 * 平滑减速, 不冲过头。速度环 P 取极小(0.005), 靠积分保证无静差, 环路增益低
 * → 与测速滞后配合时相位裕度充足, 不会自激。
 *
 * 控制周期: 10ms (由 1ms SysTick 分频)
 * 测速: 10ms 差分 + EMA (motor.c, VEL_EMA_A=0.60, τ≈6.7ms)
 ******************************************************************************
 */
#include "motor_control.h"
#include "motor.h"
#include <math.h>   /* fabsf, sqrtf */

/* ---- 可调参数 ---- */
#define MC_VMAX         6000.0f  /* 巡航速度上限 (counts/s): 一圈 32256/6000≈5.4s */
#define MC_KP_POS       2.0f     /* 位置外环 P: vel_ref = Kp_pos × error */
#define MC_AMAX         8000.0f  /* 速度参考斜率限制 (counts/s²): 消除启动阶跃 */
#define MC_DECEL        9000.0f  /* 减速规划: |vel_ref| ≤ sqrt(2·MC_DECEL·|error|) */
#define MC_KP_VEL       0.005f   /* 速度环 P: 提供阻尼 */
#define MC_KI_VEL       0.40f    /* 速度环 I (1/s): 主积分作用 */
#define MC_FF_STIC      500.0f   /* 静摩擦前馈 (duty) */
#define MC_FF_SMOOTH    150.0f   /* 静摩擦前馈平滑尺度 (counts/s): 消除过零阶跃 */
#define MC_INT_MAX      800.0f   /* 积分限幅 (抗饱和) */
#define MC_MAX_DUTY     4000     /* duty 限幅: 巡航只要 ~400, 这个值现在只为启动突破服务 */
#define MC_ARRIVE_BAND  80       /* 到位死区: |位置误差| < 80 counts 就停车 */
#define MC_EXIT_BAND    250      /* 到位滞回: 停车后误差超过此值才重新控制 */

/* ---- 启动突破 (breakaway kick) ----
 * 现象: 上电后轮子不转, 要用手推一下才走。
 *
 * 机理: 巡航 duty≈400 就能维持 6000 counts/s, 但从静止起步要克服静摩擦
 *       (以及轮子压地的阻力), 需要大得多的 duty。而起步阶段恰好三处都不给力:
 *         ff       = 500×vel_ref/(vel_ref+150) → vel_ref=0 时就是 0, 低速时最小
 *         integral ≤ MC_INT_MAX = 800
 *         duty     ≤ MC_MAX_DUTY              → 原来只有 1200, 这才是真瓶颈
 *       实算: t≈0.7s 后 duty 就顶死在上限, 若那时还转不动就永远不动。
 *
 * 做法: 检测到"参考速度已在要求转、实测速度仍≈0"持续 MC_KICK_MS, 判定被静摩擦
 *       卡住 → 叠加一个突破 duty 强行起转, 一旦真转起来立刻撤销。
 *       突破 duty 从 MIN 起按 RAMP 爬升, 所以不必事先知道静摩擦多大 —— 它会自己
 *       爬到刚好能起转的位置。
 *
 * ★ 卡住期间必须把积分清零: 否则积分会趁机饱和到 INT_MAX, 起转后要靠反向误差
 *   慢慢退饱和, 造成大幅过冲 —— 这是"直接调大 MC_MAX_DUTY 让积分硬顶"的致命伤。
 *   突破项独立于积分, 撤销后积分仍是干净的。 */
#define MC_KICK_CMD_VEL   600.0f    /* |vel_ref| 超过此值才算"确实要求转" */
#define MC_KICK_DONE_VEL  300.0f    /* |vel| 超过此值算"已经转起来了" */
#define MC_KICK_MS        150       /* 要求转却不动超过这么久, 判定卡住 */
#define MC_KICK_DUTY_MIN  1200.0f   /* 突破 duty 起点 */
#define MC_KICK_DUTY_MAX  3500.0f   /* 突破 duty 上限 */
#define MC_KICK_RAMP      3000.0f   /* 突破 duty 爬升速率 (duty/s) */

static int32_t  s_target_pos;
static int32_t  s_duty;
static uint32_t s_div;
static uint8_t  s_arrived;
static float    s_vel_ref;       /* 斜率受限后的速度参考 (counts/s) */
static float    s_vel_integral;  /* 速度环积分 (duty) */
static uint8_t  s_enabled = 1;   /* 0=挂起闭环, 供开环辨识使用 */

/* 启动突破状态 */
static uint8_t  s_kicking;       /* 1=正在执行突破 */
static uint16_t s_kick_ms;       /* 卡住观察计时 (ms) */
static float    s_kick_duty;     /* 当前突破 duty 幅值, 方向由 vel_ref 决定 */

static void Breakaway_Reset(void)
{
    s_kicking   = 0;
    s_kick_ms   = 0;
    s_kick_duty = MC_KICK_DUTY_MIN;
}

/* 每控制周期调用一次, 返回本周期要叠加的突破 duty (带符号); 0 = 不突破 */
static float Breakaway_Update(float vel_ref, float vel)
{
    uint8_t want  = (fabsf(vel_ref) > MC_KICK_CMD_VEL);  /* 参考在要求转动 */
    uint8_t moved = (fabsf(vel)    > MC_KICK_DONE_VEL);  /* 实测已经转了   */

    if (!want) {
        /* 减速段/到位附近 vel_ref 很小, 本机制完全退出 */
        Breakaway_Reset();
    } else if (!s_kicking) {
        if (moved) {
            s_kick_ms = 0;                       /* 正常起转, 不介入 */
        } else if (s_kick_ms < MC_KICK_MS) {
            s_kick_ms += MC_PERIOD_MS;           /* 要求转却没动, 先观察 */
        } else {
            s_kicking   = 1;                     /* 确认卡住 → 开始突破 */
            s_kick_ms   = 0;
            s_kick_duty = MC_KICK_DUTY_MIN;
        }
    } else if (moved) {
        s_kicking = 0;                           /* 转起来了 → 立刻撤销 */
        s_kick_ms = 0;
    } else if (s_kick_duty < MC_KICK_DUTY_MAX) {
        s_kick_duty += MC_KICK_RAMP * (MC_PERIOD_MS / 1000.0f);   /* 还没动 → 加力 */
        if (s_kick_duty > MC_KICK_DUTY_MAX) s_kick_duty = MC_KICK_DUTY_MAX;
    }

    return s_kicking ? (s_kick_duty * ((vel_ref >= 0.0f) ? 1.0f : -1.0f)) : 0.0f;
}

void MotorControl_SetEnabled(uint8_t en)
{
    s_enabled = en;
    if (!en) {
        s_duty = 0;
        s_vel_ref = 0.0f;
        s_vel_integral = 0.0f;
        s_arrived = 0;
        Breakaway_Reset();
    }
}

void MotorControl_Init(void)
{
    s_target_pos   = Encoder_GetRightPosition();
    s_duty         = 0;
    s_div          = 0;
    s_arrived      = 0;
    s_vel_ref      = 0.0f;
    s_vel_integral = 0.0f;
    Breakaway_Reset();
}

void MotorControl_SetTargetPosition(int32_t pos)
{
    s_target_pos   = pos;
    s_arrived      = 0;
    s_vel_ref      = 0.0f;
    s_vel_integral = 0.0f;
    Breakaway_Reset();
}

int32_t MotorControl_GetTargetPosition(void) { return s_target_pos; }
int32_t MotorControl_GetDuty(void)           { return s_duty; }
float   MotorControl_GetVelRef(void)         { return s_vel_ref; }
float   MotorControl_GetIntegral(void)       { return s_vel_integral; }
/* 直接读编码器, 不使用缓存 —— 缓存曾在"闭环挂起"时永不刷新, 导致遥测恒为 0 */
int32_t MotorControl_GetVelocity(void)       { return (int32_t)Encoder_GetRightVelocity(); }

void MotorControl_Update(void)
{
    Encoder_Update();   /* 先刷新位置/速度反馈 (10ms 节拍) */

    if (!s_enabled) return;   /* 开环辨识期间挂起闭环 */

    int32_t pos   = Encoder_GetRightPosition();
    float   vel   = Encoder_GetRightVelocity();
    int32_t error = s_target_pos - pos;

    /* 到位滞回: 已到位则短路制动锁住, 只有误差超出退出阈值才重新控制 */
    if (s_arrived) {
        if (error > -MC_EXIT_BAND && error < MC_EXIT_BAND) {
            s_duty = 0;
            Motor_Right_Brake();
            return;
        }
        s_arrived = 0;
    }

    if (error > -MC_ARRIVE_BAND && error < MC_ARRIVE_BAND) {
        /* 到位: 短路制动, 静摩擦自锁 (近空载滑行会冲过头) */
        s_arrived      = 1;
        s_duty         = 0;
        s_vel_ref      = 0.0f;
        s_vel_integral = 0.0f;
        Motor_Right_Brake();
        return;
    }

    /* ---- 1. 位置外环 → 速度参考, 并受"减速距离"约束 ---- */
    float err     = (float)error;
    float vel_ref = MC_KP_POS * err;

    float v_dec = sqrtf(2.0f * MC_DECEL * fabsf(err));   /* 剩余距离内能减到 0 的速度 */
    if (fabsf(vel_ref) > v_dec) vel_ref = (err > 0.0f) ? v_dec : -v_dec;

    if (vel_ref >  MC_VMAX) vel_ref =  MC_VMAX;
    if (vel_ref < -MC_VMAX) vel_ref = -MC_VMAX;

    /* ---- 2. 斜率限制: 消除速度参考的阶跃(启动过猛) ---- */
    float dv = MC_AMAX * (MC_PERIOD_MS / 1000.0f);
    if (vel_ref > s_vel_ref + dv) vel_ref = s_vel_ref + dv;
    if (vel_ref < s_vel_ref - dv) vel_ref = s_vel_ref - dv;
    s_vel_ref = vel_ref;

    /* ---- 3. 前馈: 只补静摩擦, 平滑饱和(过零无阶跃)
     * 反电动势项故意不做前馈 —— 它要求精确已知 1/G, 猜错反而制造超调;
     * 由速度环积分器在线估计正确的稳态 duty, 与 G 无关。 ---- */
    float ff = MC_FF_STIC * (s_vel_ref / (fabsf(s_vel_ref) + MC_FF_SMOOTH));

    /* ---- 4. 速度环 PI ---- */
    float vel_err = s_vel_ref - vel;
    s_vel_integral += MC_KI_VEL * vel_err * (MC_PERIOD_MS / 1000.0f);
    if (s_vel_integral >  MC_INT_MAX) s_vel_integral =  MC_INT_MAX;
    if (s_vel_integral < -MC_INT_MAX) s_vel_integral = -MC_INT_MAX;

    /* ---- 5. 启动突破: 卡住时叠加, 并冻结积分防止起转后过冲 ---- */
    float kick = Breakaway_Update(s_vel_ref, vel);
    if (kick != 0.0f) s_vel_integral = 0.0f;

    float duty = ff + MC_KP_VEL * vel_err + s_vel_integral + kick;

    int32_t d = (int32_t)duty;
    if (d >  MC_MAX_DUTY) d =  MC_MAX_DUTY;
    if (d < -MC_MAX_DUTY) d = -MC_MAX_DUTY;

    s_duty = d;

    /* 右电机方向: Forward 使 pos 递增(与左电机相反), 故不取反。
     * 若实测车轮反向(远离目标), 改成 Motor_Right_Drive(-s_duty)。 */
    Motor_Right_Drive(s_duty);
}

/* 1ms SysTick 中断调用，内部分频到控制周期 */
void MotorControl_Tick(void)
{
    if (++s_div >= MC_PERIOD_MS) {
        s_div = 0;
        MotorControl_Update();
    }
}

uint8_t MotorControl_IsArrived(void)
{
    int32_t err = s_target_pos - Encoder_GetRightPosition();
    float   vel = Encoder_GetRightVelocity();
    return (err > -MC_ARRIVE_BAND && err < MC_ARRIVE_BAND &&
            vel > -25.0f && vel < 25.0f);
}
