/**
 ******************************************************************************
 * @file    nav.c
 * @brief   4鈫? 宸﹁浆: 5鍑哄彂鐐瑰乏杞姬绾库啋2, 鍚庨€€寰嚎鑷?璺彛
 ******************************************************************************
 */
#include "nav.h"
#include "motor.h"
#include "tim.h"
#include "mpu6050.h"
#include "ir_sensor.h"
#include "hc06.h"
#include "oled.h"
#include <stdio.h>
#include <math.h>

extern IR_SensorTypeDef m_ir;

/* ================================================================
 * 鍐呴儴鐘舵€佹満
 * ================================================================ */
enum {
    PHASE_WAIT_JUNC = 0,  /* 绛夊緟4璺叏榛?鈫?鍚姩杞集 */
    PHASE_TURN      = 1,  /* 宸﹁浆寮х嚎 5鈫? */
    PHASE_PAUSE     = 2,  /* 杞集瀹屾垚鏆傚仠, 璇诲彇IR */
    PHASE_REVERSE   = 3,  /* 浣庨€熷悗閫€寰嚎 鈫?9璺彛 */
    PHASE_SWAY      = 4,  /* 鑴辩嚎鎽囨憜鎭㈠ */
    PHASE_DONE      = 5,  /* 鍒拌揪9璺彛, 瀹屾垚 */
};

/* 鍚庨€€鍙傛暟 */
#define REV_BASE      1500
#define REV_K         350.0f
#define REV_MIN       400
#define REV_MAX       2800
#define SWAY_DUTY     2000
#define SWAY_LIMIT    8.0f
#define REV_BLIND_CM  5.0f
#define PAUSE_MS      400

/* 缂栫爜鍣ㄢ啋璺濈: 4脳PPR=1792, 杞懆闀?蟺脳65mm鈮?0.42cm */
#define ENC_CM (20.42035f / 1792.0f)

static float enc_dist(int32_t dL, int32_t dR) {
    float a = (float)(dL < 0 ? -dL : dL);
    float b = (float)(dR < 0 ? -dR : dR);
    return (a + b) * 0.5f * ENC_CM;
}

/* ================================================================
 * Nav_Turn4to9 鈥?姣忓惊鐜皟鐢ㄤ竴娆? 杩斿洖0=鎵ц涓?1=瀹屾垚
 * ================================================================ */
uint8_t Nav_Turn4to9(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = PHASE_WAIT_JUNC;

    /* TURN 妫€娴嬫爣蹇?*/
    static uint8_t  ir1_active, ir1_done, ir_armed, d1_hit, d2_hit;

    /* PAUSE / REVERSE / SWAY 鍙橀噺 */
    static int32_t  rev_encL0, rev_encR0;
    static uint8_t  rev_ever_on_line;
    static uint32_t pause_start;
    static float    sway_yaw;
    static int8_t   sway_dir;

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    /* ---- yaw 绉垎 ---- */
    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) {
        float gz = MPU6050_GetGz();
        yaw      -= gz * dt;
        sway_yaw -= gz * dt;
    }

    /* ================================================================
     * P0 WAIT_JUNC
     * ================================================================ */
    if (phase == PHASE_WAIT_JUNC) {
        /* 直接启动 (车已在4路口, 不等4黑) */
        yaw = 0; sway_yaw = 0;
        ir1_active = 0; ir1_done = 0; ir_armed = 0;
        d1_hit = 0; d2_hit = 0;
        Motor_Left_Forward(800);
        Motor_Right_Forward(4000);
        phase = PHASE_TURN;
    }
    /* ================================================================
     * P1 TURN: 宸﹁浆寮х嚎 5鈫?, yaw 0鈫?85掳
     * ================================================================ */
    else if (phase == PHASE_TURN) {
        if (fabsf(yaw) >= 85.0f) {
            Move_Stop(); pause_start = now; phase = PHASE_PAUSE;
        } else {
            Motor_Left_Forward(800);
            Motor_Right_Forward(4000);
        }
    }
    /* ================================================================
     * P2 PAUSE
     * ================================================================ */
    else if (phase == PHASE_PAUSE) {
        if (now - pause_start >= PAUSE_MS) {
            rev_encL0 = Encoder_GetLeftCount();
            rev_encR0 = Encoder_GetRightCount();
            rev_ever_on_line = (m_ir.active_count > 0);
            sway_yaw = 0;
            phase = PHASE_REVERSE;
        }
    }
    /* ================================================================
     * P3 REVERSE
    else if (phase == PHASE_REVERSE) {
        if (now - pause_start >= 3000) { Move_Stop(); phase = PHASE_DONE; }
        else { Motor_Left_Reverse(2200); Motor_Right_Reverse(2200); }
    }

    /* ================================================================
     * OLED 鍒锋柊
     * ================================================================ */
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        const char *phname[] = {"W_JUNC","TURN","PAUSE","REV","SWAY","DONE"};
        snprintf(buf, sizeof(buf), "P%d %s", phase, phname[phase]);
        OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "Y%+.0f Sy%+.0f", yaw, sway_yaw);
        OLED_ShowString(0, 16, buf);
        if (phase <= PHASE_TURN)
            snprintf(buf, sizeof(buf), "1:%d%d A%d D1%dD2%d",
                     ir1_active, ir1_done, ir_armed, d1_hit, d2_hit);
        else
            snprintf(buf, sizeof(buf), "Gz%+5.0f OL%d", MPU6050_GetGz(), rev_ever_on_line);
        OLED_ShowString(0, 32, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d C%d",
                 m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3],
                 m_ir.active_count);
        OLED_ShowString(0, 48, buf);
        OLED_Refresh();
    }

    return (phase == PHASE_DONE) ? 1 : 0;
}

/* ================================================================
 * Nav_Turn4to10 鈥?鍙宠浆寮х嚎 5鈫?, 鍚庨€€寰嚎鑷?0璺彛
 *
 * 涓庡乏杞暅鍍?
 *   鐢垫満:   宸﹀揩鍙虫參 鈫?鍙宠浆寮х嚎 (yaw 姝ｆ柟鍚?
 *   IR1:    D4榛戝紑濮?杩涘叆鍒嗘敮) 鈫?D1榛戠粨鏉?绂诲紑鍒嗘敮)
 *   IR2:    D4鈫掑噺閫? D3鈫掔紦鍙宠浆鑷矰2鈫掓殏鍋? *   鍚庨€€:    鍚屽乏杞? IR寰嚎鑷?璺叏榛?10璺彛
 * ================================================================ */
uint8_t Nav_Turn4to10(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = PHASE_WAIT_JUNC;

    /* TURN 妫€娴嬫爣蹇?鈥?鍙宠浆鐢?D4/D3/D2 */
    static uint8_t  ir1_active, ir1_done, ir_armed, d4_hit, d3_hit;

    /* PAUSE / REVERSE / SWAY 鍙橀噺 */
    static int32_t  rev_encL0, rev_encR0;
    static uint8_t  rev_ever_on_line;
    static uint32_t pause_start;
    static float    sway_yaw;
    static int8_t   sway_dir;

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    /* ---- yaw 绉垎 ---- */
    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) {
        float gz = MPU6050_GetGz();
        yaw      -= gz * dt;
        sway_yaw -= gz * dt;
    }

    /* ================================================================
     * P0 WAIT_JUNC
     * ================================================================ */
    if (phase == PHASE_WAIT_JUNC) {
        if (m_ir.active_count == 4) {
            yaw = 0; sway_yaw = 0;
            ir1_active = 0; ir1_done = 0; ir_armed = 0;
            d4_hit = 0; d3_hit = 0;
            Motor_Left_Forward(2800);   /* 宸﹀揩 */
            Motor_Right_Forward(1000);  /* 鍙虫參 鈫?绱у彸杞姬绾?*/
            phase = PHASE_TURN;
        }
    }
    /* ================================================================
     * P1 TURN: 鍙宠浆寮х嚎, yaw 姝ｆ柟鍚?     *
     * IR1 (鍒嗘敮绾?: D4榛戝紑濮?鈫?D1榛戠粨鏉?     * 姝﹁:        IR1缁撴潫 + |yaw|鈮?5掳
     * IR2 (鐩爣绾?: D4鈫掔户缁浆, D3鈫掔紦鍙宠浆鑷矰2鈫掓殏鍋?     * ================================================================ */
    else if (phase == PHASE_TURN) {
        uint8_t s1 = m_ir.raw[0];
        uint8_t s2 = m_ir.raw[1], s3 = m_ir.raw[2], s4 = m_ir.raw[3];

        /* IR1: D4寮€濮?鈫?D1缁撴潫 (闀滃儚宸﹁浆) */
        if (!ir1_done) {
            if (s4 && !ir1_active) ir1_active = 1;
            if (ir1_active && s1)  ir1_done   = 1;
        }
        if (ir1_done && !ir_armed && fabsf(yaw) >= 65.0f) ir_armed = 1;
        /* IR2: D4鈫掔户缁? D3鈫掔紦鍙宠浆, D2鈫掑仠姝?*/
        if (ir_armed) {
            if (s4)           d4_hit = 1;
            if (d4_hit && s3) d3_hit = 1;
        }

        if (fabsf(yaw) >= 85.0f) {
            Move_Stop(); pause_start = now; phase = PHASE_PAUSE;
        } else if (d3_hit) {
            if (s2) {
                Move_Stop(); pause_start = now; phase = PHASE_PAUSE;
            } else {
                Motor_Left_Forward(700);   /* 宸﹀揩 鈫?缂撳彸杞?*/
                Motor_Right_Forward(0);
            }
        } else {
            Motor_Left_Forward(2800);      /* 姝ｅ父绱у彸杞? 涓嶅噺閫?*/
            Motor_Right_Forward(1000);
        }
    }
    /* ================================================================
     * P2 PAUSE (鍚屽乏杞?
     * ================================================================ */
    else if (phase == PHASE_PAUSE) {
        if (now - pause_start >= PAUSE_MS) {
            rev_encL0 = Encoder_GetLeftCount();
            rev_encR0 = Encoder_GetRightCount();
            rev_ever_on_line = (m_ir.active_count > 0);
            sway_yaw = 0;
            phase = PHASE_REVERSE;
        }
    }
    /* ================================================================
     * P3 REVERSE (鍚屽乏杞?
     * ================================================================ */
    else if (phase == PHASE_REVERSE) {
        if (m_ir.active_count == 4) {
            Move_Stop();
            phase = PHASE_DONE;
        } else {
            if (m_ir.active_count > 0) rev_ever_on_line = 1;

            int32_t dL = (int32_t)Encoder_GetLeftCount()  - rev_encL0;
            int32_t dR = (int32_t)Encoder_GetRightCount() - rev_encR0;
            float dist = enc_dist(dL, dR);
            if (dist > REV_BLIND_CM && !rev_ever_on_line) {
                Move_Stop();
                sway_yaw = 0; sway_dir = 0;
                phase = PHASE_SWAY;
            } else {
                float corr = m_ir.position * REV_K;
                int16_t dl = (int16_t)(REV_BASE - corr);
                int16_t dr = (int16_t)(REV_BASE + corr);
                if (dl < REV_MIN) dl = REV_MIN;
                if (dl > REV_MAX) dl = REV_MAX;
                if (dr < REV_MIN) dr = REV_MIN;
                if (dr > REV_MAX) dr = REV_MAX;
                Motor_Left_Reverse((uint16_t)dl);
                Motor_Right_Reverse((uint16_t)dr);
            }
        }
    }
    /* ================================================================
     * P4 SWAY (鍚屽乏杞?
     * ================================================================ */
    else if (phase == PHASE_SWAY) {
        if (m_ir.active_count > 0) {
            Move_Stop();
            rev_encL0 = Encoder_GetLeftCount();
            rev_encR0 = Encoder_GetRightCount();
            rev_ever_on_line = 1;
            phase = PHASE_REVERSE;
        } else {
            if      (sway_yaw >  SWAY_LIMIT) sway_dir = 1;
            else if (sway_yaw < -SWAY_LIMIT) sway_dir = 0;
            if (sway_dir == 0) { Motor_Right_Forward(SWAY_DUTY); Motor_Left_Reverse(SWAY_DUTY); }
            else              { Motor_Left_Forward(SWAY_DUTY);  Motor_Right_Reverse(SWAY_DUTY); }
        }
    }

    /* ================================================================
     * OLED 鍒锋柊
     * ================================================================ */
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        const char *phname[] = {"W_JUNC","TURN","PAUSE","REV","SWAY","DONE"};
        snprintf(buf, sizeof(buf), "R10 P%d %s", phase, phname[phase]);
        OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "Y%+.0f Sy%+.0f", yaw, sway_yaw);
        OLED_ShowString(0, 16, buf);
        if (phase <= PHASE_TURN)
            snprintf(buf, sizeof(buf), "1:%d%d A%d D4%dD3%d",
                     ir1_active, ir1_done, ir_armed, d4_hit, d3_hit);
        else
            snprintf(buf, sizeof(buf), "Gz%+5.0f OL%d", MPU6050_GetGz(), rev_ever_on_line);
        OLED_ShowString(0, 32, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d C%d",
                 m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3],
                 m_ir.active_count);
        OLED_ShowString(0, 48, buf);
        OLED_Refresh();
    }

    return (phase == PHASE_DONE) ? 1 : 0;
}




/* ================================================================
 * Nav_Turn10to6 — 左转90° 10→6
 * ================================================================ */
uint8_t Nav_Turn10to6(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = 0;  /* 0=等路口 1=转弯 2=后退1 3=修正/右转 4=后退2 5=完成 */
    static uint32_t rev_start;
    static uint8_t  p3_mode;    /* P3模式: 0=0011修正 1=0111右转 */
    static uint8_t  p3_ir;      /* 进入P3时IR模式 */
    static uint32_t p3_start;   /* P3暂停起始 */

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    /* yaw 积分 */
    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) {
        yaw -= MPU6050_GetGz() * dt;
    }

    /* ---- P0 ---- */
    if (phase == 0) {
        if (m_ir.active_count == 4) {
            yaw = 0;
            Motor_Left_Forward(800);
            Motor_Right_Forward(4000);
            phase = 1;
        }
    }
    /* ---- P1: 左转至|yaw|>=85°且IR=1101 ---- */
    else if (phase == 1) {
        if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==1 && m_ir.raw[2]==0 && m_ir.raw[3]==1) {
            Move_Stop();
            rev_start = now;
            Motor_Left_Reverse(2200);
            Motor_Right_Reverse(2200);
            phase = 2;
        } else {
            Motor_Left_Forward(800);
            Motor_Right_Forward(4000);
        }
    }
    /* ---- P2: 后退至IR=0000且持续>1秒 ---- */
    else if (phase == 2) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) {
            /* 判定模式: 0011→修正, 0111→右转 */
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            p3_ir = pat;
            if (pat == 0x3) {  /* 0011: 修正到0111 */
                p3_mode = 0;
            } else {           /* 0111: 右转弧线 */
                p3_mode = 1;
                yaw = 0;
            }
            p3_start = now;
            phase = 3;
        }
    }
    /* ---- P3: 100ms暂停→分支 ---- */
    else if (phase == 3) {
        if (now - p3_start < 100000) {
            /* 停100秒, 不驱动电机 */
        } else if (p3_mode == 0) {
            /* 0011→0111: IR循线后退修正 */
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            if (pat == 0x7) {  /* 0111 */
                Move_Stop();
                phase = 5;
            } else {
                float corr = m_ir.position * 350.0f;
                int16_t dl = (int16_t)(2200 - corr);
                int16_t dr = (int16_t)(2200 + corr);
                if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
                if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
                Motor_Left_Reverse((uint16_t)dl);
                Motor_Right_Reverse((uint16_t)dr);
            }
        } else {
            /* 0111: 右转弧线 → |yaw|≥85°且IR=0100 → 后退 */
            if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==1 && m_ir.raw[2]==0 && m_ir.raw[3]==1) {
                Move_Stop();
                rev_start = now;
                Motor_Left_Reverse(2200);
                Motor_Right_Reverse(2200);
                phase = 4;
            } else {
                Motor_Left_Forward(2800);
                Motor_Right_Forward(1000);
            }
        }
    }
    /* ---- P4: 后退至IR=0000且持续>1秒 ---- */
    else if (phase == 4) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) {
            Move_Stop();
            phase = 5;
        }
    }

    /* OLED */
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "10->6 P%d", phase);
        OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "YAW %+6.1f", yaw);
        OLED_ShowString(0, 16, buf);
        snprintf(buf, sizeof(buf), "Gz%+5.0f T%1d", MPU6050_GetGz(),
                 phase==2 ? (int)((now - rev_start)/1000) : 0);
        OLED_ShowString(0, 32, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d E%d",
                 m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3],
                 p3_ir);
        OLED_ShowString(0, 48, buf);
        OLED_Refresh();
    }

    return (phase == 5) ? 1 : 0;
}

/* ================================================================
 * Nav_Turn10to8 — 右转90° 10→8 (上下对称10→6)
 * ================================================================ */
uint8_t Nav_Turn10to8(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = 0;  /* 0=等路口 1=转弯 2=后退1 3=修正/左转 4=后退2 5=完成 */
    static uint32_t rev_start;
    static uint8_t  p3_mode;    /* P3模式: 0=1100修正 1=1110左转 */
    static uint8_t  p3_ir;
    static uint32_t p3_start;

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) {
        yaw -= MPU6050_GetGz() * dt;
    }

    /* ---- P0 ---- */
    if (phase == 0) {
        if (m_ir.active_count == 4) {
            yaw = 0;
            Motor_Left_Forward(2800);
            Motor_Right_Forward(1000);
            phase = 1;
        }
    }
    /* ---- P1: 右转至|yaw|>=85且IR=1011 ---- */
    else if (phase == 1) {
        if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==0 && m_ir.raw[2]==1 && m_ir.raw[3]==1) {
            Move_Stop();
            rev_start = now;
            Motor_Left_Reverse(2200);
            Motor_Right_Reverse(2200);
            phase = 2;
        } else {
            Motor_Left_Forward(2800);
            Motor_Right_Forward(1000);
        }
    }
    /* ---- P2: 后退至IR=0000且持续>1秒 ---- */
    else if (phase == 2) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) {
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            p3_ir = pat;
            if (pat == 0xC) {  /* 1100: 修正到1110 */
                p3_mode = 0;
            } else {           /* 1110: 左转弧线 */
                p3_mode = 1;
                yaw = 0;
            }
            p3_start = now;
            phase = 3;
        }
    }
    /* ---- P3: 100秒暂停→分支 ---- */
    else if (phase == 3) {
        if (now - p3_start < 100000) {
        } else if (p3_mode == 0) {
            /* 1100→1110: IR循线后退修正 */
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            if (pat == 0xE) {  /* 1110 */
                Move_Stop();
                phase = 5;
            } else {
                float corr = m_ir.position * 350.0f;
                int16_t dl = (int16_t)(2200 - corr);
                int16_t dr = (int16_t)(2200 + corr);
                if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
                if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
                Motor_Left_Reverse((uint16_t)dl);
                Motor_Right_Reverse((uint16_t)dr);
            }
        } else {
            /* 1110: 左转弧线 → |yaw|>=85且IR=1011 → 后退 */
            if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==0 && m_ir.raw[2]==1 && m_ir.raw[3]==1) {
                Move_Stop();
                rev_start = now;
                Motor_Left_Reverse(2200);
                Motor_Right_Reverse(2200);
                phase = 4;
            } else {
                Motor_Left_Forward(800);
                Motor_Right_Forward(4000);
            }
        }
    }
    /* ---- P4: 后退至IR=0000且持续>1秒 ---- */
    else if (phase == 4) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) {
            Move_Stop();
            phase = 5;
        }
    }

    /* OLED */
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "10->8 P%d", phase);
        OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "YAW %+6.1f", yaw);
        OLED_ShowString(0, 16, buf);
        snprintf(buf, sizeof(buf), "Gz%+5.0f T%1d", MPU6050_GetGz(),
                 phase==2 ? (int)((now - rev_start)/1000) : 0);
        OLED_ShowString(0, 32, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d E%d",
                 m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3],
                 p3_ir);
        OLED_ShowString(0, 48, buf);
        OLED_Refresh();
    }

    return (phase == 5) ? 1 : 0;
}





/* ================================================================
 * Nav_9to1 — 9→1 (同10→8右转弧线)
 * ================================================================ */
uint8_t Nav_9to1(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = 0;
    static uint32_t rev_start;
    static uint8_t  p3_mode, p3_ir;
    static uint32_t p3_start;

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);
    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) yaw -= MPU6050_GetGz() * dt;

    if (phase == 0) {
        if (m_ir.active_count == 4) {
            yaw = 0;
            Motor_Left_Forward(2800);
            Motor_Right_Forward(1000);
            phase = 1;
        }
    } else if (phase == 1) {
        if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==0 && m_ir.raw[2]==1 && m_ir.raw[3]==1) {
            Move_Stop(); rev_start = now;
            Motor_Left_Reverse(2200); Motor_Right_Reverse(2200);
            phase = 2;
        } else { Motor_Left_Forward(4000); Motor_Right_Forward(800); }
    } else if (phase == 2) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) {
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            p3_ir = pat;
            p3_mode = (pat == 0xC) ? 0 : 1;
            if (p3_mode == 1) yaw = 0;
            p3_start = now; phase = 3;
        }
    } else if (phase == 3) {
        if (now - p3_start < 100000) {
        } else if (p3_mode == 0) {
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            if (pat == 0xE) { Move_Stop(); phase = 5; }
            else {
                float corr = m_ir.position * 350.0f;
                int16_t dl = (int16_t)(2200 - corr), dr = (int16_t)(2200 + corr);
                if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
                if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
                Motor_Left_Reverse((uint16_t)dl); Motor_Right_Reverse((uint16_t)dr);
            }
        } else {
            if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==0 && m_ir.raw[2]==1 && m_ir.raw[3]==1) {
                Move_Stop(); rev_start = now;
                Motor_Left_Reverse(2200); Motor_Right_Reverse(2200);
                phase = 4;
            } else { Motor_Left_Forward(800); Motor_Right_Forward(4000); }
        }
    } else if (phase == 4) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) { Move_Stop(); phase = 5; }
    }

    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "9->1 P%d", phase); OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "YAW %+6.1f", yaw); OLED_ShowString(0, 16, buf);
        snprintf(buf, sizeof(buf), "Gz%+5.0f T%1d", MPU6050_GetGz(), phase==2?(int)((now-rev_start)/1000):0);
        OLED_ShowString(0, 32, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d E%d", m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3], p3_ir);
        OLED_ShowString(0, 48, buf);
        OLED_Refresh();
    }
    return (phase == 5) ? 1 : 0;
}

/* ================================================================
 * Nav_9to3 — 9→3 (同10→6左转弧线)
 * ================================================================ */
uint8_t Nav_9to3(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = 0;
    static uint32_t rev_start;
    static uint8_t  p3_mode, p3_ir;
    static uint32_t p3_start;

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);
    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) yaw -= MPU6050_GetGz() * dt;

    if (phase == 0) {
        if (m_ir.active_count == 4) {
            yaw = 0;
            Motor_Left_Forward(800);
            Motor_Right_Forward(4000);
            phase = 1;
        }
    } else if (phase == 1) {
        if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==1 && m_ir.raw[2]==0 && m_ir.raw[3]==1) {
            Move_Stop(); rev_start = now;
            Motor_Left_Reverse(2200); Motor_Right_Reverse(2200);
            phase = 2;
        } else { Motor_Left_Forward(800); Motor_Right_Forward(4000); }
    } else if (phase == 2) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) {
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            p3_ir = pat;
            p3_mode = (pat == 0x3) ? 0 : 1;
            if (p3_mode == 1) yaw = 0;
            p3_start = now; phase = 3;
        }
    } else if (phase == 3) {
        if (now - p3_start < 100000) {
        } else if (p3_mode == 0) {
            uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
            if (pat == 0x7) { Move_Stop(); phase = 5; }
            else {
                float corr = m_ir.position * 350.0f;
                int16_t dl = (int16_t)(2200 - corr), dr = (int16_t)(2200 + corr);
                if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
                if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
                Motor_Left_Reverse((uint16_t)dl); Motor_Right_Reverse((uint16_t)dr);
            }
        } else {
            if (fabsf(yaw) >= 85.0f && m_ir.raw[0]==1 && m_ir.raw[1]==1 && m_ir.raw[2]==0 && m_ir.raw[3]==1) {
                Move_Stop(); rev_start = now;
                Motor_Left_Reverse(2200); Motor_Right_Reverse(2200);
                phase = 4;
            } else { Motor_Left_Forward(4000); Motor_Right_Forward(800); }
        }
    } else if (phase == 4) {
        if (m_ir.active_count == 0 && (now - rev_start) >= 1000) { Move_Stop(); phase = 5; }
    }

    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "9->3 P%d", phase); OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "YAW %+6.1f", yaw); OLED_ShowString(0, 16, buf);
        snprintf(buf, sizeof(buf), "Gz%+5.0f T%1d", MPU6050_GetGz(), phase==2?(int)((now-rev_start)/1000):0);
        OLED_ShowString(0, 32, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d E%d", m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3], p3_ir);
        OLED_ShowString(0, 48, buf);
        OLED_Refresh();
    }
    return (phase == 5) ? 1 : 0;
}

/* ================================================================
 * Nav_9to2 — 直行 9→2, IR=0110循线, IR=0000停止
 * ================================================================ */
uint8_t Nav_9to2(void)
{
    static uint8_t  phase = 0;
    static uint32_t last_disp;
    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    if (phase == 0) {
        if (m_ir.active_count == 4) {
            Motor_Left_Forward(2000);
            Motor_Right_Forward(2000);
            phase = 1;
        }
    } else if (phase == 1) {
        if (m_ir.active_count == 0) {
            Move_Stop(); phase = 2;
        } else {
            float corr = m_ir.position * 350.0f;
            int16_t dl = (int16_t)(2000 - corr), dr = (int16_t)(2000 + corr);
            if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
            if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
            Motor_Left_Forward((uint16_t)dl); Motor_Right_Forward((uint16_t)dr);
        }
    }
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "9->2 P%d", phase); OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d C%d", m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3], m_ir.active_count);
        OLED_ShowString(0, 32, buf);
        OLED_Refresh();
    }
    return (phase == 2) ? 1 : 0;
}

/* ================================================================
 * Nav_10to7 — 直行 10→7, IR=0110循线, IR=0000停止
 * ================================================================ */
uint8_t Nav_10to7(void)
{
    static uint8_t  phase = 0;
    static uint32_t last_disp;
    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    if (phase == 0) {
        if (m_ir.active_count == 4) {
            Motor_Left_Forward(2000);
            Motor_Right_Forward(2000);
            phase = 1;
        }
    } else if (phase == 1) {
        if (m_ir.active_count == 0) {
            Move_Stop(); phase = 2;
        } else {
            float corr = m_ir.position * 350.0f;
            int16_t dl = (int16_t)(2000 - corr), dr = (int16_t)(2000 + corr);
            if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
            if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
            Motor_Left_Forward((uint16_t)dl); Motor_Right_Forward((uint16_t)dr);
        }
    }
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "10->7 P%d", phase); OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d C%d", m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3], m_ir.active_count);
        OLED_ShowString(0, 32, buf);
        OLED_Refresh();
    }
    return (phase == 2) ? 1 : 0;
}

/* ================================================================
 * Nav_6to10 — 6→10: 后退巡线→0111暂停→右转90°→10→右转90°
 * ================================================================ */
uint8_t Nav_6to10(void)
{
    static float    yaw = 0;
    static uint32_t last_disp = 0, last_gyro = 0;
    static uint8_t  phase = 0;  /* 0=等起点 1=后退巡线 2=暂停1 3=右转1 4=右转2 5=完成 */
    static uint32_t pause_start;

    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    if (last_gyro == 0) last_gyro = now;
    float dt = (now - last_gyro) * 0.001f; last_gyro = now;
    if (dt > 0 && dt < 0.5f) yaw -= MPU6050_GetGz() * dt;

    /* ---- P0: 等IR=0000或0110(到达6终点) ---- */
    if (phase == 0) {
        if (m_ir.active_count == 0 || (m_ir.raw[0]==0 && m_ir.raw[1]==1 && m_ir.raw[2]==1 && m_ir.raw[3]==0)) {
            Motor_Left_Reverse(2000);
            Motor_Right_Reverse(2000);
            phase = 1;
        }
    }
    /* ---- P1: 后退巡线保持IR=0110, 至IR=0111暂停 ---- */
    else if (phase == 1) {
        uint8_t pat = (m_ir.raw[0]<<3)|(m_ir.raw[1]<<2)|(m_ir.raw[2]<<1)|m_ir.raw[3];
        if (pat == 0x7) {  /* 0111: 转角 */
            Move_Stop();
            pause_start = now;
            phase = 2;
        } else {
            float corr = m_ir.position * 350.0f;
            int16_t dl = (int16_t)(2000 - corr), dr = (int16_t)(2000 + corr);
            if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
            if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
            Motor_Left_Reverse((uint16_t)dl);
            Motor_Right_Reverse((uint16_t)dr);
        }
    }
    /* ---- P2: 暂停400ms ---- */
    else if (phase == 2) {
        if (now - pause_start >= 400) {
            yaw = 0;
            Motor_Left_Forward(2800);
            Motor_Right_Forward(1000);
            phase = 3;
        }
    }
    /* ---- P3: 右转90°到达10路口 ---- */
    else if (phase == 3) {
        if (fabsf(yaw) >= 85.0f) {
            Move_Stop();
            yaw = 0;
            Motor_Left_Forward(2800);
            Motor_Right_Forward(1000);
            phase = 4;
        }
    }
    /* ---- P4: 右转90°车头朝向10→9 ---- */
    else if (phase == 4) {
        if (fabsf(yaw) >= 85.0f) {
            Move_Stop();
            phase = 5;
        }
    }

    /* OLED */
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "6->10 P%d", phase);
        OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "YAW %+6.1f", yaw);
        OLED_ShowString(0, 16, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d C%d",
                 m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3],
                 m_ir.active_count);
        OLED_ShowString(0, 32, buf);
        OLED_Refresh();
    }

    return (phase == 5) ? 1 : 0;
}

/* ================================================================
 * Nav_10to9 — 直行 10→9, IR=0110循线, 4路全黑停止
 * ================================================================ */
uint8_t Nav_10to9(void)
{
    static uint8_t  phase = 0;
    static uint32_t last_disp;
    uint32_t now = HAL_GetTick();
    IR_Update(&m_ir);

    if (phase == 0) {
        if (m_ir.active_count == 4) {
            Motor_Left_Forward(2000);
            Motor_Right_Forward(2000);
            phase = 1;
        }
    } else if (phase == 1) {
        if (m_ir.active_count == 4) {
            Move_Stop(); phase = 2;
        } else {
            float corr = m_ir.position * 350.0f;
            int16_t dl = (int16_t)(2000 - corr), dr = (int16_t)(2000 + corr);
            if (dl < 400) dl = 400;
                if (dl > 2800) dl = 2800;
            if (dr < 400) dr = 400;
                if (dr > 2800) dr = 2800;
            Motor_Left_Forward((uint16_t)dl); Motor_Right_Forward((uint16_t)dr);
        }
    }
    if (now - last_disp >= 200) {
        last_disp = now;
        for (uint8_t pg = 0; pg < 8; pg++) { for (uint8_t col = 0; col < 128; col++) oled_buf[col][pg] = 0; dirty_page[pg] = 1; }
        char buf[32];
        snprintf(buf, sizeof(buf), "10->9 P%d", phase); OLED_ShowString(0, 0, buf);
        snprintf(buf, sizeof(buf), "IR%d%d%d%d C%d", m_ir.raw[0], m_ir.raw[1], m_ir.raw[2], m_ir.raw[3], m_ir.active_count);
        OLED_ShowString(0, 32, buf);
        OLED_Refresh();
    }
    return (phase == 2) ? 1 : 0;
}

