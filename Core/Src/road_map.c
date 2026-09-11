/**
 ******************************************************************************
 * @file    road_map.c  — 5->4 防冲线专用版
 ******************************************************************************
 */
#include "road_map.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"
#include "error_log.h"
#include "tim.h"
#include <string.h>
#include <math.h>

#define PID_KP      5000.0f
#define PID_KI      500.0f
#define PID_KD      0.0f
#define PID_MAX_DUTY  2800
#define PID_NORM_MAX  2400
#define SPEED_NORMAL  0.15f
#define SPEED_SLOW    0.10f

static PID_HandleTypeDef pid_L, pid_R;

RoadMapPosition g_pos;
ChassisParams g_chassis = { .wheel_dia_mm=65,.track_width_mm=215,.kf_calib=8,.duty_min=3760,.duty_run=4760 };
IR_SensorTypeDef m_ir;
static Kalman2D m_kf;
static uint32_t m_last_pred;
static MotionPhase m_phase = PHASE_IDLE;
static MapNode   m_target = NODE_5;
static float     m_distance, m_goal;
static uint16_t  m_duty;
static uint8_t   m_junc_ok;
static int32_t   m_encR0, m_encL0;
static uint8_t   m_seen_junc;  /* 见路口 */
uint8_t   g_dbg_motor;  /* 0=PID前驱 1=路口恢复 2=已停 3=冲出 */
#define K_ENC_TO_DEG 0.03037f

void RoadMap_Init(void) {
    memset(&g_pos,0,sizeof(g_pos));
    m_phase=PHASE_IDLE; m_target=NODE_5;
    IR_Init(); memset(&m_ir,0,sizeof(m_ir));
    Kalman2D_Init(&m_kf,0.0001f,0.1f,0.5f,99.0f);
    PID_Init(&pid_L,PID_KP,PID_KI,PID_KD,0,PID_MAX_DUTY);
    PID_Init(&pid_R,PID_KP,PID_KI,PID_KD,0,PID_MAX_DUTY);
}
void RoadMap_Update(void) {
    Motor_UpdateRPM();
    uint32_t now=HAL_GetTick();
    if(now-m_last_pred>=10){ float dt=(now-m_last_pred)*0.001f; m_last_pred=now;
        float ax=MPU6050_IsOK()?MPU6050_GetAy()*9.8f:0;
        Kalman2D_Predict(&m_kf,ax,dt);
        float vL=Motor_GetLeftSpeedMS(),vR=Motor_GetRightSpeedMS();
        Kalman2D_UpdateVel(&m_kf,(vL+vR)*0.5f);
    }
}
void RoadMap_GoToNode(MapNode t,uint16_t d) {
    __HAL_TIM_SET_COUNTER(&htim1,0); __HAL_TIM_SET_COUNTER(&htim3,0);
    Kalman2D_Init(&m_kf,0.0001f,0.1f,0.5f,99.0f); IR_ClearJunction(&m_ir);
    m_phase=PHASE_MOVING; m_target=t; m_distance=0; m_goal=0; m_duty=d; m_junc_ok=0; m_seen_junc=0; m_last_pred=HAL_GetTick();
}
void RoadMap_GoStraight(float dist,uint16_t d) {
    __HAL_TIM_SET_COUNTER(&htim1,0); __HAL_TIM_SET_COUNTER(&htim3,0);
    Kalman2D_Init(&m_kf,0.0001f,0.1f,0.5f,99.0f); IR_ClearJunction(&m_ir);
    m_phase=PHASE_MOVING; m_goal=dist; m_distance=0; m_duty=d; m_junc_ok=0; m_seen_junc=0; m_last_pred=HAL_GetTick();
    EL_BeginSegment(0,dist);
}
void RoadMap_Rotate(float a,uint16_t d) { m_encR0=Encoder_GetRightCount(); m_encL0=Encoder_GetLeftCount(); m_phase=PHASE_ROTATING; m_goal=a; m_distance=0; m_duty=d; }
void RoadMap_Stop(void) { Move_Stop(); m_phase=PHASE_IDLE; }
void RoadMap_QRFix(MapNode n,float x,float y,float h) { g_pos.node=n; g_pos.x_cm=x; g_pos.y_cm=y; g_pos.heading_deg=h; g_pos.conf=CONF_QR; Kalman2D_Reset(&m_kf); }

void RoadMap_Run(void) {
    if(m_phase==PHASE_IDLE||m_phase==PHASE_DONE) return;
    uint32_t now=HAL_GetTick();
    RoadMap_Update();
    if(m_phase==PHASE_MOVING) {
        m_distance=m_kf.x/g_chassis.kf_calib*100.0f;
        /* 固定占空比 + 末端减速 + 右轮补偿 */
        uint16_t dL=m_duty, dR=m_duty*1.2f;
        if(dR>PID_MAX_DUTY)dR=PID_MAX_DUTY;
        if(m_goal>0&&m_distance>m_goal*0.6f) { dL=m_duty/2; dR=dL*1.2f; }
        float vL=Motor_GetLeftSpeedMS(), vR=Motor_GetRightSpeedMS();
        float df=(vL>vR)?(vL-vR):(vR-vL);
        if(df>0.10f&&vL>0.02f&&vR>0.02f) { uint16_t av=(dL+dR)/2; if(vL>vR){dL=av*0.9f;dR=av*1.1f;}else{dR=av*0.9f;dL=av*1.1f;} }
        if(dL>dR+800)dL=dR+800; if(dR>dL+800)dR=dL+800;
        /* IR循线 */
        if(m_ir.line_lost==0) { float e=m_ir.error; if(e<-0.5f)dR/=2; else if(e>0.5f)dL/=2; }
        EL_UpdatePID(dL,dR,vL,vR);
        /* 路口恢复期间不驱动前进, 下面路口处理统一控制 */
        if(!(m_seen_junc&&!m_junc_ok)) {
            Motor_Left_Forward(dL); Motor_Right_Forward(dR);
            g_dbg_motor = 0;  /* PID */
        }
        /* 路口 */
        if(m_distance>2.0f&&IR_IsJunction(&m_ir)) m_seen_junc=1;
        if(m_seen_junc&&!m_junc_ok) {
            g_dbg_motor = 1;  /* 路口恢复 */
            if(m_ir.active_count==4) { m_junc_ok=1; g_pos.conf=CONF_JUNC; Move_Stop(); m_phase=PHASE_DONE; IR_ClearJunction(&m_ir); EL_EndSegment(); g_dbg_motor=2; }
            else if(m_ir.active_count==0) { g_dbg_motor=3; /* 冲出,后退 */ uint8_t r0=m_ir.raw[0],r1=m_ir.raw[1],r2=m_ir.raw[2],r3=m_ir.raw[3]; uint16_t rv=m_duty/3;
                if(r0&&!r1&&!r2&&!r3){Motor_Left_Forward(0);Motor_Right_Reverse(rv*2);}
                else if(!r0&&r1&&!r2&&!r3){Motor_Left_Reverse(rv);Motor_Right_Reverse(rv*2);}
                else if(!r0&&!r1&&r2&&!r3){Motor_Left_Reverse(rv*2);Motor_Right_Reverse(rv);}
                else if(!r0&&!r1&&!r2&&r3){Motor_Left_Reverse(rv*2);Motor_Right_Forward(0);}
                else{Motor_Left_Reverse(rv);Motor_Right_Reverse(rv);} }
            else { Move_Stop(); }
        }
        if(m_goal>0&&m_distance>=m_goal&&!m_junc_ok) { Move_Stop(); m_phase=PHASE_DONE; }
    } else if(m_phase==PHASE_ROTATING) {
        int32_t dR=(int32_t)(uint16_t)(Encoder_GetRightCount()-m_encR0), dL=(int32_t)(uint16_t)(Encoder_GetLeftCount()-m_encL0);
        float raw=K_ENC_TO_DEG*(float)(dR-dL); m_distance=raw<0?-raw:raw;
        uint8_t ic=(m_ir.line_lost==0&&m_distance>10.0f);
        if(m_distance<m_goal&&!ic) { Motor_Right_Forward(m_duty); Motor_Left_Reverse(m_duty); }
        else { Move_Stop(); m_phase=PHASE_DONE; }
    }
}
MotionPhase RoadMap_GetPhase(void) { return m_phase; }
uint8_t RoadMap_IsDone(void) { return m_phase==PHASE_DONE||m_phase==PHASE_IDLE; }
float RoadMap_GetProgress(void) { return m_distance; }
float RoadMap_GetSpeedL(void) { return Motor_GetLeftSpeedMS(); }
float RoadMap_GetSpeedR(void) { return Motor_GetRightSpeedMS(); }
MapNode RoadMap_GetCurrentNode(void) { return g_pos.node; }
MapNode RoadMap_GetTargetNode(void) { return m_target; }
PosConfidence RoadMap_GetConfidence(void) { return g_pos.conf; }
uint8_t RoadMap_IsAtJunction(void) { return m_junc_ok; }
