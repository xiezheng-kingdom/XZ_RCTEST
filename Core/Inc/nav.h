/**
 ******************************************************************************
 * @file    nav.h
 * @brief   导航动作: 4→9 左转 / 4→10 右转
 ******************************************************************************
 */
#ifndef __NAV_H__
#define __NAV_H__

#include "main.h"

/* 返回值: 0=执行中, 1=完成 */
uint8_t Nav_Turn4to9(void);   /* 左转: 5→2弧线 + 后退至9路口 */
uint8_t Nav_Turn4to10(void);  /* 右转: 5→7弧线 + 后退至10路口 */
uint8_t Nav_Turn10to6(void);  /* 左转90° 10→6 */
uint8_t Nav_Turn10to8(void);  /* 右转90° 10→8 */
uint8_t Nav_9to1(void);       /* 9→1 (同10→8) */
uint8_t Nav_9to3(void);       /* 9→3 (同10→6) */
uint8_t Nav_9to2(void);       /* 9→2 直行 IR=0110 → IR=0000 */
uint8_t Nav_10to7(void);      /* 10→7 直行 IR=0110 → IR=0000 */
uint8_t Nav_6to10(void);      /* 6→10 后退巡线→0111→右转90°→10→右转90° */
uint8_t Nav_10to9(void);      /* 10→9 直行 IR=0110 → 4路全黑(9路口) */

#endif
