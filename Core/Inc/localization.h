/**
 ******************************************************************************
 * @file    localization.h
 * @brief   K230 视觉定位数据解析 (QR码 + 货物坐标)
 *
 * 协议格式: [字段1,字段2,...,字段N]
 *   起始 '[', 分隔 ',', 结束 ']', 数字为 ASCII 整数 (单位: 0.1cm)
 *
 * 模式1 — QR码扫描 (K230 → STM32):
 *   [节点号, X_01cm, Y_01cm, 航向_deg]
 *   例: [1,1205,340,90] → 节点1, X=120.5cm, Y=34.0cm, 航向90°
 *
 * 模式2 — 货物扫描 (K230 → STM32):
 *   [数量, X1, Y1, COLOR1, X2, Y2, COLOR2, ...]
 *   例: [2,234,567,RED,345,678,BLUE] → (23.4,56.7,RED) (34.5,67.8,BLUE)
 *
 * 控制协议 (STM32 → K230, 请求-应答模式):
 *   货物: 'y'→切货物模式  's'→请求货物坐标 → 返 [cnt,x1,y1,c1,...]
 *   QR:   'a'→切QR定位码   'p'→请求QR坐标  → 返 [node,x,y,h]
 *
 * 数据流:
 *   Localization_ProcessByte() 逐字节解析 [...] 帧
 ******************************************************************************
 */
#ifndef __LOCALIZATION_H__
#define __LOCALIZATION_H__

#include "main.h"

/* ================================================================
 * 定位数据
 * ================================================================ */
#define LOC_FRAME_MAX_LEN    128    /* 最大帧长 */
#define LOC_CARGO_MAX        8      /* 单次最多扫描8个货物 */

/* QR 扫描结果 (模式1) */
typedef struct {
    uint8_t node;           /* 节点号 1~10 */
    float   x_cm;           /* X (cm, 小车相对QR) */
    float   y_cm;           /* Y (cm) */
    float   heading_deg;    /* 航向角 (度) */
    uint8_t valid;          /* 1=有效数据 */
    uint32_t frame_count;
} LOC_QRData;

/* 货物扫描结果 (模式2) */
typedef struct {
    uint8_t count;
    float   x_cm[LOC_CARGO_MAX];
    float   y_cm[LOC_CARGO_MAX];
    char    color[LOC_CARGO_MAX][8];    /* 颜色字符串 */
    uint8_t valid;
    uint32_t frame_count;
} LOC_CargoData;

/* ================================================================
 * API
 * ================================================================ */

void     Localization_Init(void);
void     Localization_ProcessByte(uint8_t byte);

/* ---- K230 控制命令 (STM32 → K230, 通过 USART 发送) ---- */
void     K230_SwitchToQR(void);        /* 发送 'y', 切换到QR定位码模式 */
void     K230_SwitchToCargo(void);     /* 发送 'a', 切换到货物扫描模式 */
void     K230_RequestQR(void);         /* 发送 's', 请求QR坐标 */
void     K230_RequestCargo(void);      /* 发送 'p', 请求货物坐标 */
uint8_t  K230_ScanQR(float *x, float *y, float *h, uint8_t *node); /* 完整QR扫描流程 */
uint8_t  K230_ScanCargo(float *x, float *y, char (*color)[8], uint8_t max);

/* QR 数据 */
uint8_t  Localization_IsQRValid(void);
float    Localization_GetQR_X(void);
float    Localization_GetQR_Y(void);
float    Localization_GetQR_Heading(void);
uint8_t  Localization_GetQR_Node(void);

/* 货物数据 */
uint8_t  Localization_IsCargoValid(void);
uint8_t  Localization_GetCargoCount(void);
float    Localization_GetCargoX(uint8_t idx);
float    Localization_GetCargoY(uint8_t idx);
const char* Localization_GetCargoColor(uint8_t idx);

/* 通用 */
uint8_t  Localization_IsDataValid(void);  /* QR 或 货物任一有效 */

#endif
