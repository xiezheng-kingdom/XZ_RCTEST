/**
 ******************************************************************************
 * @file    localization.c
 * @brief   K230 协议解析: [字段1,字段2,...,字段N]
 *
 * 帧格式:
 *   '[' + 数字(ASCII) + (',' + 数字)* + ']'
 *   数字 = 可选的 '-' + 1~5位 ASCII 十进制, 单位 0.1cm
 *
 * 模式1 QR:  [node, x, y, heading]  → 4字段 → QR数据
 * 模式2 货物: [count, x1,y1, x2,y2, ...] → 1+2N字段 → 货物数据
 ******************************************************************************
 */
#include "localization.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* K230 通信 USART: UART7 (PE7=RX, PE8=TX) */
#define K230_UART  &huart7

/* ================================================================
 * K230 控制命令
 * ================================================================ */
void K230_SwitchToQR(void)
{
    HAL_UART_Transmit(K230_UART, (uint8_t *)"a", 1, 100);  /* 'a'=QR定位码模式 */
}

void K230_SwitchToCargo(void)
{
    HAL_UART_Transmit(K230_UART, (uint8_t *)"y", 1, 100);  /* 'y'=货物扫描模式 */
}

void K230_RequestQR(void)
{
    HAL_UART_Transmit(K230_UART, (uint8_t *)"p", 1, 100);  /* 'p'=请求QR坐标 */
}

void K230_RequestCargo(void)
{
    HAL_UART_Transmit(K230_UART, (uint8_t *)"s", 1, 100);  /* 's'=请求货物坐标 */
}

/* 完整QR扫描: 切模式→请求→等待→返回 */
uint8_t K230_ScanQR(float *x, float *y, float *h, uint8_t *node)
{
    K230_SwitchToQR();
    HAL_Delay(50);
    K230_RequestQR();

    /* 等待应答, 最多 500ms */
    uint32_t t0 = HAL_GetTick();
    while (!Localization_IsQRValid()) {
        if (HAL_GetTick() - t0 > 500) return 0;
    }
    *x    = Localization_GetQR_X();
    *y    = Localization_GetQR_Y();
    *h    = Localization_GetQR_Heading();
    *node = Localization_GetQR_Node();
    return 1;
}

/* 完整货物扫描: 切模式→请求→等待→返回(第0个货物坐标) */
uint8_t K230_ScanCargo(float *x, float *y, char (*color)[8], uint8_t max)
{
    K230_SwitchToCargo();
    HAL_Delay(50);
    K230_RequestCargo();

    uint32_t t0 = HAL_GetTick();
    while (!Localization_IsCargoValid()) {
        if (HAL_GetTick() - t0 > 500) return 0;
    }
    uint8_t n = Localization_GetCargoCount();
    if (n > max) n = max;
    for (uint8_t i = 0; i < n; i++) {
        x[i] = Localization_GetCargoX(i);
        y[i] = Localization_GetCargoY(i);
        if (color) strncpy(color[i], Localization_GetCargoColor(i), 7);
    }
    return n;
}

/* ================================================================
 * 内部状态
 * ================================================================ */
static LOC_QRData    qr_data;
static LOC_CargoData cargo_data;

/* 帧解析状态 */
static uint8_t  rx_buf[LOC_FRAME_MAX_LEN];
static uint8_t  rx_idx;
static uint8_t  rx_done;
static uint8_t  in_frame;       /* 1=已在帧内 */
static uint8_t  frame_paren;    /* 1=括号帧 '('  0=方括号帧 '[' */

/* 像素→cm 转换 (需实测标定: cm_per_pixel 随距离变化) */
#define PIXEL_TO_CM_SCALE  0.05f  /* 暂用 0.05cm/pixel, 实际需标定 */

/* 解析出的整数字段 */
#define FLD_MAX  24
static int32_t  fields[FLD_MAX];
static uint8_t  fld_count;

/* ================================================================
 * 初始化
 * ================================================================ */
void Localization_Init(void)
{
    memset(&qr_data,    0, sizeof(qr_data));
    memset(&cargo_data, 0, sizeof(cargo_data));
    memset(rx_buf, 0, sizeof(rx_buf));
    rx_idx   = 0;
    rx_done  = 0;
    in_frame = 0;
    fld_count = 0;
}

/* ================================================================
 * 解析整数字段
 * ================================================================ */
static int parse_fields(void)
{
    fld_count = 0;
    memset(fields, 0, sizeof(fields));

    char *p = (char *)rx_buf;
    while (*p && fld_count < FLD_MAX) {
        /* 跳过逗号 */
        while (*p == ',') p++;
        if (!*p) break;

        /* 解析有符号整数 */
        int sign = 1;
        if (*p == '-') { sign = -1; p++; }
        if (!isdigit((int)*p)) break;

        int32_t val = 0;
        while (isdigit((int)*p)) {
            val = val * 10 + (*p - '0');
            p++;
        }
        fields[fld_count++] = sign * val;

        /* 期望逗号或结束 */
        if (*p && *p != ',') break;
    }
    return fld_count;
}

/* ================================================================
 * 逐字节喂入
 * ================================================================ */
void Localization_ProcessByte(uint8_t byte)
{
    if (rx_done) return;

    /* 帧头: '('=货物(像素)  '['=QR/货物(0.1cm) */
    if (byte == '(' || byte == '[') {
        rx_idx   = 0;
        in_frame = 1;
        frame_paren = (byte == '(') ? 1 : 0;
        return;
    }

    if (!in_frame) return;

    if (byte == ')' || byte == ']') {  /* 帧尾 */
        rx_buf[rx_idx] = '\0';
        rx_done = 1;
        in_frame = 0;

        int nf = parse_fields();
        if (nf < 2) { rx_done = 0; return; }

        /* '(' 括号帧 = 货物像素坐标 (x_pixel, y_pixel, color) */
        if (frame_paren) {
            cargo_data.count = 1;
            cargo_data.x_cm[0] = (float)fields[0] * PIXEL_TO_CM_SCALE;
            cargo_data.y_cm[0] = (float)fields[1] * PIXEL_TO_CM_SCALE;
            /* 颜色: 从原始帧提取 (parse_fields只能解析数字) */
            char *p = (char *)rx_buf;
            for (int i = 0; i < 2; i++) { while (*p && *p != ',') p++; if (*p==',') p++; }
            uint8_t ci = 0;
            while (*p && ci < 7 && *p != ')') cargo_data.color[0][ci++] = *p++;
            cargo_data.color[0][ci] = '\0';
            cargo_data.valid = 1;
            cargo_data.frame_count++;
        }
        /* '[' 方括号帧 = QR/货物(0.1cm单位) */
        else if (nf == 4) {
            /* 模式1: [node, x, y, heading] */
            qr_data.node       = (uint8_t)fields[0];
            qr_data.x_cm       = (float)fields[1] * 0.1f;
            qr_data.y_cm       = (float)fields[2] * 0.1f;
            qr_data.heading_deg = (float)fields[3];
            qr_data.valid      = 1;
            qr_data.frame_count++;
        } else if (nf >= 4 && (nf % 3 == 1)) {
            /* 模式2: [count, x1,y1,color1, x2,y2,color2, ...]  每货物3字段 */
            uint8_t cnt = (uint8_t)fields[0];
            if (cnt > LOC_CARGO_MAX) cnt = LOC_CARGO_MAX;
            cargo_data.count = cnt;
            /* 颜色需要直接从原始帧字符串解析 (fields是数字) */
            char *p = (char *)rx_buf;
            /* 跳过 count */
            while (*p && *p != ',') p++; if (*p==',') p++;
            for (uint8_t i = 0; i < cnt; i++) {
                /* 解析 x */
                while (*p == ',') p++;
                cargo_data.x_cm[i] = (float)atoi(p) * 0.1f;
                while (*p && *p != ',') p++; if (*p==',') p++;
                /* 解析 y */
                while (*p == ',') p++;
                cargo_data.y_cm[i] = (float)atoi(p) * 0.1f;
                while (*p && *p != ',') p++; if (*p==',') p++;
                /* 解析 color */
                while (*p == ',') p++;
                char *c = cargo_data.color[i];
                uint8_t ci = 0;
                while (*p && *p != ',' && ci < 7) c[ci++] = *p++;
                c[ci] = '\0';
            }
            cargo_data.valid = 1;
            cargo_data.frame_count++;
        }
        /* 非法帧 → 丢弃 */
        rx_done = 0;
        return;
    }

    if (rx_idx < LOC_FRAME_MAX_LEN - 1) {
        rx_buf[rx_idx++] = byte;
    } else {
        /* 溢出 */
        rx_idx   = 0;
        in_frame = 0;
    }
}

/* ================================================================
 * QR 数据查询
 * ================================================================ */
uint8_t Localization_IsQRValid(void)    { uint8_t v=qr_data.valid; qr_data.valid=0; return v; }
float   Localization_GetQR_X(void)      { return qr_data.x_cm; }
float   Localization_GetQR_Y(void)      { return qr_data.y_cm; }
float   Localization_GetQR_Heading(void){ return qr_data.heading_deg; }
uint8_t Localization_GetQR_Node(void)   { return qr_data.node; }

/* ================================================================
 * 货物数据查询
 * ================================================================ */
uint8_t Localization_IsCargoValid(void) { uint8_t v=cargo_data.valid; cargo_data.valid=0; return v; }
uint8_t Localization_GetCargoCount(void){ return cargo_data.count; }
float   Localization_GetCargoX(uint8_t idx) { return (idx<LOC_CARGO_MAX)?cargo_data.x_cm[idx]:0; }
float   Localization_GetCargoY(uint8_t idx) { return (idx<LOC_CARGO_MAX)?cargo_data.y_cm[idx]:0; }
const char* Localization_GetCargoColor(uint8_t idx) { return (idx<LOC_CARGO_MAX)?cargo_data.color[idx]:""; }

/* ================================================================
 * 通用
 * ================================================================ */
uint8_t Localization_IsDataValid(void)  { return qr_data.valid || cargo_data.valid; }
