/**
  ******************************************************************************
  * @file    can.h
  * @brief   FDCAN1 收发驱动 (STM32H743, 标准 CAN 模式, 1 Mbps)
  ******************************************************************************
  */

#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ── 引脚 ────────────────────────────────────────────────────────────────────
 *   PB8 -> FDCAN1_RX  (AF9)
 *   PB9 -> FDCAN1_TX  (AF9)
 *
 *   ★ 这两个脚原先在 i2c.h/.c 里是 I2C1_SCL/SDA, 驱动 OLED 屏。按"停用 OLED"
 *     的决定, main.c 已不再调用 MX_I2C1_Init(), 因此 i2c.c 的 HAL_I2C_MspInit
 *     不会执行, 不会来抢这两个脚。
 *     将来若要复用 OLED, 必须把它挪到 I2C2 (PB10/PB11) 再改回来。
 *
 * ── 位时序 (1 Mbps) ─────────────────────────────────────────────────────────
 *   FDCAN 内核时钟 = HSE = 25 MHz
 *   ★ 不是 PLL1Q。原因见 can.c 的 CAN_Init(): 这块板子上 FDCANSEL=01(PLL1Q)
 *     时分频出来的 CAN 时钟域是死的 —— 引擎连同步都做不到 —— 实测换成 HSE
 *     (FDCANSEL=00) 立刻就活了。别再改回去。
 *
 *   NominalPrescaler = 1  ->  NBRP = 0, tq 时钟 = 25MHz, 1 tq = 40 ns
 *   1 bit = Sync(1) + Seg1(21) + Seg2(3) = 25 tq = 25 × 40ns = 1000 ns = 1 Mbps
 *   采样点 = (1+21)/25 = 88%   (CiA 推荐 87.5%)
 *
 *   ★ 改波特率只需改这三个参数, 保持 Sync(1)+Seg1+Seg2 的总 tq 数不变即可:
 *       1 Mbps : Prescaler 1, Seg1 21, Seg2 3
 *       500k   : Prescaler 2, Seg1 21, Seg2 3
 *       250k   : Prescaler 5, Seg1 21, Seg2 3
 *     (HSE=25MHz 除不出 16MHz, 所以总 tq 数是 25 而不是 PLL1Q 那套的 16。)
 */
#define CAN_BITRATE_BPS       1000000U

#define CAN_MAX_DATA_LEN      8U
#define CAN_MAX_DLC_ID        0x7FFU   /* 标准帧 11 位 ID 上限 */

/* ── 演示协议: 用 CAN ID 区分报文 ────────────────────────────────────────────
 *   0x11   主机 -> 从机   请求  (KEY1 按下时发出)
 *   0x22   从机 -> 主机   应答  (从机收到 0x11 后回发, 同时翻转 PA1)
 *
 * can.c 按这两个 ID 配了硬件精确匹配过滤器 (掩码 0x7FF 全比较),
 * 总线上其它 ID 的帧会被 FDCAN 外设直接丢弃, 根本不会进 FIFO。
 * 要加新报文就在这里加 ID, 并同步扩大 can.c 里 StdFiltersNbr 的数量。 */
#define CAN_ID_REQ            0x11U
#define CAN_ID_ACK            0x22U
#define CAN_STD_ID_MASK       0x7FFU

/* 一帧的载荷描述, 收和发都用它 */
typedef struct {
    uint32_t id;                        /* 标准帧 11 位, 扩展帧 29 位 */
    uint8_t  data[CAN_MAX_DATA_LEN];
    uint8_t  len;                       /* 0..8 */
    uint8_t  ext;                       /* 0 = 标准帧, 1 = 扩展帧 */
} CAN_FrameTypeDef;

extern FDCAN_HandleTypeDef hfdcan1;

/**
  * @brief  初始化 FDCAN1: 时钟源 + PB8/PB9 引脚 + 位时序 + 过滤器 + 启动
  * @note   必须在 SystemClock_Config() 之后调用 (内核时钟源用的是 HSE, 需要
  *         HAL_RCC_OscConfig() 先把 HSE 起振)。
  */
void CAN_Init(void);

/** @brief 发送标准帧 (11 位 ID)。失败返回非 HAL_OK */
HAL_StatusTypeDef CAN_SendStd(uint16_t id, const uint8_t *data, uint8_t len);

/** @brief 发送扩展帧 (29 位 ID)。失败返回非 HAL_OK */
HAL_StatusTypeDef CAN_SendExt(uint32_t id, const uint8_t *data, uint8_t len);

/**
  * @brief  从 RX FIFO0 取一帧 (非阻塞)
  * @param  frame 取到的帧写到这里
  * @retval 1 = 取到一帧, 0 = 当前 FIFO 为空
  */
uint8_t CAN_Receive(CAN_FrameTypeDef *frame);

/** @brief 当前 RX FIFO0 里积压的帧数 */
uint32_t CAN_RxPending(void);

/* ── bus-off 事故现场快照 ────────────────────────────────────────────────────
 *   ★ 抓到的都是"事后"的值, 别把它当成出错瞬间的现场:
 *     M_CAN 进入 bus-off 时会把 TEC/REC 归零, PSR.BO 也会在总线静默约 1.4ms
 *     后自己清掉, LEC 更是被抹成 7(NoChange)。所以 ecr == 0 **不代表没出过错**,
 *     真正能说明"停机了"的只有 cccr 里的 INIT 位。 */
typedef struct {
    uint32_t cccr;    /* CCCR  —— bit0 = INIT, 1 表示协议引擎已停机 */
    uint32_t psr;     /* PSR   —— LEC/ACT/BO */
    uint32_t ecr;     /* ECR   —— TEC/REC */
    uint32_t txfqs;   /* TXFQS —— 发送 FIFO 占用情况 */
    uint32_t ir;      /* IR    —— ★ bit25 = BO。和 PSR.BO 不同, IR 的位是"写 1 才清",
                       *         不会被时间抹掉, 所以它是"到底有没有真的 bus-off 过"
                       *         唯一靠得住的事后证据。bit9 = TC(发送完成),
                       *         bit0 = RF0N(收到帧)。 */
} CAN_FaultInfoTypeDef;

/**
  * @brief  检查 FDCAN1 是否已被硬件置为 INIT(即 bus-off) 并停机, 是就把整个外设
  *         重新初始化一遍 (走完整的 CAN_Init, 顺带清空发送 FIFO)
  * @param  fault 非 NULL 时写入故障瞬间的寄存器快照, 供上层打印
  * @retval 1 = 确实发生了 bus-off 且已恢复, 0 = 一切正常
  * @note   必须在主循环里周期性调用。M_CAN 自己不会从 bus-off 里爬出来 ——
  *         不调用它, 一次总线故障就永久废掉 CAN。
  */
uint8_t CAN_RecoverIfBusOff(CAN_FaultInfoTypeDef *fault);

/* 自检结果 + 失败时的现场, 供上层打印 */
typedef struct {
    uint32_t tries;   /* 试过几种环回定义 (1 或 2) */
    uint32_t cccr;    /* 最后一次尝试里读回的 CCCR, bit7 = TEST */
    uint32_t test;    /* 最后一次尝试里读回的 TEST, bit4 = LBCK —— 必须是 1 */
    uint32_t lec;     /* 自检期间抓到的第一个错误码, 7 = 一个都没抓到 */
} CAN_SelfTestInfoTypeDef;

/**
  * @brief  环回自检: 发出的帧在芯片内部绕回自己的接收端, 节点给自己回 ACK,
  *         不依赖总线上有没有第二个节点, 也不需要收发器正常工作
  * @param  info 非 NULL 时写入自检现场, 失败时用来看是哪一步没成
  * @retval 1 = 通过, 0 = 失败
  * @note   用来把"MCU/FDCAN 侧的问题"和"收发器/总线的问题"一刀切开。
  *         会依次试两种环回定义 (INTERNAL_LOOPBACK / EXTERNAL_LOOPBACK),
  *         收尾时恢复正常模式; 期间总线上可能有帧, 所以只在启动时调用一次。
  */
uint8_t CAN_SelfTest(CAN_SelfTestInfoTypeDef *info);

/**
  * @brief  在 window_ms 内死盯 PSR.LEC, 抓第一个"具体错误码"
  * @param  window_ms 盯多久 (帧在 1Mbps 下不到 130us, 几毫秒足够)
  * @retval LEC 码: 0=无错 1=填充 2=格式 3=应答 4=Bit1 5=Bit0 6=CRC 7=整个窗口
  *         没抓到任何错误
  * @note   这是唯一能区分"总线上没有第二个节点回 ACK"(LEC=3) 和
  *         "收发器没把总线拉起来"(LEC=5) 的办法 —— 这两种故障最终都会走到
  *         bus-off, 事后再读寄存器已经分辨不出来了。
  */
uint32_t CAN_CatchFirstLec(uint32_t window_ms);

/**
  * @brief  把 FDCAN1 的状态寄存器打成两行串口日志, 回答"这一帧到底发出去没有"
  * @param  tag 行首标签, 用来区分调用点
  * @note   ★ 会读 PSR, 而 PSR.LEC **读后即清** —— 必须在 CAN_CatchFirstLec()
  *         之后调用, 否则会把刚抓到的错误码抹掉。IR 是写 1 清除、读不清除,
  *         所以反复调用本函数是安全的, 也因此 IR 才适合当判据。
  *
  *         怎么读这两行:
  *           IR.TC=1            -> 帧真的发完了(并拿到 ACK)
  *           IR.BO=1            -> 真的进过 bus-off
  *           IR.RF0N=1          -> 收到过帧
  *           TC=0 且 ECR=0 且 BO=0 -> 帧压根没上总线, 问题在 MCU 侧
  *           TXBTO 对应位置 1     -> 那个发送槽确实传输出去过
  */
void CAN_DumpBusState(const char *tag);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
