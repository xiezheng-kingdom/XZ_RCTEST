/**
  ******************************************************************************
  * @file    can.c
  * @brief   FDCAN1 收发驱动 (STM32H743, 标准 CAN 模式, 1 Mbps)
  ******************************************************************************
  */

#include "can.h"
#include <stdio.h>   /* CAN_DumpBusState 要打印 */

FDCAN_HandleTypeDef hfdcan1;

/* ── 发送窗口内的 CCCR 点位 ──────────────────────────────────────────────────
 * 实测: 入队前 CCCR.INIT=0, 3ms 后变成 1, 而 peakTEC 全程是 0 —— 也就是
 * **总线零活动的情况下引擎被停机了**。bus-off 做不到这件事(那需要 TEC=256),
 * 所以得在窗口内部取点, 看是哪一个调用把它翻过去的。
 *
 * ★ 这两个值只能在 CAN_SendFrame() 里赋值, 打印放到 CAN_DumpBusState() ——
 *   窗口内插一句 printf 是 4ms 的阻塞串口, 会把要观察的时序整个改掉。 */
static uint32_t s_cccr_pre  = 0U;   /* 过闸门之前 */
static uint32_t s_cccr_post = 0U;   /* HAL_FDCAN_AddMessageToTxFifoQ 返回之后 */

/* ★ TXBRP = 发送请求待办位图。这是回答"TXBAR 那次写到底生效没有"的唯一寄存器:
 *     TXBRP 对应位 = 1  -> 请求已登记, 但引擎一直没去处理它(= 引擎当时是停的)
 *     TXBRP 全 0        -> 请求压根没登记(M_CAN 规定 CCCR.INIT=1 时 TXBAR 写入被忽略),
 *                          或者登记完立刻被处理掉了
 *   配合 TXBTO(= 传输成功过) 就能三分:
 *     TXBRP!=0 && TXBTO=0  -> 请求在, 引擎没动
 *     TXBRP=0  && TXBTO=0  -> 请求根本没进去
 *     TXBTO!=0             -> 帧真的发出去过 */
static uint32_t s_txbrp_post = 0U;
static uint32_t s_txbto_post = 0U;

/* ── 翻转瞬间的现场 ──────────────────────────────────────────────────────────
 * 之前的取样点全在窗口**外**: before-gate/after-queue 是 0x1000, 3ms 后 dump 是
 * 0x1001 —— 中间隔了一个纯寄存器读循环, 于是"哪一刻翻的、翻的时候总线是什么样"
 * 一直是个黑盒。这里在 CAN_CatchFirstLec() 的采样循环里每轮都读一次 CCCR,
 * 第一次看到 INIT=1 就把当时的全部寄存器拍下来。采样间隔不到 20ns。 */
static uint32_t s_flip_iter = 0U;   /* 第几次采样抓到的, 0 = 整个窗口都没抓到 */
static uint32_t s_flip_ms   = 0U;   /* 抓到时刻距进入窗口的毫秒数 */
static uint32_t s_flip_cccr = 0U;
static uint32_t s_flip_psr  = 0U;
static uint32_t s_flip_ecr  = 0U;
static uint32_t s_flip_ir   = 0U;
static uint32_t s_flip_txbrp= 0U;
static uint32_t s_flip_txbto= 0U;
static uint32_t s_flip_txfqs= 0U;

/* ── PB8/PB9 -> AF9 FDCAN1 ───────────────────────────────────────────────────
 * 这两个脚原来是 I2C1 的 SCL/SDA (i2c.c 的 HAL_I2C_MspInit 里配成 AF4)。
 * 因为 main.c 不再调用 MX_I2C1_Init(), 那段代码不会执行, 所以这里可以独占。
 * ★ 如果哪天有人把 MX_I2C1_Init() 加回 main.c 且在 CAN_Init() 之后调用,
 *   引脚会被改回 AF4, CAN 就收不到数据了 —— 顺序和取舍见 can.h 顶部注释。 */
static void CAN_GpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ── HAL_FDCAN_Init + 过滤器 + Start, 用指定的工作模式 ────────────────────────
 *  CAN_Init() 和环回自检都走这里, 保证两条路径配出来的东西完全一致 ——
 *  自检通过才有资格代表正常模式的配置。
 *
 *  注意每次 HAL_FDCAN_Init() 之后都要重配过滤器: Init 会把消息 RAM 的各区段
 *  重新划分, 但**不会**替你写过滤器元素, 不重配的话硬件匹配表里是上一次的
 *  残留内容, 收到的帧会被莫名其妙地丢掉。 */
static void CAN_StartWithMode(uint32_t mode)
{
  FDCAN_FilterTypeDef f = {0};

  hfdcan1.Init.Mode = mode;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();

  /* 接收过滤器: 掩码 0x7FF = 11 位全比较, 即"ID 必须完全相等"才算命中。
   * 这里同时放行 0x11 和 0x22 两个 ID —— 因为主机/从机是运行时才定的
   * (见 main.c 的角色仲裁), 事先不知道本板该收哪个, 索性两个都放进来,
   * 由 can1.c / can2.c 在应用层按当前角色决定处理谁。
   * 想只收一个 ID, 把 StdFiltersNbr 改成 1 并删掉用不上的那段即可。 */
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterType   = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  f.FilterIndex  = 0U;                 /* 过滤器 0: 收 0x11 */
  f.FilterID1    = CAN_ID_REQ;
  f.FilterID2    = CAN_STD_ID_MASK;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) Error_Handler();

  f.FilterIndex  = 1U;                 /* 过滤器 1: 收 0x22 */
  f.FilterID1    = CAN_ID_ACK;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) Error_Handler();

  /* 两个过滤器都没命中的帧直接丢在硬件层, 不占 FIFO; 远程帧也一律拒收 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
}

void CAN_Init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* ★ FDCAN 外设时钟门控 (RCC->APB1HENR 的 FDCANEN 位)。
   *
   * 这一行正常属于 HAL_FDCAN_MspInit(), 由 CubeMX 生成在 stm32h7xx_hal_msp.c 里。
   * 但本工程的 .ioc 从来没配过 FDCAN, 那个文件里根本没有 FDCAN 段落, HAL 链接的是
   * 自己的空弱函数 —— 结果就是 ClockEnable 一次都没被调用过。
   *
   * 后果: FDCAN 寄存器访问全部无效, HAL_FDCAN_Init/Start 表面上不报错(返回 HAL_OK),
   * 但 hfdcan1.State 到不了 HAL_FDCAN_STATE_BUSY。之后每次
   * HAL_FDCAN_AddMessageToTxFifoQ() 都走 else 分支置 HAL_FDCAN_ERROR_NOT_STARTED
   * 并返回 HAL_ERROR —— 串口上看到的就是 "TX failed, st=1"。
   *
   * 必须放在任何 FDCAN 寄存器访问之前, 所以提到 HAL_FDCAN_Init() 前面。 */
  __HAL_RCC_FDCAN_CLK_ENABLE();

  /* ── FDCAN 内核时钟源: ★ 必须用 HSE, 不能用 PLL1Q ─────────────────────────
   *
   * 这里原本写的是 RCC_FDCANCLKSOURCE_PLL(即 PLL1Q = 400MHz), 那是纸面上更
   * 漂亮的选择 —— 400MHz/25 = 16MHz 的 tq 时钟, 16 tq 一位, 采样点正好 87.5%。
   * 但在这块板子上 **PLL1Q 这一路根本不给 FDCAN 供时钟**。
   *
   * 实测(CPU 停住, 纯 pyocd 读写寄存器, 排除一切软件干扰):
   *     RCC_D2CCIP1R.FDCANSEL = 01 (PLL1Q) -> CCCR.INIT 清 0 之后引擎也不动:
   *         PSR 永远是复位值 0x00000707 (ACT=00 "Synchronizing", 连 11 个隐性位
   *         都同步不上), ECR 恒为 0, TSCV 连续读四次纹丝不动,
   *         发送 FIFO 里的帧永远不被取走 —— 环回模式下也一样。
   *     RCC_D2CCIP1R.FDCANSEL = 00 (HSE)   -> 同样配置下一切正常:
   *         PSR.ACT 跳到 01 "Idle", TSCV 稳定递增, 引擎活。
   *   两者只差 mux 一位, 其余寄存器完全一致, 所以因果关系是确定的。
   *
   * 而时钟树本身查不出毛病: RCC_CR 里 HSEON/HSERDY/PLL1ON/PLL1RDY 全置位,
   * RCC_CFGR 的 SW/SWS 都是 11 (= PLL1), 说明 SYSCLK 就是 PLL1P —— PLL1 是锁定
   * 的、VCO 是活的, 同一个 VCO 分出来的 PLL1Q 按理不该不存在。PLLCFGR 的
   * DIVQ1EN(bit17) 也确实是 1, FDCANEN 也确实是 1。原因没能定位到寄存器一级,
   * 但现象可复现, 且改用 HSE 之后一切正常, 所以工程上就这么办。
   *
   * ★ 换 HSE 的代价: 时钟从 400MHz 降到 25MHz, tq 精度变粗, 位时序要重算
   *   (见下面 NominalPrescaler 那一段)。好处是彻底不依赖 PLL1Q。
   *
   * ★ 注意 RCC_FDCANCLKSOURCE_HSE 定义成 0x00000000, 而 HAL_RCCEx_PeriphCLKConfig
   *   内部是 MODIFY_REG(..., FDCANSEL, 0) —— 传 0 是有效的, 不是"没传参",
   *   别看到 0 就以为这行没生效。 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_HSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

  CAN_GpioInit();

  hfdcan1.Instance                  = FDCAN1;
  hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;   /* 标准 CAN, 非 FD */
  hfdcan1.Init.AutoRetransmission   = ENABLE;                /* 仲裁丢失/出错自动重发 */
  hfdcan1.Init.TransmitPause        = DISABLE;
  hfdcan1.Init.ProtocolException    = DISABLE;
  /* 位时序 (内核时钟 = HSE = 25MHz):
   *   NominalPrescaler = 1  ->  NBRP = 0, tq 时钟 = 25MHz/1 = 25MHz, 1 tq = 40ns
   *   1 位 = Sync(1) + Seg1(21) + Seg2(3) = 25 tq = 25 × 40ns = 1000ns = 1 Mbps
   *   采样点 = (1+21)/25 = 88%
   *
   * ★ 25MHz 除不出 16MHz, 所以不能照搬 PLL1Q 那套 "16 tq" 的分法, 得走 25 tq。
   *   数一下位时间: 25MHz 下一个位时间就是 25 个 tq, 正好 1us, 所以这样分是精确的。
   *   Seg1 给 21 而不是 20, 是为了把采样点从 84% 抬到 88% (CiA 推荐 87.5%)。
   *   SJW 上限 = min(Seg1, Seg2) = 3, 所以填 3 是能取到的最大值。 */
  hfdcan1.Init.NominalPrescaler     = 1U;
  hfdcan1.Init.NominalSyncJumpWidth = 3U;
  hfdcan1.Init.NominalTimeSeg1      = 21U;
  hfdcan1.Init.NominalTimeSeg2      = 3U;
  /* 以下 Data* 字段只在 CAN FD 格式下生效, 这里保留 HAL 要求的最小合法值 */
  hfdcan1.Init.DataPrescaler        = 1U;
  hfdcan1.Init.DataSyncJumpWidth    = 1U;
  hfdcan1.Init.DataTimeSeg1         = 1U;
  hfdcan1.Init.DataTimeSeg2         = 1U;
  hfdcan1.Init.MessageRAMOffset     = 0U;
  hfdcan1.Init.StdFiltersNbr        = 2U;   /* 两个: 0x11 和 0x22, 见下面的过滤器配置 */
  hfdcan1.Init.ExtFiltersNbr        = 0U;
  hfdcan1.Init.RxFifo0ElmtsNbr      = 16U;
  hfdcan1.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr      = 0U;
  hfdcan1.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr         = 0U;
  hfdcan1.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr          = 0U;
  hfdcan1.Init.TxBuffersNbr         = 0U;
  hfdcan1.Init.TxFifoQueueElmtsNbr  = 8U;
  hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;

  CAN_StartWithMode(FDCAN_MODE_NORMAL);
}

/* ── 发送前的硬件闸门 ────────────────────────────────────────────────────────
 * ★ HAL_FDCAN_AddMessageToTxFifoQ() 只检查软件变量 hfdcan1.State, **从不看硬件的
 *   CCCR.INIT**。所以即使协议引擎已经停机, 它照样返回 HAL_OK、TXBAR 也照样写进去,
 *   帧却永远静静地烂在发送 FIFO 里。
 *
 *   实测踩到过: 按 KEY1 之后抓到的现场是
 *       CCCR=0x1001 (INIT=1)  TXBTO=0  ECR=0  IR.BO=0  帧还占着 FIFO
 *   也就是"发出去没人回 ACK"和"压根没发"这两种情况, 从返回值上根本分不出来 ——
 *   串口打印的 "sent ID=0x11" 是假的, 总线上什么都没有。
 *
 *   这里在入队前确认引擎真的在跑; 不在跑就重来一遍 CAN_Init()。重完仍然停着
 *   就返回 HAL_ERROR, 让上层把那句话打成失败, 而不是继续撒谎。 */
static HAL_StatusTypeDef CAN_EnsureRunning(void)
{
  if ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U) return HAL_OK;

  /* 打印现场 —— 这是"发帧那一刻引擎为什么停着"的唯一直接证据。
   * IR.bit25(BO) 把"真的 bus-off 过"和"别的原因把 INIT 置起来了"分开:
   * 真 bus-off 一定会留下 IR.BO=1, 而 ECR 事后归零说明不了任何事。 */
  {
    uint32_t cccr = hfdcan1.Instance->CCCR;
    uint32_t ir   = hfdcan1.Instance->IR;

    printf("[CAN] engine halted AT TX TIME! CCCR=0x%08lX PSR=0x%08lX ECR=0x%08lX IR=0x%08lX (%s) -> re-init\r\n",
           (unsigned long)cccr,
           (unsigned long)hfdcan1.Instance->PSR,
           (unsigned long)hfdcan1.Instance->ECR,
           (unsigned long)ir,
           ((ir & FDCAN_IR_BO) != 0U) ? "real bus-off" : "NOT bus-off");
  }

  CAN_Init();   /* 整份重来, 顺带把发送 FIFO 和消息 RAM 一起清干净 */

  return ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U) ? HAL_OK : HAL_ERROR;
}

/* 收发共用的帧头组装 + 入队 */
static HAL_StatusTypeDef CAN_SendFrame(uint32_t  id,
                                       const uint8_t *data,
                                       uint8_t   len,
                                       uint32_t  idType)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t buf[CAN_MAX_DATA_LEN] = {0};

  if (len > CAN_MAX_DATA_LEN) return HAL_ERROR;
  if (data == NULL && len > 0U) return HAL_ERROR;

  for (uint8_t i = 0; i < len; i++) buf[i] = data[i];

  txHeader.Identifier          = id;
  txHeader.IdType              = idType;
  txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  /* ★ DataLength 填的是"DLC 码", 不是字节数左移。
   *   FDCAN_DLC_BYTES_n 就定义为 n (见 stm32h7xx_hal_fdcan.h):
   *     FDCAN_DLC_BYTES_1 = 1, FDCAN_DLC_BYTES_8 = 8, FDCAN_DLC_BYTES_12 = 9 (经典 CAN 映射)。
   *   往硬件搬的时候 HAL 自己会移: TxElementW2 |= DataLength << 16。
   *   经典 CAN 每帧最多 8 字节, DLC 码恰好等于字节数, 所以直接填 len 即可
   *   (上面已挡掉 len > 8)。
   *
   *   ⚠️ 千万别写成 len << 16 —— HAL_FDCAN_AddMessageToTxFifoQ 会把它原样传给
   *      FDCAN_CopyMessageToRAM, 那里拿它去索引只有 16 个元素的 DLCtoBytes[],
   *      越界读出垃圾长度, 再按那个长度把 pTxData 读穿 —— 直接 BusFault。 */
  txHeader.DataLength          = (uint32_t)len;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker       = 0U;

  /* 引擎停着的话 HAL 会假装成功 —— 先把闸门过一遍, 见 CAN_EnsureRunning() */
  s_cccr_pre = hfdcan1.Instance->CCCR;

  if (CAN_EnsureRunning() != HAL_OK) return HAL_ERROR;

  {
    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, buf);

    /* 入队之后立刻取点: CCCR 看引擎还在不在, TXBRP 看"发送请求"有没有登记上 */
    s_cccr_post  = hfdcan1.Instance->CCCR;
    s_txbrp_post = hfdcan1.Instance->TXBRP;
    s_txbto_post = hfdcan1.Instance->TXBTO;
    return st;
  }
}

HAL_StatusTypeDef CAN_SendStd(uint16_t id, const uint8_t *data, uint8_t len)
{
  if (id > CAN_MAX_DLC_ID) return HAL_ERROR;
  return CAN_SendFrame((uint32_t)id, data, len, FDCAN_STANDARD_ID);
}

HAL_StatusTypeDef CAN_SendExt(uint32_t id, const uint8_t *data, uint8_t len)
{
  if (id > 0x1FFFFFFFU) return HAL_ERROR;
  return CAN_SendFrame(id, data, len, FDCAN_EXTENDED_ID);
}

uint8_t CAN_Receive(CAN_FrameTypeDef *frame)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};

  if (frame == NULL) return 0U;
  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) return 0U;

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, frame->data) != HAL_OK)
  {
    return 0U;
  }

  /* 只接数据帧, 远程帧在全局过滤器里已被拒收, 这里是双保险 */
  if (rxHeader.RxFrameType != FDCAN_DATA_FRAME) return 0U;

  frame->id  = rxHeader.Identifier;
  frame->ext = (rxHeader.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
  /* ★ DataLength 同样是 DLC 码, 不是"长度左移 16"。
   *   HAL_FDCAN_GetRxMessage 填的是 (报文元素字 & DLC 掩码) >> 16,
   *   见 stm32h7xx_hal_fdcan.c:3084。
   *   经典 CAN 下 DLC 码 0..8 就等于字节数;
   *   9..15 是 CAN FD 才有的尺寸 (12/16/20/24/32/48/64), 本工程只用经典帧,
   *   真收到就一律夹到 8, 不让 len 越界。 */
  frame->len = (rxHeader.DataLength <= 8U) ? (uint8_t)rxHeader.DataLength
                                           : (uint8_t)CAN_MAX_DATA_LEN;

  return 1U;
}

uint32_t CAN_RxPending(void)
{
  return HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  bus-off 恢复
 * ────────────────────────────────────────────────────────────────────────────
 *  M_CAN 进入 bus-off 时会自己把 CCCR.INIT 置 1, 协议引擎随即停机: 之后入队的
 *  帧永远发不出去, 而且**硬件不会自己恢复**, 必须由软件清 INIT。
 *
 *  bus-off 有多容易触发: 只要总线上没有第二个节点回 ACK, 每发一帧 TEC 就 +8,
 *  AutoRetransmission 又让它一直重发, 32 次(约 2ms) 后 TEC 到 256 -> bus-off。
 *  也就是说"对面没上电 / 没接线 / 波特率不对 / 只有一块板"都会走到这里,
 *  调 CAN 的时候几乎必然踩到一次。没有这段恢复代码, 踩一次 CAN 就永久哑了。
 * ──────────────────────────────────────────────────────────────────────────── */
uint8_t CAN_RecoverIfBusOff(CAN_FaultInfoTypeDef *fault)
{
  /* 只认 CCCR.INIT, 不认 PSR.BO:
   * BO 位在总线静默 129×11 位后由硬件自动清掉, 等主循环轮询到时多半已经没了,
   * 拿它当判据会漏。INIT 则是"停机"这个事实本身。 */
  if ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U) return 0U;

  if (fault != NULL)
  {
    /* IR 先读: 它读不清除, 而且 bit25=BO 是"到底有没有真进过 bus-off"的唯一硬证据。
     * PSR 读一次就把 LEC 抹了, 所以它只读这一遍。 */
    fault->ir    = hfdcan1.Instance->IR;
    fault->cccr  = hfdcan1.Instance->CCCR;
    fault->psr   = hfdcan1.Instance->PSR;
    fault->ecr   = hfdcan1.Instance->ECR;
    fault->txfqs = hfdcan1.Instance->TXFQS;
  }

  /* 走完整的 CAN_Init() 而不是只 HAL_FDCAN_Stop() + HAL_FDCAN_Start():
   * bus-off 时那帧还赖在发送 FIFO 里(TFQPI=1), 只重启的话它会被立刻重发,
   * 结果照样没人回 ACK, 2ms 后再次 bus-off —— 变成每轮一次的死循环抖动。
   * CAN_Init() 里的 HAL_FDCAN_Init() 会把消息 RAM 各区段重配一遍, FIFO 索引
   * 也跟着归零, 相当于干净重来。 */
  CAN_Init();
  return 1U;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  环回自检 —— 回答"到底是 MCU 的问题还是 TJA1050 的问题"
 * ────────────────────────────────────────────────────────────────────────────
 *  环回模式下发送端在芯片内部接到接收端, 不经过 PB8/PB9, 因而完全不经过收发器;
 *  节点还会给自己回 ACK, 所以帧能真正"发成功", 不需要总线上有第二个节点。
 *
 *    通过 -> MCU/FDCAN 侧(时钟、位时序、消息 RAM)全都没问题,
 *            故障一定在收发器或总线上;
 *    不通过 -> 跟 TJA1050 无关, 先把 MCU 侧修好。
 *
 *  ★ 不要手写 CCCR/TEST 的位 —— TEST 寄存器是**写保护**的, 必须先置
 *    CCCR.TEST=1 才有权写 TEST.LBCK。顺序反了 LBCK 会被静默丢弃, 于是"环回自检"
 *    实际上是在真总线上发帧, 没有 ACK, 2ms 后 bus-off, 结果只会得到 FAIL,
 *    而且这个 FAIL 完全不说明 MCU 有没有问题。(这个坑踩过一次。)
 *    这里直接用 ST 官方定义的 FDCAN_MODE_* 常量, 由 HAL 去写那串位。
 * ──────────────────────────────────────────────────────────────────────────── */
#define CAN_SELFTEST_TIMEOUT_MS   50U

static uint8_t CAN_LoopbackTry(uint32_t mode, CAN_SelfTestInfoTypeDef *info)
{
  const uint8_t    payload[1] = { 0xA5U };
  CAN_FrameTypeDef rx;
  uint8_t          ok = 0U;

  CAN_StartWithMode(mode);

  /* 记下现场: TEST.bit4(LBCK) 必须是 1, 否则这次"环回"根本没生效 */
  if (info != NULL)
  {
    info->cccr = hfdcan1.Instance->CCCR;
    info->test = hfdcan1.Instance->TEST;
  }

  if (CAN_SendStd(CAN_ID_REQ, payload, 1U) == HAL_OK)
  {
    /* ★ 先抓 LEC 再打印/延时: 若环回没生效, 帧会跑到真总线上, 2ms 内就 bus-off,
     *   LEC 会被抹掉。这里边等边盯, 失败时至少能看出是"没人回 ACK"(3)。 */
    for (uint32_t ms = 0U; (ms < CAN_SELFTEST_TIMEOUT_MS) && (ok == 0U); ms++)
    {
      /* 这 1ms 拿来死盯 LEC, 顺便当延时; 不要用 HAL_Delay —— 它一整毫秒
       * 只在最后采一次, 而 LEC 是读后即清的, 那样基本什么都抓不到。 */
      uint32_t lec = CAN_CatchFirstLec(1U);

      if ((info != NULL) && (lec != 7U) && (info->lec == 7U)) info->lec = lec;

      if (CAN_Receive(&rx) != 0U)
      {
        ok = ((rx.id == CAN_ID_REQ) && (rx.ext == 0U) &&
              (rx.len == 1U) && (rx.data[0] == 0xA5U)) ? 1U : 0U;
        break;
      }
    }
  }

  /* ★ 现场快照。放在这里而不是循环里: 上面的 CAN_CatchFirstLec 已经先把 LEC 抓走了,
   *   而 IR/TXBTO 是粘滞的(读不清除、写 1 才清), 什么时候读都不会丢。
   *   判读方法:
   *     TC=1    -> 帧确实发出去了(环回模式下还会自动 ACK), 那么收不到就是收的问题
   *     TC=0    -> 帧根本没上总线, 跟 TJA1050 无关, 是 MCU 侧的问题
   *     BO=1    -> 这次"环回"其实跑到真总线上去了, LBCK 没生效 */
  CAN_DumpBusState((mode == FDCAN_MODE_INTERNAL_LOOPBACK) ? "int-lpbk" : "ext-lpbk");

  /* 自检那一帧是我们自己发的, 别留在 FIFO 里被上层当成"从机收到了 0x11" */
  while (CAN_Receive(&rx) != 0U) { }

  return ok;
}

uint8_t CAN_SelfTest(CAN_SelfTestInfoTypeDef *info)
{
  uint8_t ok;

  if (info != NULL)
  {
    info->tries = 0U; info->cccr = 0U; info->test = 0U; info->lec = 7U;
  }

  /* 两种环回定义都试一下:
   *   INTERNAL_LOOPBACK = TEST + MON + LBCK, 节点在真总线上保持静默;
   *   EXTERNAL_LOOPBACK = TEST + LBCK,       节点会真的去驱动总线。
   * 哪个能自发自收都说明 MCU 侧没问题, 所以两个都试, 免得因为 MON 的语义
   * 判断错而误报。 */
  ok = CAN_LoopbackTry(FDCAN_MODE_INTERNAL_LOOPBACK, info);
  if (info != NULL) info->tries = 1U;

  if (ok == 0U)
  {
    ok = CAN_LoopbackTry(FDCAN_MODE_EXTERNAL_LOOPBACK, info);
    if (info != NULL) info->tries = 2U;
  }

  /* 不管成败都得把外设还原成正常模式, 否则后面的业务收发全废 */
  CAN_Init();

  return ok;
}

/* ★ 死盯窗口里 TEC/REC 的峰值。
 *
 * PSR.LEC 靠不住(读后即清、bus-off 还会把它抹平), 但 **ECR 不是读后即清的** ——
 * 所以这两个峰值是可以事后采信的硬证据:
 *     peakTEC 从 0 一路爬(8, 16, 24 ... 256)  -> 帧真的发出去了, 只是没人回 ACK
 *     peakTEC 自始至终是 0                     -> 引擎压根没往外发, 问题在 MCU 侧
 * TEC 每失败一次 +8, 32 次到 256 就是 bus-off, 也就是为什么"总线上只有一块板"
 * 必然走向 bus-off。 */
static uint32_t s_peak_tec = 0U;
static uint32_t s_peak_rec = 0U;

uint32_t CAN_CatchFirstLec(uint32_t window_ms)
{
  uint32_t start = HAL_GetTick();
  uint32_t lec   = (uint32_t)(FDCAN_PSR_LEC_Msk >> FDCAN_PSR_LEC_Pos);   /* 默认 7 */
  uint32_t got   = 0U;
  uint32_t n     = 0U;   /* 采样次数, 用来标定翻转发生在第几次(间隔 <20ns) */

  s_peak_tec = 0U;
  s_peak_rec = 0U;
  s_flip_iter = 0U;   /* 0 = 这个窗口里没抓到引擎停机 */

  /* ★ 必须"死盯"而不是隔一会儿看一眼: M_CAN 的 PSR.LEC 是**读后即清**的
   *   (读一次就回到 7=NoChange), bus-off 时更是直接抹平。隔几十微秒再看就已经
   *   什么都看不到了 —— 这也是之前几次全靠事后读寄存器, 始终读不到出错原因的
   *   原因。这里循环体只有一次读 + 两次比较, 400MHz 下采样间隔不到 20ns。
   *
   *   window_ms = 0 表示"只采样一次就走", 用作廉价的探针。写成 for(;;) 而不是
   *   while(tick-start < window) 就是为了保证 window_ms=0 时也真的读上一次。 */
  for (;;)
  {
    uint32_t psr  = hfdcan1.Instance->PSR;      /* 读一次 LEC 就被抹掉, 所以整字读下来留档 */
    uint32_t ecr  = hfdcan1.Instance->ECR;
    uint32_t cccr = hfdcan1.Instance->CCCR;     /* ★ 每轮都看一眼 INIT —— 抓翻转瞬间 */
    uint32_t v    = psr & FDCAN_PSR_LEC_Msk;
    uint32_t tec  = ecr & 0xFFU;
    uint32_t rec  = (ecr >> 8) & 0xFFU;

    if (tec > s_peak_tec) s_peak_tec = tec;   /* TEC 是一路爬上去的, 得盯峰值 */
    if (rec > s_peak_rec) s_peak_rec = rec;

    /* 记下第一个具体错误码, 但**不提前退出** —— 早期版本一看到错就 break,
     * 结果只知道"出过错", 不知道 TEC 最后爬到了多少(那个数才说明重发了多少次)。 */
    if ((v != FDCAN_PSR_LEC_Msk) && (got == 0U))   /* 7 = NoChange, 还没出过错 */
    {
      lec = v >> FDCAN_PSR_LEC_Pos;
      got = 1U;
    }

    /* ★ 第一次看到引擎停机: 把那一瞬的全部寄存器拍下来。
     *   这里不再 break —— 窗口还没用完, 继续跑不花任何代价, 而且能顺带确认
     *   "停机之后 TEC/REC 有没有动过"。 */
    if (((cccr & FDCAN_CCCR_INIT) != 0U) && (s_flip_iter == 0U))
    {
      s_flip_iter  = n;
      s_flip_ms    = (uint32_t)(HAL_GetTick() - start);
      s_flip_cccr  = cccr;
      s_flip_psr   = psr;
      s_flip_ecr   = ecr;
      s_flip_ir    = hfdcan1.Instance->IR;
      s_flip_txbrp = hfdcan1.Instance->TXBRP;
      s_flip_txbto = hfdcan1.Instance->TXBTO;
      s_flip_txfqs = hfdcan1.Instance->TXFQS;
    }

    n++;

    if ((uint32_t)(HAL_GetTick() - start) >= window_ms) break;   /* 窗口用完就收工 */
  }

  return lec;
}

void CAN_DumpBusState(const char *tag)
{
  /* ★ 读 PSR 会清掉 LEC —— 这个函数必须在 CAN_CatchFirstLec() 之后调用。
   *   IR 读不清除, TXBTO 只增不减, 这两个才是不会消失的证据。 */
  uint32_t psr   = hfdcan1.Instance->PSR;
  uint32_t ecr   = hfdcan1.Instance->ECR;
  uint32_t ir    = hfdcan1.Instance->IR;
  uint32_t cccr  = hfdcan1.Instance->CCCR;
  uint32_t txfqs = hfdcan1.Instance->TXFQS;
  uint32_t txbto = hfdcan1.Instance->TXBTO;   /* 哪个发送槽真的传输出去过 */
  uint32_t rxf0s = hfdcan1.Instance->RXF0S;

  printf("[CAN] %-8s CCCR=%08lX PSR=%08lX ECR=%08lX IR=%08lX\r\n",
         tag, (unsigned long)cccr, (unsigned long)psr,
         (unsigned long)ecr, (unsigned long)ir);

  printf("[CAN] %-8s TXFQS=%08lX TXBTO=%08lX RXF0S=%08lX | "
         "TC=%lu RF0N=%lu BO=%lu EP=%lu EW=%lu MRAF=%lu | "
         "peakTEC=%lu peakREC=%lu\r\n",
         tag, (unsigned long)txfqs, (unsigned long)txbto, (unsigned long)rxf0s,
         (unsigned long)((ir & FDCAN_IR_TC)   ? 1U : 0U),
         (unsigned long)((ir & FDCAN_IR_RF0N) ? 1U : 0U),
         (unsigned long)((ir & FDCAN_IR_BO)   ? 1U : 0U),
         (unsigned long)((ir & FDCAN_IR_EP)   ? 1U : 0U),
         (unsigned long)((ir & FDCAN_IR_EW)   ? 1U : 0U),
         (unsigned long)((ir & FDCAN_IR_MRAF) ? 1U : 0U),
         (unsigned long)s_peak_tec, (unsigned long)s_peak_rec);

  /* 发送窗口内的两个点位: 哪一步把 INIT 翻上去的, 看这行就知道。
   * 两个都是 0x...00 而上面 CCCR 是 0x...01 -> 是入队**之后**才翻的。 */
  printf("[CAN] %-8s tx-window: CCCR before-gate=%08lX after-queue=%08lX "
         "TXBRP=%08lX TXBTO=%08lX\r\n",
         tag, (unsigned long)s_cccr_pre, (unsigned long)s_cccr_post,
         (unsigned long)s_txbrp_post, (unsigned long)s_txbto_post);

  /* ★ 翻转瞬间。这是整个排查里最关键的一行:
   *     FLIP 有值  -> 引擎是在采样窗口内被停机的, 且给出了第几次采样、过了几毫秒
   *     FLIP 没抓到 -> 窗口内 INIT 全程是 0, 那么停机发生在窗口**之后**(即在
   *                    紧跟着的那几次 printf 阻塞串口期间), 方向完全不同 */
  if (s_flip_iter != 0U)
  {
    printf("[CAN] %-8s FLIP! sample #%lu  +%lums  CCCR=%08lX PSR=%08lX ECR=%08lX IR=%08lX\r\n",
           tag, (unsigned long)s_flip_iter, (unsigned long)s_flip_ms,
           (unsigned long)s_flip_cccr, (unsigned long)s_flip_psr,
           (unsigned long)s_flip_ecr, (unsigned long)s_flip_ir);

    printf("[CAN] %-8s FLIP regs: TXBRP=%08lX TXBTO=%08lX TXFQS=%08lX "
           "(%s)\r\n",
           tag, (unsigned long)s_flip_txbrp, (unsigned long)s_flip_txbto,
           (unsigned long)s_flip_txfqs,
           ((s_flip_txbrp != 0U) && (s_flip_txbto == 0U))
             ? "request pending, engine never serviced it"
             : ((s_flip_txbto != 0U) ? "frame DID transmit"
                                     : "TXBAR never registered"));
  }
  else
  {
    printf("[CAN] %-8s FLIP: not seen in window (INIT stayed 0)\r\n", tag);
  }
}
