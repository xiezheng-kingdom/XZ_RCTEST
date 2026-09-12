# XZ_RCTEST 开发文档

> STM32H743VIT6 双电机底盘控制工程 —— 架构设计、关键决策、踩过的坑与验证方法
>
> 当前版本 **V1.3** ｜ 2026-09-12

---

## 目录

1. [项目概述](#1-项目概述)
2. [工程架构](#2-工程架构)
3. [控制架构设计](#3-控制架构设计)
4. [关键技术决策](#4-关键技术决策)
5. [踩过的坑](#5-踩过的坑)
6. [验证方法论](#6-验证方法论)
7. [当前状态与已知问题](#7-当前状态与已知问题)
8. [构建与烧录](#8-构建与烧录)
9. [版本与仓库](#9-版本与仓库)

---

## 1. 项目概述

### 1.1 当前目标

**右电机低速精确旋转一圈，无抖动。**

轮子一圈 = 编码器 32256 counts，闭环控制目标位 = 当前位 + 32256。巡航速度 6000 counts/s ≈ 5.4 秒一圈。

### 1.2 硬件

| 部件 | 型号 | 说明 |
|---|---|---|
| 主控 | STM32H743VIT6 | Cortex-M7 @ 400MHz，2MB Flash，DTCM 128KB |
| 电机驱动 | TB6612FNG | 双 H 桥，A 通道 = 左电机，B 通道 = 右电机 |
| 电机 | JGB37-520 | 带 448 线霍尔编码器，减速比 18 |
| 显示 | OLED SSD1309 | 硬件 I2C1 |
| 调试 | ST-Link V2 | SWD，`V2J37S7` |

### 1.3 引脚映射

| 功能 | 引脚 | 外设 |
|---|---|---|
| 左电机 AIN1 / AIN2 | PC1 / PC0 | GPIO |
| 左电机 PWMA | PA0 | TIM2_CH1 |
| 右电机 BIN1 / BIN2 | PA4 / PA5 | GPIO |
| 右电机 PWMB | PB3 | TIM2_CH2 |
| 左编码器 | PE9 / PE11 | TIM1_CH1/CH2 |
| 右编码器 | PA6 / PA7 | TIM3_CH1/CH2 |
| OLED SCL / SDA | PB8 / PB9 | I2C1 |
| 遥测输出 | PD8 / PD9 | USART3 TX/RX |
| 蓝牙（模块已移除） | PA2 / PA3 | USART2 |

---

## 2. 工程架构

### 2.1 分层

```
┌─────────────────────────────────────────────────┐
│  应用层 (Core/Src/ 手写代码)                      │
│    main.c            初始化 + 10ms 遥测帧          │
│    motor_control.c   串级闭环控制器                │
│    motor.c           TB6612 驱动 + 编码器测速       │
│    oled.c            OLED 显示                    │
├─────────────────────────────────────────────────┤
│  CubeMX 生成层 (生成后手改需放 USER CODE 区)        │
│    tim.c  usart.c  i2c.c  adc.c  gpio.c           │
│    stm32h7xx_it.c    中断向量                      │
│    stm32h7xx_hal_msp.c  HAL 底层初始化              │
├─────────────────────────────────────────────────┤
│  HAL 驱动层 (Drivers/STM32H7xx_HAL_Driver/)       │
├─────────────────────────────────────────────────┤
│  CMSIS + 启动文件                                  │
└─────────────────────────────────────────────────┘
```

**分层原则**：CubeMX 生成的文件（`tim.c` / `usart.c` 等）只在 `/* USER CODE BEGIN */ ~ /* USER CODE END */` 之间手改，否则下次重新生成会被覆盖。`motor.c` / `motor_control.c` / `oled.c` 是纯手写文件，不受 CubeMX 影响。

### 2.2 文件职责

| 文件 | 职责 | 是否手写 |
|---|---|---|
| `Core/Src/main.c` | 外设初始化顺序、10ms 二进制遥测帧 | 手写（已重写） |
| `Core/Src/motor_control.c` | 位置外环 → 速度环串级控制、启动突破 | 手写 |
| `Core/Src/motor.c` | TB6612 真值表驱动、编码器位置累积与测速 | 手写 |
| `Core/Src/oled.c` | SSD1309 显示 | 手写 |
| `Core/Src/tim.c` | TIM1/2/3 配置 | CubeMX |
| `Core/Src/usart.c` | USART2/3 配置（USART3 为裸寄存器） | CubeMX + 手改 |
| `Core/Src/stm32h7xx_it.c` | 中断向量，`SysTick_Handler` 调用控制节拍 | CubeMX + USER CODE |

### 2.3 构建系统

CMake + MinGW Makefiles + arm-none-eabi-gcc。

- `CMakeLists.txt` —— 用户源文件**显式列出**在 `target_sources()`，**不是 glob**。新增/删除模块必须手动改这里。
- `cmake/stm32cubemx/CMakeLists.txt` —— CubeMX 生成的源文件列表（`MX_Application_Src`），同样是显式列表。
- 编译选项含 `-ffunction-sections -fdata-sections` + `--gc-sections`，**未引用的函数会被链接器直接丢弃**。这一点在排查"删了代码体积不变"时很关键（见 [5.12](#512-模块清理include-图不足以判定死代码)）。

---

## 3. 控制架构设计

### 3.1 串级双环

```
目标位置 ─→ [位置环 P] ─→ 目标速度 ─→ [速度环 PI] ─→ duty ─→ TB6612
             ↑                          ↑
          编码器位置                  编码器速度
```

**为什么用串级而不是单环位置 PID？**

单环位置 PID 的输出直接是 duty。位置误差在启动瞬间最大，输出立刻饱和，电机被拉满 → 机械冲击 + 过冲。串级结构把"要跑多快"（`vel_ref`）和"出多大力"（duty）解耦，中间那层速度参考**可以单独做斜率限制和减速规划**，这是单环做不到的。

### 3.2 位置外环

```c
vel_ref = MC_KP_POS * error;              /* P = 2.0 */
v_dec   = sqrtf(2 * MC_DECEL * |error|);  /* 减速规划 */
if (|vel_ref| > v_dec) vel_ref = ±v_dec;  /* 到目标前能减到 0 */
vel_ref = clamp(vel_ref, ±MC_VMAX);
```

`v_dec` 是运动学约束：在剩余距离 `error` 内以 `MC_DECEL` 减速所需的速度上限。它保证不会冲过头。

### 3.3 速度参考斜率限制

```c
float dv = MC_AMAX * (MC_PERIOD_MS / 1000.0f);   /* 8000 × 0.01 = 80 counts/s per tick */
if (vel_ref > s_vel_ref + dv) vel_ref = s_vel_ref + dv;
```

**这是为了解决启动阶跃**。原始代码里 `vel_ref = Kp × error` 在 t=0 直接跳到限幅 6000，电机被瞬间拉满，实测启动峰值 30000 counts/s。加上斜率限制后 vel_ref 用 0.75 秒爬升，启动平滑。

### 3.4 速度环：为什么用积分器而不做反电动势前馈

这是本工程最重要的一个设计决策。

**一个直觉上很合理的方案**是给速度环加反电动势前馈：`ff = k × vel_ref`，其中 `k = 1/G`（G 是 duty→速度的静态增益）。听起来能把稳态 duty 直接算出来，积分器就不用干活了。

**它的问题在于：`1/G` 必须精确已知。** G 是未知的、且随负载和电池电压变化。G 猜大了，前馈本身就把工作点推到错误的转速，闭环再来回纠正；G 猜小了，积分器要补差额。**前馈的斜率误差直接变成极限环。**

**最终方案：反电动势前馈置零，稳态 duty 完全交给积分器。**

数学上，速度环积分器 `∫ e dt` 本身就是一个**在线的 1/G 估计器** —— 它自动收敛到使速度误差为零的那个 duty，与 G 的真实值无关。代价是收敛慢一点，换来的是对 G 不敏感。

保留下来的只有**静摩擦前馈**，而且做了平滑饱和处理：

```c
ff = MC_FF_STIC * (vel_ref / (|vel_ref| + MC_FF_SMOOTH));
```

用 `x/(|x|+a)` 而不是 `copysign`：前者在 `vel_ref` 过零时**连续**，后者会造成 ±1000 的 duty 阶跃（瞬间满反向），实测到位时产生 -20000 counts/s 的反向尖峰。

### 3.5 控制周期与测速

| 项 | 值 | 说明 |
|---|---|---|
| SysTick | 1 ms | `HAL_IncTick()` 之后调用 `MotorControl_Tick()` |
| 控制周期 | 10 ms | `MotorControl_Tick()` 内分频 `MC_PERIOD_MS` |
| 测速窗口 | 10 ms | `Encoder_Update()` 与控制器同节拍 |
| 测速滤波 | EMA α = 0.60 | 等效时间常数 ≈ **10.9 ms**，平均延迟 ≈ 6.7 ms |

> ⚠️ **代码注释里的两处 τ 值都不准确**：`motor.c:120` 写 `τ≈17ms`（那是 T/α 的粗略近似），`motor_control.c:32` 写 `τ≈6.7ms`（那是平均延迟，不是时间常数）。一阶 EMA `y += α(x−y)` 的等效时间常数正确算法是 `τ = −T/ln(1−α) = −10/ln(0.4) ≈ 10.9 ms`。

编码器位置累积用 16 位差值自动处理溢出：

```c
delta = (int16_t)(cnt - last_cnt);   /* 65535→0 回绕自动正确 */
pos32 += delta;
```

### 3.6 到位处理：短路制动 + 滞回

```c
if (|error| < MC_ARRIVE_BAND) {      /* 80 counts */
    s_arrived = 1;
    s_duty = 0;
    Motor_Right_Brake();             /* IN1=IN2=1 → 短路制动 */
    return;
}
```

近空载时若用"滑行"（IN1=IN2=0），轮子会靠惯性冲过目标位置。**短路制动**把电机绕组短接，反电动势产生的制动电流使轮子自锁，同时静摩擦帮忙保持位置。

`MC_EXIT_BAND`（250 counts）是滞回：到位后只有误差超过 250 才重新接管控制，避免在死区边缘反复启停。

---

## 4. 关键技术决策

### 4.1 PWM 频率选 25 kHz

时钟链：`HSE 25MHz → PLL → SYSCLK 400MHz → HCLK 200MHz → TIMxCLK 200MHz`

```
PWM_Hz = TIM2_CLK / (PSC+1) / (ARR+1)
       = 200MHz / 1 / 8000 = 25 kHz      ← PSC=0, ARR=7999
```

选 25 kHz 的原因：JGB37-520 推荐 25~60 kHz。**低于 20kHz 会进入人耳可闻范围，且直流电机在低频 PWM 下电流脉动大**。

> ⚠️ **历史错误**：早期配置是 `PSC=199, ARR=19999` → `200M/200/20000 = 50 Hz`。那是**舵机**的频率，用在直流电机上会造成严重的低频脉动和大电流。修改 PWM 频率前必须按上式验算。

### 4.2 编码器换算

```
PPR = 448 线
4 倍频 (TIM_ENCODERMODE_TI12)  →  448 × 4 = 1792 counts / 电机轴圈
减速比 18                       →  1792 × 18 = 32256 counts / 车轮一圈
```

TI12 模式同时用 CH1 和 CH2 的上升沿与下降沿计数，所以是 4 倍频。定时器 ARR 设为 65535（16 位满量程），靠软件差值累积成 32 位位置。

### 4.3 遥测协议：VOFA+ JustFloat

选它而不是 ASCII CSV（FireWater）的原因是**带宽**：一帧 6 个 float + 4 字节帧尾 = 28 字节，115200 下 2.4 ms 发完，占用主循环 24%，能做到真正的 100 Hz 全采样。

```
帧格式: [float32 × 6] + 帧尾 {0x00, 0x00, 0x80, 0x7F}
```

帧尾 `0x00 0x00 0x80 0x7F` 按小端解释是 `+Inf`，VOFA 用它做帧同步。

**通道定义**（VOFA 里协议选 `JustFloat`，通道数填 6）：

| 通道 | 含义 |
|---|---|
| 0 | 位置 `Encoder_GetRightPosition()` |
| 1 | 实测速度 `MotorControl_GetVelocity()` |
| 2 | 速度参考 `MotorControl_GetVelRef()` |
| 3 | 实际 duty `MotorControl_GetDuty()` |
| 4 | 速度环积分 `MotorControl_GetIntegral()` |
| 5 | 目标位置 `MotorControl_GetTargetPosition()` |

**必须与控制同节拍（10 ms）发送。** 早期用 200 ms 发送，把几十 ms 的极限环混叠成了看不出真实形状的锯齿，直接导致误判（见 [5.8](#58-遥测采样率过低把震荡混叠成假波形)）。

### 4.4 USART3 走裸寄存器初始化

`usart.c` 里 USART3 **故意不调用 `HAL_UART_Init()`**（它会在 `UART_CheckIdleState()` 上超时），只手工配寄存器。后果是：

**`huart3.Init.BaudRate` 那行完全无效，真正决定波特率的只有 `USART3->BRR`。**

```c
USARTDIV = PCLK1 / (16 × baud);     /* PCLK1 = 100MHz, OVER8=0 */
/* 115200 → 54.253 → 0x364 */
USART3->BRR = 0x364;
```

**改时钟后 BRR 必须重算。** 这个坑详见 [5.7](#57-usart3-波特率长期停在-9600静默变慢且制造假波形)。

---

## 5. 踩过的坑

### 5.1 启动过猛：速度参考阶跃

**现象**：启动瞬间实测速度峰值 30000 counts/s，远超巡航 6000。

**根因**：`vel_ref = Kp_pos × error`，启动时位置误差 = 32256，`Kp=2.0` → `vel_ref = 64512`，直接顶到限幅 6000。**是阶跃而不是斜坡**，电机被瞬间拉满。

**解决**：加斜率限制 `MC_AMAX = 8000 counts/s²`，vel_ref 用 0.75 秒爬升。

**教训**：位置外环的输出天然是阶跃的。**只要速度参考是"算出来的"而不是"规划出来的"，启动就一定过猛。**

---

### 5.2 巡航剧烈振荡：前馈斜率必须精确等于 1/G

**现象**：巡航段速度在整个周期内大幅振荡。

**根因分析**：原方案用反电动势前馈 `ff = k × vel_ref` 抵消稳态 duty。但该前馈的斜率必须精确等于 `1/G`，而 **G 是未知的**。猜错 → 前馈本身把工作点推到错误转速 → 闭环来回纠正 → 极限环。

**解决**：反电动势前馈**置零**，改由积分器承担。积分器数学上等于在线的 `1/G` 估计器，自动收敛到正确稳态 duty，与 G 无关。

**教训**：**用未知参数的模型去做前馈，等于把开环误差直接注入闭环。** 能靠积分器自适应解决的事，不要靠猜参数。

---

### 5.3 到位反向尖峰：`copysign` 在过零处不连续

**现象**：接近目标位置时出现 -20000 counts/s 的反向尖峰。

**根因**：静摩擦前馈写作 `math.copysign(500, vel_ref)`。`vel_ref` 过零时，前馈从 +500 瞬间跳到 -500 —— **duty 发生 ±1000 的阶跃，等于瞬间满反向**。

**解决**：改成平滑饱和 `ff = MC_FF_STIC × vel_ref / (|vel_ref| + MC_FF_SMOOTH)`，过零处连续。

**教训**：**任何"符号函数"形式的前馈，在换向瞬间都会制造阶跃。** 换向是控制里最容易出问题的地方。

---

### 5.4 遥测恒为 0：取值路径依赖被测量本身的模式

**现象**：闭环挂起时，遥测里的速度永远是 0。

**根因**：`MotorControl_GetVelocity()` 返回缓存的 `s_vel_last`，而那个缓存的赋值点在 `if (!s_enabled) return;` **之后** —— 闭环一挂起，赋值就被跳过，缓存永远不刷新。

**解决**：改成直接返回 `Encoder_GetRightVelocity()`。

**教训**：**遥测取值路径不能依赖被测量自身所处的模式。** 观测手段必须独立于被测对象的状态。

---

### 5.5 ★ "执行器无法调速" —— 一次差点把项目带偏的重大误判

**现象（当时的判断依据）**：duty 从 200 变到 8000（40 倍），转速只从 252726 变到 264664 counts/s（4.5%）。看起来 TB6612 的 B 通道完全失控，执行器坏了。

**由这个结论推出的后续动作**（全部错误）：绕线改 PA5/IN2 的方案、`motor.h:24` 的烧毁注释推论、以及**所有基于这个前提做的参数调优**。

**它错在哪里**：265000 counts/s 对应 8880 RPM（电机轴）。而 JGB37-520 空载也就几百 RPM —— **物理上不可能**。这个读数本身是坏的。

**推翻它的证据**：闭环稳定巡航时的实测点：

```
ff       = 500 × 6000/(6000+150) = 488
KP·e     = 0.005 × 700           = 3.5      ← 可忽略
integral                         = -90
──────────────────────────────────────────
duty     ≈ 401                     实测 400 ✓
```

**自洽到 1 counts 以内。** duty 确实在驱动电机，电机确实按 duty 出转速。如果执行器真卡在满速，会看到 `vel ≈ 265000`、`integral` 钉死在 -800、`duty` 在 ±1200 之间暴力翻转 —— 完全不是这样。

**最终结论**：执行器正常。**两组数据差 44 倍，只能有一组对**，而当前这组（6000 counts/s = 201 RPM 电机轴 = 11.2 RPM 车轮）既自洽又符合常识。

**教训（本工程最重要的一条）**：

1. **先量被控对象，再调参。** 本项目连续多轮"对着不可信的模型和读数调参"，全部作废。
2. **用物理常识做量纲检查。** 8880 RPM 这个数字如果当时对照一下电机手册，整条弯路都不会走。
3. **别让用户从飞转的轮子上抄数字。** 让固件自己用纯位置差测速并冻结显示。
4. 后来用户明确要求"取消烧毁可能性，恢复 PB3，我用示波器测试" —— **在拿到直接测量之前，不要基于推论动手改动硬件方案。**

---

### 5.6 静摩擦：起转只需比巡航多 7% duty

**现象**：上电后轮子不转，要用手推一下才走。且是**间歇性**的 —— 三次不卡不代表问题消失。

**实测量**：

| 项 | duty |
|---|---|
| 静止起转（三次实测） | 426 / 435 / 449 |
| 稳态巡航 6000 counts/s | ≈ 400 |

**起转需求只比巡航高 7%。** 这个余量极窄，是所有现象的根：起转需求随齿轮啮合位置和轮胎压地状态漂移，一旦漂过 ~430 而前馈还没爬到，就得手推。

**已实现的机制**：`Breakaway_Update()` —— 检测"参考速度已在要求转（`|vel_ref|>600`）、实测速度仍≈0（`|vel|<300`）"持续 150ms，判定卡住，叠加一个从 1200 起按 3000/s 爬升的突破 duty（上限 3500），一旦真转起来立刻撤销。卡住期间**冻结积分**，防止起转后过冲。

**但它一次都没出手**（实测数据）：

| t | vel_ref | ff | integral | duty |
|---|---|---|---|---|
| 90 ms | 720 | 414 | 13 | ≈431（实测 426） |
| 100 ms | 800 | 421 | 16 | ≈441（实测 435） |
| 110 ms | 880 | 427 | 19 | ≈450（实测 449） |

`vel_ref` 在 t=75ms 越过 600 开始计时，要 t=225ms 才满 150ms；而轮子 t≈100ms 就自己走了，`|vel|>300` 把计时器清零。**所以这个机制在启动阶段完全没生效。**

**更好的下一步（未做）**：用固定 duty 逐档实测（250/300/350/400/430/460）量出确切的起转边界，再据此把 `ff` 改成物理正确的形状 —— 现在是 `500×vel_ref/(vel_ref+150)`，`vel_ref=0` 时为 0，即**"最需要它的时候恰好不给力"**，形状本身就不对。

> ⚠️ **duty→速度的静态增益 G 至今没有被可靠测出。** 早期估计的 `G≈45~100 counts/s per duty` 来自已被推翻的数据（见 5.5），不可引用。目前只有两个可信的工作点（duty 0 → 静止、duty 400 → 6000 counts/s），中间曲线未知。

---

### 5.7 USART3 波特率长期停在 9600：静默变慢且制造假波形

**现象**：无报错，但波形明显不对，且控制周期被抽稀。

**根因**：`usart.c` 里 USART3 走裸寄存器初始化、从不调用 `HAL_UART_Init()`，所以真正生效的只有 `USART3->BRR`。它长期写着 `0x28B1` = **9600 bps**（HC06 蓝牙还接在 USART3 的年代留下的）。

**为什么"静默"**：一帧 28 字节在 9600 下需要 29 ms。而 `HAL_UART_Transmit` 的超时给的是 100 ms —— **没超时，所以不报错**，只是把主循环拖成 ~29ms 一轮，10ms 的控制周期被抽稀成 ~34 Hz 采样。

**解决**：`USART3->BRR = 0x364`（100MHz / 115200，USARTDIV = 54.253）。

**修复前必须先确认的事**：HC06 是否还在 USART3。查 `hc06.c` 的 `HC06_UART = &huart2` 确认蓝牙早已移到 USART2，改波特率不会破坏蓝牙。

**教训**：**"没报错"不等于"没问题"。** 外设配置错误最危险的形态是静默降级 —— 它不产生错误码，只产生错误的数据，而错误的数据比没有数据更糟。

---

### 5.8 遥测采样率过低：把震荡混叠成假波形

**现象**：遥测波形呈锯齿状，看不出真实形状。

**根因**：主循环用 200 ms 发送遥测帧。而被观测的极限环周期是几十 ms —— **采样率低于信号频率，发生混叠**，拍出一个完全虚构的低频锯齿。

**解决**：改为与控制同节拍 10 ms 发送。

**教训**：**观测系统的带宽必须高于被观测对象的带宽。** 欠采样不会让信号"看起来模糊"，而是会**生成一个不存在的信号** —— 这比噪声危险得多，因为它看起来是有规律的。

---

### 5.9 OpenOCD `Error: open failed`：进程独占 USB，不是硬件坏

**现象**：连续 10 次烧录全部 `Error: open failed`，看起来像 ST-Link 挂了或 USB 接触不良。

**根因**：`stlinkserver.exe`（ST 的 ST-Link 共享访问守护进程）和 `cube.exe`（STM32CubeProgrammer）**独占着 ST-Link 的 USB 句柄**，OpenOCD 必然 open 失败。

**诊断**：
```bash
tasklist | grep -iE "stlinkserver|cube|openocd|gdb"
```

**判据**：
- 偶发抖动（重试 1~3 次能成功或自己恢复）= USB 接触问题；
- **连续 5 次以上全是 open failed = 被占用了**，重试再多次也没用。

关掉这两个进程后一次就烧成功。

**教训**：**重试次数本身就是诊断信息。** "偶发"和"必然"指向完全不同的原因，把它们混在一起会导致一直在解决错误的问题。另外，杀 `cube.exe` 前要让用户自己确认（里面可能有未保存的工作），不要擅自动别人的进程。

---

### 5.10 OLED 用硬件 I2C1：差点误删初始化

**经过**：清理 main.c 时，判断"I2C1 好像没人用"，准备删掉 `MX_I2C1_Init()`。删之前 grep 了一下 `oled.c`，发现 `oled.c:124` 是 `HAL_I2C_Master_Transmit(&hi2c1, ...)` —— **OLED 就挂在 I2C1 上**。

**教训**：**删除任何初始化代码前，必须 grep 对应的句柄变量**（这里是 `hi2c1`），而不是靠"我记得没人用"。外设句柄往往在别的模块里被引用，光看 main.c 看不出来。

---

### 5.11 电机模块的隐藏引用：函数体内 `extern`

**经过**：清理未使用模块时，`ultrasonic.c` 在 include 反向图上是标准孤岛（`ultrasonic.h` 只有它自己 include），看起来可以直接删。但它其实被引用了：

```c
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  extern void US_IRQ_Handler(void);   /* ← 就地声明，任何头文件反向图都看不见 */
  US_IRQ_Handler();                   /* → ultrasonic.c:119 */
```

**`US_IRQ_Handler` 声明在 HAL 头以外、直接写在中断函数里**，所以基于 `#include` 的反向依赖图完全查不到它。

**教训**：见下一节。

---

### 5.12 模块清理：include 图不足以判定死代码

`ultrasonic.c` 那次惊险之后，总结出**判死代码必须查三类引用**，include 反向图只是其中一类的下界：

| # | 引用方式 | 检查方法 | 本项目实例 |
|---|---|---|---|
| 1 | 正常 `#include` | include 反向图 | 大部分模块 |
| 2 | **函数体内的 `extern` 声明** | `grep -nE "^\s*extern"` 逐个可达文件 | `stm32h7xx_it.c` 的 `US_IRQ_Handler` |
| 3 | **HAL weak 回调的覆写** | grep `Callback\|_Handler` 于待删模块 | `hc06.c:140` 覆写 `HAL_UART_RxCpltCallback` |

第 3 类特别隐蔽：**HAL 的 weak 回调声明在 HAL 头里，不在模块自己的头里**，模块可以在不 include 任何自有头文件的情况下覆写它。

**本项目对第 3 类的实际裁决**：`hc06.c` 覆写了 `HAL_UART_RxCpltCallback`，但没有立刻下结论。继续查发现**全工程只有 `hc06.c` 自己调用 `HAL_UART_Receive_IT`**，即接收中断从未武装，回调永不触发 —— 这才确认删除是安全的。

**最终裁决权交给链接器**：删完 `.c` 后重新链接，`undefined reference` 是唯一权威证据。

**还有一个必须分清的区别**：

```
删除前 25 个模块（cargo/nav/hc06/mpu6050 等） →  FLASH 只减 4 字节
删除 ultrasonic.c                            →  FLASH 减 480 字节
```

因为 `-ffunction-sections --gc-sections` 早已把无人引用的函数丢弃了 —— **那 25 个模块的代码在删除之前就不在 Flash 里**。而 `ultrasonic.c` 被中断链激活，是真的占着空间。

**教训**：**"删了源码"和"省了空间"是两件事，不要混为一谈。** 报告优化效果时必须基于实测，而不是基于"我删了很多文件"的直觉。

---

### 5.13 云端仓库：本地缓存的远程 ref 是过期的

**经过**：准备推送时，本地 `origin/main` 指向 `f8a5c65`（只有一个 README 的 Initial commit），据此设计了"保留 README、把提交 rebase 到它上面"的方案。

**实际情况**：云端 `main` 早已是 `59c13559`，且有 8 个分支。**本地那个 `origin/main` 是几个月没更新的缓存。**

**如果照原方案执行，会把一个基于废弃提交的分支推上去。**

**教训**：**动手前用 `git ls-remote origin` 看真实状态，不要相信本地缓存的远程 ref。**

---

## 6. 验证方法论

从上面的坑里提炼出的可复用方法：

### 6.1 选择两种假说预测相反的工况

诊断要挑**两种竞争假说会给出相反预测**的工况，一次就能分开。

实例：判断"执行器坏了"还是"测量坏了"，滑行段（duty=0）就是这样的工况 —— 若是传感器噪声，读数应≈0；若是真惯性，读数仍应很大。

### 6.2 量纲检查优先于数据分析

8880 RPM 这个数字，对照一下电机手册就该被立刻否掉。**在深入分析之前，先问"这个数物理上可能吗"。**

### 6.3 让固件自己测量，不要让用户抄数字

从飞转的轮子上抄数字既危险又不可靠。让固件用纯位置差测速并把结果冻结显示在 OLED 上。

### 6.4 链接器是死代码的最终裁决者

静态分析（include 图、符号 grep）能给出候选集，但**只有链接能证明**。删完重新链接，`undefined reference` 是唯一权威证据。

### 6.5 干净重建才算验证

增量构建可能被残留的 `.obj` 干扰。声称"验证通过"时应当用 `--clean-first` 从零重建，并比对内存区域占用。

### 6.6 报告要区分"实测"和"推断"

本文档里所有数字都标注了来源。**建立在已被推翻数据之上的推断（如 `G≈45~100`），必须明确标记为不可引用**，否则会被后来的人当成事实继续使用。

---

## 7. 当前状态与已知问题

### 7.1 内存占用（V1.3，干净重建实测）

```
DTCMRAM:   4424 B / 128 KB   3.38%
FLASH:    60044 B /   2 MB   2.86%
```

### 7.2 控制参数

```c
#define MC_VMAX         6000.0f   /* 巡航速度上限 counts/s, 一圈 ≈5.4s */
#define MC_KP_POS       2.0f      /* 位置外环 P */
#define MC_AMAX         8000.0f   /* 速度参考斜率限制 counts/s² */
#define MC_DECEL        9000.0f   /* 减速规划 */
#define MC_KP_VEL       0.005f    /* 速度环 P (只提供阻尼) */
#define MC_KI_VEL       0.40f     /* 速度环 I (主积分作用) */
#define MC_FF_STIC      500.0f    /* 静摩擦前馈 */
#define MC_FF_SMOOTH    150.0f    /* 前馈平滑尺度 */
#define MC_INT_MAX      800.0f    /* 积分限幅 */
#define MC_MAX_DUTY     4000      /* duty 限幅 */
#define MC_ARRIVE_BAND  80        /* 到位死区 counts */
#define MC_EXIT_BAND    250       /* 到位滞回 counts */

/* 启动突破 */
#define MC_KICK_CMD_VEL   600.0f
#define MC_KICK_DONE_VEL  300.0f
#define MC_KICK_MS        150
#define MC_KICK_DUTY_MIN  1200.0f
#define MC_KICK_DUTY_MAX  3500.0f
#define MC_KICK_RAMP      3000.0f
```

> ⚠️ **这些参数是在"执行器能调速"的前提下调的**，该前提**后来被证实成立**（见 5.5），但调参过程中使用过的部分数据是不可信的。**`MC_KP_VEL = 0.005` 极小**（速度环几乎没有 P 阻尼，全靠积分），这是为配合测速滞后而刻意压低的环路增益，改动时需重新评估相位裕度。

### 7.3 活代码与死代码

**右电机闭环路径（活的）**：

```
SysTick_Handler → MotorControl_Tick → MotorControl_Update
                                      ├─ Encoder_Update      (motor.c)
                                      └─ Motor_Right_Drive   (motor.c)
                                   → Motor_Right_Brake      (到位时)
```

**已确认无调用点的代码**（`--gc-sections` 已将其从 Flash 中移除）：

| 符号 | 位置 | 说明 |
|---|---|---|
| `Motor_UpdateRPM` 及 `Motor_Get*RPM` / `Motor_Get*SpeedMS` | `motor.c` | 100ms 窗口 RPM 测量，已被 `Encoder_Update` 的 10ms 测速取代 |
| `Motor_SetSpeedFromADC` / `Motor_SetSpeedFromVoltage` | `motor.c` | 电位器调速；两者只互相调用 |
| `Move_Forward` / `Move_Reverse` / `Move_Stop` | `motor.c` | 只被上面那个函数调用 |
| `Motor_Left_*` 全套 | `motor.c` | **左电机路径整体未使用**（含 `Motor_Left_Drive`，它只在 `motor_control.h:7` 的注释里出现过） |
| `Encoder_GetLeftPosition` / `Encoder_GetLeftVelocity` | `motor.c` | 左编码器仍在累积（`Encoder_Update` 照常计算），但无人读取 |
| `MotorControl_SetEnabled` | `motor_control.c` | 开环辨识用，辨识代码已删 |
| `MotorControl_IsArrived` | `motor_control.c` | 无人调用 |

> 注意：`Encoder_Update()` 仍然每 10ms 计算左电机的 `left_pos32` 和 `left_vel`，**这是运行时开销但结果无人使用**。若确定不用左电机，可一并裁掉。

**外设层的悬空初始化**（建议在 CubeMX 里关闭，不要手改 `.c`）：

| 项 | 位置 | 状况 |
|---|---|---|
| `MX_I2C2_Init()` | `main.c` | `hi2c2` 唯一使用者是已删除的 MPU6050 |
| `MX_ADC3_Init()` | `main.c` | `ADC3_ReadChannel/Voltage` 零调用 |
| `MX_TIM4_Init()` | `tim.c:189` | 零调用 |
| `TIM5_IRQHandler` | `stm32h7xx_it.c` | 空壳，调用已摘除；TIM5 仍在 `.ioc` 的 NVIC 里 |

⚠️ **`H7_Arm.ioc` 需要同步**，否则下次 CubeMX 重新生成会把 I2C2 / ADC3 / TIM4 / TIM5 全部恢复。

### 7.4 文档一致性问题

- `motor_control.h:3` 写"**左**电机"，`motor_control.c:3` 写"**右**电机" —— 实现是右电机，头文件注释过时。
- 两处 EMA 时间常数注释不准确（见 [3.5](#35-控制周期与测速)）。

### 7.5 待办

- [ ] 用固定 duty 逐档实测（250/300/350/400/430/460）量出确切的起转边界
- [ ] 据此重新设计 `ff` 的形状 —— 当前 `x/(|x|+150)` 在低速段偏弱，正是最需要它的区间
- [ ] 决定启动突破机制的去留：要么重新调参让它真正生效（`MC_KICK_MS 150→40`、`MC_KICK_DUTY_MIN 1200→400`），要么删掉
- [ ] 清理 [7.3](#73-活代码与死代码) 中的死代码
- [ ] 在 CubeMX 中关闭 I2C2 / ADC3 / TIM4 / TIM5 并重新生成

---

## 8. 构建与烧录

### 8.1 工具链

- `arm-none-eabi-gcc`
- CMake ≥ 3.22
- MinGW Makefiles 生成器
- OpenOCD 0.12.0

### 8.2 构建

```bash
cmake -B build -G "MinGW Makefiles"
cmake --build build
# 产物: build/H7_Arm.elf
```

从零重建（验证用）：

```bash
cmake --build build --clean-first
```

### 8.3 烧录

```bash
openocd.exe -s "<openocd>/scripts" \
  -f interface/stlink.cfg \
  -f target/stm32h7x.cfg \
  -f tools/flash.cfg
```

### 8.4 常见故障

| 现象 | 原因 | 处理 |
|---|---|---|
| `Error: open failed`（连续 5 次以上） | `stlinkserver.exe` / `cube.exe` 独占 ST-Link | 关掉这些进程 |
| `Error: Debug Adapter has to be specified` | 漏了 `-f interface/stlink.cfg -f target/stm32h7x.cfg` | 补上 |
| 偶发 open failed | USB 接触问题 | 重插 |

### 8.5 调试遥测

USART3 (PD8) 输出 VOFA+ JustFloat 帧，115200 8N1。VOFA 里协议选 `JustFloat`、通道数 = 6。

---

## 9. 版本与仓库

### 9.1 云端

`https://github.com/xiezheng-kingdom/XZ_RCTEST.git`

**云端按"一个版本一个分支"管理，不使用 tag**：

| 分支 | 内容 |
|---|---|
| `main` | 主分支 |
| `RCTEST_PID_MOTOR_V1.2` | V1.2 |
| `RCTEST_PID_MOTOR_V1.3` | V1.3（当前） |
| `RCTEST_CAN` / `RCTEST_VOFA` / `RCTEST_F103C8T6` | 其他实验 |
| `RCTEST_YUNTAI_CONTROL` / `RCTEST_YUNTAI_MOVE` | 云台控制 |

推送新版本分支：

```bash
git push origin main:refs/heads/RCTEST_PID_MOTOR_V1.X
```

### 9.2 本地 tag 与云端分支是两套东西

本地存在轻量 tag `RCTEST_PID_MOTOR_V1.2` / `RCTEST_PID_MOTOR_V1.3`，**它们从未推送到云端**。云端同名的 `RCTEST_PID_MOTOR_V1.2` 是**分支**，哈希与本地 tag 指向的提交没有任何关系。讨论"V1.2"时必须说清是哪一套。

### 9.3 V1.3 的变更

- 删除未使用的搬运车业务链模块共 27 个文件（`Core` 从 52 个文件降到 25 个）
- 含 `cargo` / `nav` / `road_map` / `kalman` / `pid` / `ir_sensor` / `mpu6050` / `hc06` / `obstacle` / `openmv` / `localization` / `error_log` / `ultrasonic` 及 `config.h`
- 这些模块的代码完整保留在提交 `e34052b`（V1.2），需要时可用 `git checkout e34052b -- <路径>` 取回
- 清理了 `CMakeLists.txt`、`stm32h7xx_it.c`、`i2c.h` 及若干失效注释

---

## 附：设计原则速查

1. **先量被控对象，再调参。** 对着不可信的模型调参，全部作废。
2. **不要用未知参数做前馈。** 能靠积分器自适应的事，不要靠猜。
3. **位置外环的输出是阶跃，必须规划。** 算出来的速度参考一定过猛。
4. **换向处必查连续性。** 任何符号函数形式的前馈都会制造阶跃。
5. **观测系统的带宽必须高于被观测对象。** 欠采样会生成不存在的信号。
6. **"没报错"不等于"没问题"。** 静默降级是最危险的故障形态。
7. **删初始化前先 grep 句柄。** 外设句柄常在别的模块里被引用。
8. **判死代码不能只信 include 图。** 函数体内 `extern` 和 HAL weak 回调都会漏，最终裁决交给链接器。
9. **"删了源码"不等于"省了空间"。** 有 `--gc-sections` 在，报告优化必须基于实测。
10. **动手前 `git ls-remote`。** 本地缓存的远程 ref 可能过期很久。
