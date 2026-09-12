# 二自由度云台控制板 · 开发文档

> 目标板：STM32H743（`XZ_RCTEST` 硬件）
> 分支：`RCTEST_YUNTAI_CONTROL`
> 本文记录架构怎么设计的、踩了哪些坑、怎么解决的。调试过程里的每一个结论都注明是**实测**还是**推导** —— 这个区分在本项目里非常重要（见 §5.5）。

---

## 1. 项目背景与目标

这块板子原本属于一个智能车/机械臂工程（`XZ_RCTEST`），`Core/` 里塞着 cargo、nav、pid、kalman、motor、oled 等 14 个与本任务无关的模块。本文件夹的定位是**二自由度云台控制板固件**。

要做的功能：

| 项 | 内容 |
|---|---|
| 输入 | 2 个电位器 **或** 1 个 MPU6050，二选一 |
| 切换 | PE3 按键（按下接地），指示灯 PC5 同步亮灭 |
| 输出 | 目标角度文本 `偏航,俯仰` → USART3 |
| 约定 | 正 = 右转（偏航）/ 向下俯冲（俯仰），两轴限幅 ±30° |

**本步不输出舵机 PWM**，只算角度并上报。PWM 留到下一步（TIM4 已配好，见 §4.4）。

---

## 2. 硬件资源

| 资源 | 配置 | 备注 |
|---|---|---|
| 电位器 1 | PC2 = ADC3_INP0（rank1） | 12 位，0~4095 |
| 电位器 2 | PC3 = ADC3_INP1（rank2） | 与 rank1 同一次扫描 |
| MPU6050 | I2C2，PB10=SCL / PB11=SDA，地址 0xD0 | 100 kHz |
| 按键 | **PE3**，按下接地 | 全 Core 零占用，内部上拉 |
| 指示灯 | **PC5**，高电平点亮 | 全 Core 零占用 |
| 上位机 | USART3，PD8=TX / PD9=RX，115200 | `printf` 重定向到此处 |
| 蓝牙 HC06 | USART2，PD5=TX / PD6=RX，115200 | 本步不用（见 §4.5） |
| 舵机 PWM | TIM4，PD14=CH3 / PD15=CH4，50Hz | 本步不调（见 §4.4） |
| 系统时钟 | HSE → PLL1 → SYSCLK 400MHz，HCLK 200MHz | APB1 = 100MHz |

PE3 / PC5 是**从 `Core/` 里挑出来的空闲引脚** —— 全工程对这两个引脚零占用，所以不需要动硬件。

---

## 3. 软件架构

### 3.1 分层

```
              ┌──────────────────────────────────────┐
              │  main.c        初始化 + 主循环节拍    │
              └───────────────┬──────────────────────┘
                              │ 每 10ms / 每 50ms
              ┌───────────────▼──────────────────────┐
              │  gimbal.c      输入源 → 目标角度      │  ← 本任务新增
              │    key_scan()  按键消抖 + 切换        │
              │    pot_update()  电位器支路           │
              │    mpu_update()  MPU6050 支路         │
              └───────┬───────────────┬──────────────┘
                      │               │
        ┌─────────────▼───┐   ┌───────▼──────────┐
        │  adc.c          │   │  mpu6050.c       │   ← 原工程已有，未改
        │  ADC3_ReadChannel│   │  DataUpdate()    │
        └─────────────────┘   └───────┬──────────┘
                                      │ I2C2
                              ┌───────▼──────────┐
                              │  i2c.c           │   ← 删掉了 I2C1(OLED)
                              └──────────────────┘

              gpio.c   PE3 上拉输入 / PC5 推挽输出
              usart.c  printf → USART3
```

### 3.2 目录与模块

精简后 `Core/` 只剩：

```
Core/Src/   adc.c  gimbal.c  gpio.c  hc06.c  i2c.c  main.c  mpu6050.c
            stm32h7xx_hal_msp.c  stm32h7xx_it.c  syscalls.c  sysmem.c
            system_stm32h7xx.c  tim.c  usart.c
Core/Inc/   adc.h  gimbal.h  gpio.h  hc06.h  i2c.h  main.h  mpu6050.h
            stm32h7xx_hal_conf.h  stm32h7xx_it.h  tim.h  usart.h
```

删掉的 14 个模块（`.c`/`.h` 成对删除）：
`cargo` `error_log` `ir_sensor` `kalman` `localization` `motor` `motor_control` `nav` `obstacle` `oled` `openmv` `pid` `road_map` `ultrasonic`

外加 `Core/Inc/config.h` —— 它里面是全局变量的**定义**（不是 `extern`），且全工程零 `#include`，属死文件。

规模：`408 insertions(+), 5574 deletions(-)`。

### 3.3 数据流

```
   PE3 ──► key_scan() ──► s_src (0=电位器 / 1=MPU) ──► led_apply() ──► PC5
                                │
        ┌───────────────────────┴────────────────────────┐
        ▼ s_src==0                                      ▼ s_src==1
   pot_update()                                    mpu_update()
   ADC3 → 0~4095                                   accel → pitch
   (adc-2048)*60/4095                              gyro Z 积分 → yaw
        │                                                │
        └────────────────┬───────────────────────────────┘
                         ▼
                  s_yaw / s_pitch  (clamp ±30°)
                         │  每 50ms
                         ▼
              printf("%.1f,%.1f\r\n") ──► USART3
```

### 3.4 为什么控制节拍放在主循环，不放 SysTick

**这是本工程最关键的一条架构约束。**

`SysTick_Handler` 是 1ms 中断，里面**不能**做这三件事：

| 操作 | 耗时 | 后果 |
|---|---|---|
| MPU6050 一次 14 字节 I2C 读（100kHz） | ≈ **1.5ms** | 直接超掉 1ms 的 tick |
| `ADC3_ReadChannel()`（阻塞轮询两次转换） | 几十 µs | 抖动 |
| `printf` 阻塞式输出 9 字节 | ≈ 0.8ms | 抖动 |

一旦中断里超时，`HAL_IncTick()` 会被饿死，**所有 `HAL_Delay()` 和 HAL 超时全部走样** —— 这类故障极难查。

所以 `SysTick_Handler` 被恢复成**纯 `HAL_IncTick()`**（原工程在这里挂了 `MotorControl_Tick()`，随 motor 模块一起删掉），节拍改由主循环用 `HAL_GetTick()` 门控：

```c
while (1) {
  if (HAL_GetTick() - last_tick >= GIMBAL_TICK_MS) {     /* 10ms */
    last_tick = HAL_GetTick();
    Gimbal_Update();
    if (HAL_GetTick() - last_send >= GIMBAL_SEND_MS) {   /* 50ms */
      last_send = HAL_GetTick();
      Gimbal_Report();
    }
  }
}
```

单拍预算：I2C 1.5ms + ADC ~0.1ms + 每 50ms 一次 printf 0.8ms ≈ **最坏 2.3ms / 10ms**，余量充足。

### 3.5 输入源抽象

两个输入源在 `gimbal.c` 内部收敛到同一组状态变量 `s_yaw` / `s_pitch`，对外只有：

```c
void            Gimbal_Update(void);        /* 每 10ms */
void            Gimbal_Report(void);        /* 每 50ms */
Gimbal_Source   Gimbal_GetSource(void);
void            Gimbal_GetTarget(float *yaw, float *pitch);
```

好处是**下一步接舵机时完全不用碰输入侧** —— 舵机模块只需要 `Gimbal_GetTarget()`。

### 3.6 角度换算

**电位器**（12 位，中点 2048 = 0°）：

```c
((float)adc - 2048.0f) * (2.0f * 30.0f) / 4095.0f
```

**MPU6050 俯仰**（加速度计测重力方向）：

```c
pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD2DEG;   /* 正 = 下俯 */
```

用 `sqrt(ay²+az²)` 而不是直接用 `az`，是为了在大横滚时仍然成立。

**MPU6050 偏航**（陀螺仪 Z 轴积分）：

```c
gz = MPU6050_GetGz() * GIMBAL_GYRO_SIGN;
if (fabsf(gz) < GIMBAL_GYRO_DEADBAND) gz = 0.0f;        /* 死区 */
s_yaw = clamp_angle(s_yaw + gz * (GIMBAL_TICK_MS / 1000.0f));
```

切换进 MPU 模式时把 `s_yaw` 清零 —— 陀螺仪积分是**相对角**，不重置会把切换前累积的漂移一起带进来。

---

## 4. 关键设计决策

### 4.1 删除而不是注释

14 个模块全部**成对删除**（`.c` + `.h`），不保留"以后可能用得上"的注释代码。理由：留着就会有人 include，链接期才发现依赖已经断了。

删完必须一并修掉的编译阻塞点：

| 文件 | 内容 | 处理 |
|---|---|---|
| `main.c` | include oled/motor/motor_control | 删 |
| `main.c` | OLED/Motor/MotorControl 初始化调用 | 删 |
| `main.c` | 10ms VOFA 二进制调试帧（6 个 float 全来自 motor_control） | **整块重写** |
| `stm32h7xx_it.c` | `#include "motor_control.h"` | 删 |
| `stm32h7xx_it.c` | `SysTick` 里的 `MotorControl_Tick()` | 删 |
| `stm32h7xx_it.c` | `TIM5_IRQHandler` + `US_IRQ_Handler()` | 删（TIM5 在 tim.c 里根本没初始化，孤儿） |
| `hc06.c` | `#include "cargo.h"` 与 `HC06_Process()` 里的 `Cargo_FeedByte()` | 改成纯排空环形缓冲 |

### 4.2 I2C1 整体移除

OLED 删了，`hi2c1` / `MX_I2C1_Init` / PB8/PB9 的 Msp 就没有使用者了，**整块删除**。I2C2 与 `MPU6050_I2C_ADDR` 原样保留。

### 4.3 `hc06.c` 只删依赖，不删实现

`HC06_Process()` 改成只排空环形缓冲（丢弃字节），避免开了 `HC06_Init` 之后缓冲被写满。其余（环形缓冲、AT 指令、`HAL_UART_RxCpltCallback`）**全部原样保留** —— 这是全工程唯一一处 `HAL_UART_RxCpltCallback` 定义，删了会破坏后续扩展。

### 4.4 保留 TIM4（虽然本步用不到）

严格按"只保留 ADC/MPU6050/HC06/USART"的字面意思，`tim.c` 也该删干净。**决定精简保留 TIM4**：

- 同一个句子里说了"最终目标是控制二自由度云台"，舵机 PWM 是必经之路
- `H7_Arm.ioc` 里 TIM4 本来就是 50Hz / PD14 / PD15 的两路配置，等于舵机口已经画好了
- 删掉的话下一步要照着 `.ioc` 重新生成一遍

参数特意设成 **Prescaler=199 / Period=19999**：200MHz 下分频后是 **1MHz 计数，`Pulse` 数值直接等于微秒**，正是舵机要的（1500µs 中心 ±500µs 对应 ±30°）。

代价是多约 30 行当前未调用的初始化和一个 `htim4` 全局变量，可接受。

### 4.5 HC06 默认关闭

`HC06_Init()` 内含 **1 秒阻塞延时**。本步输出走上位机 USART3、不消费蓝牙数据，所以用 `GIMBAL_HC06_ENABLE` 开关默认关掉。

### 4.6 符号全部做成宏

```c
#define GIMBAL_POT_YAW_SIGN    (+1.0f)
#define GIMBAL_POT_PITCH_SIGN  (+1.0f)
#define GIMBAL_MPU_PITCH_SIGN  (-1.0f)
#define GIMBAL_GYRO_SIGN       (-1.0f)
```

板子上装传感器时谁也没法保证朝向，**极性必须靠台架实测**。把四个符号全部提成宏，标定一次改一行，不用翻代码找 `-` 号。这是本工程最省事的一个决定。

---

## 5. 遇到的问题与解决

> 本节是本文档的重点。每个问题按 **现象 → 排查 → 根因 → 解决** 记录。

### 5.1 编译期

#### 问题 1：Keil 工程在改动之前就是坏的

**现象**：`MDK-ARM/H7_Arm.uvprojx` 列了**不存在的** `../Core/Src/servo.c`，同时**漏了** `adc.c`、`hc06.c`、`mpu6050.c`，连 HAL 驱动 `stm32h7xx_hal_adc.c` / `stm32h7xx_hal_adc_ex.c` 都没列。

**根因**：不是本次改动引入的，是原工程遗留。也就是说 Keil 这条路径**当前就链接不过**（`HAL_ADC_*` 全部未定义）。

**解决**：顺手一起修掉 —— 删 `openmv.c` 与 `servo.c` 两个 File 块，补上 `adc.c` / `hc06.c` / `mpu6050.c` / `gimbal.c`，`uvoptx` 同步。

**教训**：接手别人的工程，**先确认两条构建路径各自是否本来就是绿的**，别把原有故障当成自己改坏的。

#### 问题 2：删除模块后 CMake 的源文件清单

`CMakeLists.txt` 的 `target_sources` 里删掉 14 个已删模块，新增 `gimbal.c`。`cmake/stm32cubemx/CMakeLists.txt` **不用改** —— 它只列 main/gpio/adc/i2c/tim/usart/it/msp/sysmem/syscalls/startup，全是保留文件。

### 5.2 工具链

#### 问题 3：本地没有 Ninja / make，CMake 跑不起来

**现象**：`cmake --preset Debug` 直接失败。

**根因**：`arm-none-eabi-gcc 14.2.1` 和 `cmake 4.3.1` 都在，但生成器（Ninja 或 make）都没装。

**解决**：写了 `build/build.sh` —— 从 `cmake/stm32cubemx/CMakeLists.txt` 里 `sed` 抽取出 HAL 源文件清单，直接调 `arm-none-eabi-gcc` 编译链接。等价于完整构建，且用 `Core/Src/*.c` 通配可以**顺带证明没有残留对已删模块的引用**（少了哪个符号链接期就会报 undefined reference）。

> ⚠️ 该脚本放在 `build/` 下，而 `build/` 在 `.gitignore` 里，**没有进版本库**。换机器要重建。

#### 问题 4：`multiple definition of SystemCoreClock / SystemInit / ...`

**现象**：`build/build.sh` 第一次跑就报一堆重复定义。

**根因**：`sed` 抽出的 HAL 列表**包含了** `Core/Src/system_stm32h7xx.c`，而 `Core/Src/*.c` 通配也匹配到了它 —— 同一个文件编了两遍。

**解决**：在抽取管道后面加 `| grep -v 'Core/Src/system_stm32h7xx.c'`。

**教训**：用通配符 + 清单两路收集源文件时，**必须显式去重**，别指望两者天然不重叠。

### 5.3 调试通道

#### 问题 5：ST-Link 卡死，且软件层救不回来

**现象**：Windows 里设备照常枚举（`USB\VID_0483&PID_3748`，Status OK），`claim_interface(0)` 也成功，但 `pyocd list` 报 **"No available debug probes are connected"**。

**排查过程**（含两个我自己的错误判断，一并记下）：

1. 一度以为 `pyusb` 没装 —— **错**，pyusb 1.3.1 已安装（导入名是 `usb`）。
2. 一次 `usb.core.find()` 报 `NoBackendError`，是我自己没给后端 —— 改用 `libusb_package` 自带的 DLL 后正常。
3. 给 ST-Link 发控制传输 `0xC0/0xF1` → EPIPE。**这也是我判断错了**：pyocd 用的是**批量端点** 0x02(OUT) / 0x81(IN)，不是控制传输。改用批量传输重测，结果相同（EPIPE 然后超时）—— **这才确认固件确实卡死**。

**根因**：`pyocd/probe/stlink/usb.py:143` 读 `self._dev.serial_number` 抛 `ValueError: The device has no langid`，而 `usb.py:113` 的 `except (ValueError, usb.core.USBError, IndexError, NotImplementedError): pass` **静默丢弃**了该设备。所以报"没有探针"是假象，**探针其实在**。

**已验证无效的软件手段**：
- `libusb` 的 `dev.reset()` —— WinUSB 上是空操作
- `clear_halt()` —— 能清掉 STALL，但固件仍不应答
- PnP 断电重来 —— 需要管理员权限，这台机器上没有

**解决**：**拔掉 USB 线等 10 秒再插回**，然后重跑烧录命令。

**教训**：`pyocd list` 说不认识探针 ≠ USB/驱动有问题。看到 `No available debug probes` 而设备管理器正常，第一反应应该是**拔插**。

#### 问题 6：串口打不开（`PermissionError(13)`）

**现象**：`serial.Serial('COM6')` 报拒绝访问。

**根因**：**不是故障** —— VOFA+ 正开着，独占该端口。

**附带发现**：**COM 口号不固定**，这块板先是 COM5，拔插一次后变成 COM6。每次都重新查，别写死。

#### 问题 7：VOFA+ 协议要选对

原工程 `main.c` 用的是 **JustFloat 二进制帧（6 个 float）**，所以 VOFA+ 很可能还停在 JustFloat 模式。新固件的输出是 **ASCII 文本**（`12.5,-7.0`），对应 **FireWater** 模式。选错模式会看到乱码。

### 5.4 传感器与信号链

#### 问题 8：`road_map.h` 断言 MPU6050"芯片损坏"，结论过强

**现象**：`road_map.h:35` 记着「MPU6050：航向角参考（**芯片损坏**, 暂不可用）」，架构图里标注「(备用)」。

**排查**：实测发现 —— 烧录后第一次读 `mpu6050_ok` → **0**；拔插 USB 重新上电后再读 → **1**，且数据正常（静止时 `a_x≈a_y≈0, a_z≈1.00`，合矢量 1g）。

**根因**：**接触不良 / 偶发**，不是彻底损坏。`MPU6050_Init()` 只在启动时读一次 WHO_AM_I，读失败就整个会话都用不了。

**解决**：代码已按"传感器可能不可用"写 —— MPU 分支在 `MPU6050_IsOK()==0` 时输出 `0,0` 并保持，不阻塞不崩。同时把 `road_map.h` 的结论降级记录。

**教训**：判断这类芯片好坏**必须重新上电后再看**，不能凭一次上电的结果下结论。

#### 问题 9：两次把正常的 ADC 读数误判成故障

这是本项目里我犯得最贵的两个错，都记下来。

**错误判断 A**：「`SamplingTime = 8.5` 周期太短，导致 ADC 读数为 0」
**错误判断 B**：「ADC 读数为 0 被钳到 -30 是故障」

**实际情况**：**两个都错**。

- 对相同寄存器设置做**直接寄存器驱动的扫描读数**，结果完全一致（4095）—— 采样时间没有任何问题。
- 那个 `-30` 是**电位器被拧到最小位置**了，`DR` 到角度的映射精确正确。

**真正的诊断方法**（后来固化下来）：用 SWD 直接读 ADC3 的 `DR`（`0x58026040`），与 `s_yaw`/`s_pitch` 对账，换算关系应为 `(DR-2048)*60/4095`。

**教训**：**"贴限幅的读数"不一定是故障，也可能就是输入拧到底了。** 在断定信号链坏了之前，先直接读一手的原始寄存器。

#### 问题 10：我误读了 SQR1，一度以为扫描 rank 配错

**现象**：`SQR1 = 0x1001`，我按 `SQ2` 在 bit 11 解码，得出 SQ2=2，怀疑 rank2 选错了通道。

**根因**：**我算错了**。CMSIS 头文件里 `ADC_SQR1_SQ2_Pos = 12U`（**不是 11**）。按 bit 12 解码得 SQ2 = 1 = channel 1 = PC3 —— **代码本来就是对的**。

**教训**：解码外设寄存器**直接查 CMSIS 头文件的 `*_Pos` 宏**，不要凭记忆里的位域布局。这类错误会让人去"修"一段本来就正确的代码。

#### 问题 11：`HAL_ADC_Stop()` 不清 `REG_EOC`，差点误判转换失败

调试时用 SWD 直接驱动 ADC，需要判断一次转换是否成功。读 `hadc3.State`：

```c
HAL_ADC_Stop() → ADC_STATE_CLR_SET(State, REG_BUSY|INJ_BUSY, READY)
```

它**不会**清 `REG_EOC`。所以 `State = 0x201`（`READY | REG_EOC`）表示一次**成功**的转换，不是残留错误。

### 5.5 符号标定（本项目最重要的方法论）

**核心原则：极性靠台架实测，不靠推导。** 本项目里两次都推错了。

#### 问题 12：俯仰符号 —— 推导与实测相反

**推导**：约定传感器坐标 X=机头、Y=右、Z=上，则机头下俯 θ 时 `a_x = -sinθ`（负），所以 `pitch = atan2f(-ax, ...)` 给出的正是"下俯为正"，`GIMBAL_MPU_PITCH_SIGN` 应为 `+1.0f`。

**实测**：机头下压（俯冲）**47.7°** 时读到

```
a_x = +0.724    a_y ≈ 0    a_z = +0.659     合矢量 0.979 g
```

`a_x` 为**正**，与推导相反。

**根因**：这块板的 MPU6050 **X 轴指向机尾**（模块绕 Z 轴装了 180°）。按 `a_x = -a_x_nose` 反推，三个分量 `+0.724 / 0 / +0.659` 与 `+sinθ / 0 / cosθ` 完全吻合，交叉验证成立。

**解决**：`GIMBAL_MPU_PITCH_SIGN` → `-1.0f`。

**板上验证**：9.2 秒保持俯冲，`a_x` 稳定在 +0.70~+0.74（对应真实俯仰 -42°~-46°），固件报 `s_pitch = +30.00` —— **正数，符合约定**（因为超过 30° 所以饱和在限幅上）。✅

#### 问题 13：偏航符号 —— 我写在注释里的推导是错的

**我的推导**：模块绕 Z 轴装 180° 不动 Z 轴，静止时 `a_z` 仍为 `+1.00g`，所以陀螺仪 Z 轴符号**不受影响**，`GIMBAL_GYRO_SIGN` 保持 `+1.0f`。

**实测反馈**：**机头右转时偏航角读数为负** —— 符号是反的。

**解决**：`GIMBAL_GYRO_SIGN` → `-1.0f`。

**同时必须删掉那段注释**。它把"推导"当成了"结论"写进代码，而实测与之相反 —— 留着会误导下一次标定的人（很可能就是未来的自己）。已改成记录实测结论。

**⚠️ 验证状态**：板级验证**尚未完成**。两次抓取（25s + 40s）里 `g_z` 都平在 0、`s_yaw` 一动不动，说明云台全程没被转动。符号翻转本身依据的是用户的实际观察，但"右转 = 正"这一条**没有在板上复现过**。

**教训（本文档最该记住的一条）**：

> **注释里写"推导"还是写"实测"，必须分清楚。** 本项目两次都是推导错、实测对。凡是涉及物理安装朝向的量（极性、零点、轴向），一律按实测结论写，并且注明测量条件和数据。

### 5.6 已知未修的信号问题

#### 问题 14：陀螺仪噪声远高于死区，偏航会随机游走顶死

**现象**：对比两次 SWD 抓取的 `g_z`：

| 云台状态 | `g_z` 表现 |
|---|---|
| 放在桌上 | 平在 0，±0.06°/s（陀螺仪 1 个 LSB） |
| **拿在手里** | 在 **±3~4°/s** 之间跳 |

**根因**：`GIMBAL_GYRO_DEADBAND` 只有 **0.5°/s**，手持噪声整个从**死区上方**过去，被一路积分。后果是**手持时偏航角会在几十秒内随机游走到 ±30 顶死**。

这解释了第一次抓取里 `s_yaw` 一路贴着 `-30.00`：那 10 秒的 `g_z` 均值约 -4°/s，8 秒就撞限幅。

**影响**：标定和实际使用中，偏航输出除了符号之外还叠了这层随机游走，非常干扰。

**建议的修法**（尚未实施）：积分前给 `gz` 加一阶低通（EMA，时间常数 0.3~0.5s），并把死区抬到 ~1.5°/s。

估算依据：τ=0.5s、100Hz 采样下，EMA 对白噪声的标准差压缩比约 `sqrt(α/(2-α))`，其中 `α = 1-exp(-1/(τ·fs)) ≈ 0.02`，得约 0.1 倍 —— ±4°/s 的噪声被压到约 **±0.4°/s**，落进 1.5°/s 死区就是干净的 0，静止不漂；真实转动（>1.5°/s）照样过得去。

**未实施的原因**：当前正在逐个标定符号，加滤波器会改变动力学特性，干扰标定判断。**先标定完符号，再上滤波。**

---

## 6. 构建与烧录

### 6.1 构建

机器上没有 Ninja/make，用脚本直接调 gcc：

```bash
sh build/build.sh
```

产物 `build/gimbal.elf` + `build/gimbal.hex`。当前规模：

```
text 56988   data 476   bss 2612
```

要求：**零 warning、零 undefined symbol**。

### 6.2 烧录

```bash
pyocd flash -t stm32h743xx -f 100k -O connect_mode=under-reset build/gimbal.elf
```

降速 + 复位下连接，是从 `tools/flash_dump.cfg` 的 `connect_assert_srst` + `adapter speed 100` 抄来的配方。

**若报"No available debug probes"** → 见 §5.3 问题 5，拔插 USB。

### 6.3 构建文件改动清单

| 文件 | 改动 |
|---|---|
| 根 `CMakeLists.txt` | `target_sources` 删 14 个已删模块，新增 `gimbal.c` |
| `cmake/stm32cubemx/CMakeLists.txt` | 不改 |
| `MDK-ARM/H7_Arm.uvprojx` | 删 `openmv.c` / `servo.c`，补 `adc.c` `hc06.c` `mpu6050.c` `gimbal.c` |
| `MDK-ARM/H7_Arm.uvoptx` | 同步同一份清单 |
| `H7_Arm.ioc` | 本步不改。**注意**：`.ioc` 里没有 PE3/PC5，一旦用 CubeMX 重新生成，`gpio.c` 里手加的两个引脚会被抹掉 |

---

## 7. 标定与验证方法

### 7.1 SWD 作为无串口验证通道

串口经常被 VOFA+ 占着，且 COM 口号会变。**SWD 是一条独立的、不需要任何外设配合的验证通道**：

```python
from pyocd.core.helpers import ConnectHelper
s = ConnectHelper.session_with_chosen_probe(
        target_override='stm32h743xx', connect_mode='attach')   # attach = 不打断运行
with s:
    t = s.target
    if 'RUNNING' not in str(t.get_state()): t.resume()
    t.read32(addr)                    # 读 RAM 变量 / 外设寄存器
    t.read_memory_block8(addr, 4)     # 读 float
```

符号地址用 `arm-none-eabi-nm -C build/gimbal.elf` 现查 —— **每次重新构建后地址都可能变，不要写死**。

已用过的关键地址（本次构建）：

| 符号 | 地址 |
|---|---|
| `s_src`（输入源） | `0x2000025C` |
| `s_yaw` / `s_pitch` | `0x20000260` / `0x20000264` |
| `a_x` / `a_y` / `a_z` | `0x200002C8` / `CC` / `D0` |
| `g_z` | `0x200002E0` |
| `mpu6050_ok` | `0x200002FC` |
| `GPIOC_ODR`（PC5 指示灯） | `0x58020814` |
| `GPIOE_IDR`（PE3 按键） | `0x58021010` |
| ADC3 基地址（`DR` = +0x40） | `0x58026000` |

**为什么用 `attach` 模式**：不 halt CPU，可以观察固件**自己的循环**写下的值，而不是被调试器改过的状态。需要时也可以暂停 → 驱动寄存器 → 恢复，全程保存/恢复原寄存器值，可逆。

### 7.2 标定流程（两轴通用）

```
1. 改 gimbal.h 里对应的 *_SIGN 宏 → sh build/build.sh
2. pyocd flash ... build/gimbal.elf
3. SWD 读符号地址，确认 s_src 是目标模式
4. 摆到已知姿态/做已知动作，读 s_yaw / s_pitch
5. 对不上 → 回到 1
```

**实测标定记录**：

| 轴 | 姿态/动作 | 实测数据 | 结论 |
|---|---|---|---|
| MPU 俯仰 | 机头下压 47.7° | `a_x=+0.724, a_y≈0, a_z=+0.659` | 取反 `-1.0f` ✅ 已验证 |
| MPU 偏航 | 机头右转 | `s_yaw` 为负 | 取反 `-1.0f` ⚠️ 未在板验证 |
| 电位器偏航 | — | — | 未标定，`+1.0f` |
| 电位器俯仰 | — | — | 未标定，`+1.0f` |

### 7.3 上位机

USART3 / PD8 / 115200 → CH340 → Windows COM 口。VOFA+ 选 **FireWater** 模式。

上电应依次看到：

```
MPU6050 Init OK, WHO_AM_I=0x68
Calibrating... keep sensor still
..........
Accel offsets: ... Gyro  offsets: ...
Calibration done.
GIMBAL READY
0.0,0.0          ← 之后每 50ms 一行
```

---

## 8. 验证状态与已知问题

| 项 | 状态 |
|---|---|
| 编译 + 链接（零 warning / 零 undefined） | ✅ |
| 烧录 | ✅ |
| 电位器 ADC 链路 → 角度换算 | ✅ 已用 SWD 直接读 `DR` 对账确认 |
| PE3 按键切换 + PC5 指示灯 | ✅ |
| MPU 俯仰符号 | ✅ 板上验证过 |
| MPU 偏航符号 | ⚠️ **未在板验证**（两次抓取云台都没转动） |
| 电位器两轴符号 | ⚠️ 未标定 |
| 陀螺仪噪声 vs 死区 | ❌ **已知问题，未修**（见 §5.6） |
| HC06 蓝牙 | 未启用（`GIMBAL_HC06_ENABLE = 0`） |
| Keil / MDK 路径 | 改动已同步，**但未实测编译过**（本机没有 Keil） |

---

## 9. 下一步

1. **完成偏航符号的板上验证** —— 机头右转，确认 `s_yaw` 变正
2. **修陀螺仪低通 + 死区**（§5.6）：EMA τ≈0.3~0.5s，死区抬到 ~1.5°/s
3. **标定电位器两轴符号**
4. **接舵机**：调 `MX_TIM4_Init()` + `HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3/4)`，把 ±30° 映射到舵机脉宽（1MHz 计数下 `Pulse` 直接是微秒）
5. **把 PE3/PC5 补进 `H7_Arm.ioc`**，避免 CubeMX 重新生成时丢失
6. **把 `build/build.sh` 挪到 `tools/`** 并加进版本库（现在在 `build/` 下被 gitignore 忽略，换机器会丢）
7. 清理 `Core/Src/` 下两个游离的 IDE 文件 `PZtest.code-workspace`、`XZ_RCtest.code-workspace`
8. 如需更快的 MPU 采样，可把 I2C2 从 100kHz 提到 400kHz（改 `i2c.c` 的 `Timing`）
