# MAVLink 通信打通 —— 环境与测试流程

> 建立日期：2026-10-16
> 目标：让 RA8P1 主控 ↔ 飞控 之间跑通 MAVLink
> 状态：**PC 侧已就绪并验证**；RA8P1 侧已建，**与 Pixhawk 2.4.8 的双向链路已通（2026-10-16）**

---

## 1. 已经准备好的东西

| 项 | 位置 | 状态 |
|---|---|---|
| **MAVLink C 库**（MCU 侧用） | `mavlink-c_library_v2\` | ✅ 已拉取（blobless+稀疏，只取 `common` 方言） |
| **pymavlink**（PC 侧用） | venv：`venv\` | ✅ 已装 `pymavlink 2.4.50` |
| **假飞控脚本** | `host/fake_fc.py` | ✅ 已写，UDP 自环验证通过 |

**C 库内容**：根目录 7 个头文件（`mavlink_helpers.h` / `mavlink_types.h` / `protocol.h` / `checksum.h` / `mavlink_conversions.h` / `mavlink_get_info.h` / `mavlink_sha256.h`）+ `common/` 下 **236 个消息定义头文件**。

**验证过什么**：两个实例通过 UDP 互发，监听端连续收到并正确解码
`HEARTBEAT ... type=MAV_TYPE_FIXED_WING autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA status=MAV_STATE_ACTIVE`
⇒ 编码 / 解码 / 枚举名解析 / 收发循环全部正常。

---

## 2. 硬件接线（RA8P1 ↔ PC）

飞控的 UART 接口（**待确认具体座子**），经 USB-TTL 接 PC：

```
RA8P1 TX  ──→  USB-TTL RX
RA8P1 RX  ←──  USB-TTL TX
GND       ────  GND          ← 必须共地
```

⚠️ **TX/RX 必须交叉**；不共地是最常见的"收不到"原因。

**参考波特率**：MAVLink 常用 **57600**（兼容性最好）或 **921600**（吞吐高）。
先跑 57600，通了再往上试。

---

## 3. 怎么跑测试

```bash
# PC 侧（用 venv 里的 python）
python host/fake_fc.py --master COM3 --baud 57600
```

把 `COM3` 换成 USB-TTL 枚举出的串口。

**它会做两件事**：
1. 以 1 Hz 发 `HEARTBEAT`（扮演飞控）
2. 打印 RA8P1 发来的所有消息（`HEARTBEAT` / `STATUSTEXT` / `LANDING_TARGET` / `OBSTACLE_DISTANCE` / `DISTANCE_SENSOR` 会做友好解码）

**成功的样子**：
```
[fake-fc] 等待对端（RA8P1）HEARTBEAT，最多 30s ...
  [OK] 收到对端心跳：sys=... comp=...
```

> ⚠️ 本脚本在 Git Bash 下跑要注意 **stdout 缓冲**：重定向到文件时必须加 `-u`，
> 否则 kill 时看不到输出（这个坑已经踩过一次）。

---

## 4. RA8P1 侧（✅ 已建工程，编译通过）

工程：**`<project>`**（`MAVLink_UART.uvprojx`）

| 项 | 值 |
|---|---|
| 器件 / 核 | `R7KA8P1KFLCAC` / CPU0（单核 Flat） |
| 板级定义 | **Custom User Board**（不是 EK-RA8P1 —— 避开了板级定义不符那个坑） |
| UART | **SCI6**，句柄 `g_uart_mav`，**57600 8N1** |
| 引脚 | **PC14 = `TXD6_C`（发）**、**PC13 = `RXD6_C`（收）** |
| 线程 | `mav_rx_thread`（优先级 4，2048 B）/ `mav_tx_thread`（优先级 3，2048 B）/ `console_thread`（优先级 2，2048 B，SCI9 @115200 文本控制台 1 Hz） |
| 编译 | `Code=19916` / `RO=5924` / `ZI=11980`，**0 error**（44 条 warning 全在 MAVLink 库自己的头文件里） |

> ⚠️ **踩过的坑：FSP 线程栈属性的单位是「字节」，不是「字」。**
> 生成代码是 `static uint8_t xxx_thread_stack[2048]` + `xTaskCreateStatic(..., 2048/4, /* In words */ ...)`，
> 属性填 `512` 实际只给到 **512 字节**。`mav_rx_thread_entry()` 里一个 `mavlink_message_t`
> 局部变量就 ~280 字节，512 B 必然溢出 → UsageFault `STKOF` →（FORCED）HardFault →
> 落 `Default_Handler` 执行 `BKPT #0`。现象是「烧进去就死、串口一个字都没有」。
> 三个线程现已统一 2048（与能正常跑的 `UART` 工程一致，那个工程每个线程也是 2048）。
>
> 另一个连带现象：RX 线程在 `R_SCI_B_UART_Open()` 阶段就崩了，`s_uart_ready` 一直是 false，
> 于是 TX 线程卡在等待、`g_mav_tx_msgs` 恒为 0 —— 看到「`TX m=0` 且 `err=0`」就要想到这个。

### 已实现的行为

- **RX**：ISR 每来一个字节 → 无锁环形缓冲 → `mav_rx_thread` 喂给 `mavlink_frame_char()` 解析并分发
  - ⚠️ **绝不调用 `R_SCI_B_UART_Read()`** —— 一旦有活动 read()，ISR 就不再上报 `UART_EVENT_RX_CHAR`，MAVLink 这种未知长度流会收不到
- **TX**：`mav_tx_thread` 是唯一发送方
  - **1 Hz HEARTBEAT**（`MAV_TYPE_ONBOARD_CONTROLLER`，sysid=2 / compid=191）
  - **每 5 秒一条 STATUSTEXT**，内容是链路计数 → **日志走 MAVLink 本身，不需要第二个串口当控制台**
- **ATTITUDE 解码**（2026-10-18）：`mav_rx_thread` 解析 `ATTITUDE`，只保留最新 roll/pitch/yaw（弧度）；
  控制台 1 Hz 行尾追加 `ATT n=.. r=.. p=.. y=..`，角度按 **0.1° 的整数**打印
  （故意不用 `%f` —— 刚踩过栈溢出，不想再压 `vsnprintf` 的浮点路径）。
  **✅ 上板实测通过（同一天）**：`ATT n` 持续增长、手动倾斜飞控时 r/p 跟着变。

> **带宽预算（2026-10-18 实测）**：TELEM2 = 57600 8N1，飞控自限 `tx rate max: 2880 B/s`，
> 实际已用 ~2328 B/s ⇒ **只剩约 550 B/s**。
> `ATTITUDE`（~40 B/帧）能跑 ~10 Hz；`SCALED_IMU`（~36 B）同样只到 10 Hz；
> **`HIGHRES_IMU`（~74 B/帧）在 57600 下连 10 Hz 都塞不下** —— 要原始 IMU 必须两端都提到 921600
> （或大幅降其它流）。以后加 `LANDING_TARGET` 之类的消息，都要先算这笔账。

### 调试可观测变量（Keil Watch 窗口里加）

```c
g_mav_rx_bytes     // 收到的原始字节数
g_mav_rx_msgs      // 解析出的完整消息数（CRC 正确）
g_mav_rx_bad       // CRC 错 / 签名错
g_mav_rx_dropped   // 环形缓冲溢出丢字节
g_mav_rx_err       // 线路错误事件数（帧错/奇偶错/接收溢出/断路）
g_mav_rx_last_evt  // 最近一次线路错误的原始 UART_EVENT_* 编码（0x10=帧错，0x8=奇偶错，0x20=溢出，0x40=断路）
g_mav_tx_msgs      // 已发出的消息数
g_mav_fc_hb        // 收到的对端心跳数 —— >0 就说明链路双向通了
g_mav_fc_sysid     // 对端 system id（飞控一般是 1）
g_mav_att_roll     // 最近一条 ATTITUDE 的 roll（弧度）
g_mav_att_pitch    // 最近一条 ATTITUDE 的 pitch（弧度）
g_mav_att_yaw      // 最近一条 ATTITUDE 的 yaw（弧度）
g_mav_att_count    // 收到的 ATTITUDE 条数 —— 不涨说明飞控的流里没带姿态数据
```

> ⚠️ **统计"线路错误"必须按位掩码过滤。** `mav_uart_callback` 这个回调**同时也会收到
> `UART_EVENT_TX_DATA_EMPTY`（0x2，发送 FIFO 空）** —— 那是正常事件。
> 第一版把"非 `RX_CHAR` 的事件"全算成错误，结果 `e` 被发送节奏灌满、看着像一直在出错。
> 正确写法：`event & (ERR_PARITY|ERR_FRAMING|ERR_OVERFLOW|BREAK_DETECT)`。

### 接线

```
RA8P1 PC14 (TXD6_C) ──→ USB-TTL RX
RA8P1 PC13 (RXD6_C) ←── USB-TTL TX
GND ──────────────── GND          ← 必须共地，不共地是最常见的"收不到"
```

**PC13/PC14 是 3.3V TTL 电平**，需要一个 USB-TTL 转换器。

### 以后要换到飞控的口

改 `configuration.xml` 里 UART 栈的 Channel + Pins 页的引脚即可（RASC 里 2 分钟），
**协议层代码一行不用改** —— `mavlink_app.c` / 两个线程都是通道无关的。

> ⚠️ 注意：**SCI6 有另一组引脚 P908/P909**（你们板上标的是 "WIFI"）。
> 同一个 SCI 通道只能启用一组引脚，选了 PC13/PC14 就等于 SCI6 不给 WiFi 用了。

---

## 5. 飞控侧（PX4 + QGC）配置

飞控：**Pixhawk 2.4.8（PX4）**，走 **TELEM2** —— PX4 官方给机载计算机的推荐口。

### 5.1 QGC 里改参数（USB 连飞控，不是连我们的板子）

1. USB 连 Pixhawk → 打开 QGC，等它连上（顶部显示机型、状态为已连接）。
2. **齿轮（Vehicle Setup）→ Parameters**，搜索框里输入参数名。
3. `MAV_1_CONFIG` = **`TELEM 2`** → 保存后 QGC 会提示需要重启，**重启飞控**。
   ⚠️ 必须先重启：重启后 TELEM2 那一组参数才会出现在列表里。
4. 重启后再设：

   | 参数 | 值 | 说明 |
   |---|---|---|
   | `MAV_1_CONFIG` | `TELEM 2` | 把 MAVLink 通道 1 指到 TELEM2 |
   | `MAV_1_MODE` | `Onboard` | 给机载计算机用（`Normal` 是给地面站用的） |
   | `MAV_1_RATE` | 默认，先不动 | |
   | `MAV_1_FORWARD` | 默认，先不动 | |
   | `SER_TEL2_BAUD` | **57600** | **必须和固件的 57600 一致**。PX4 官方给机载计算机的例子写的是 921600（为了高带宽），我们先按低速跑通 |

5. 再重启一次（波特率 / MODE 生效），然后看板子控制台的 `RX b` / `FC hb` 是否开始增长。

> **实测记录（2026-10-16，这台 Pixhawk 2.4.8 的原厂默认值）**
> - `MAV_1_CONFIG` 默认 = `Disabled`（MAVLink 通道 1 默认没分配到任何口）
> - `SER_TEL2_BAUD` 默认 = `921600 8N1` ⇒ 必须改成 `57600 8N1` 才和固件一致
> - `MAV_1_MODE` / `MAV_1_FORWARD` / `MAV_1_RATE` 在 `MAV_1_CONFIG` 仍是 `Disabled` 时**搜不到** ——
>   改完 `TELEM 2` 并**重启**后才会出现。搜不到不等于这个版本没有这些参数。
> - 两处改动 QGC 都会提示 **Vehicle reboot required after change**，必须重启才生效。

> 如果参数列表里**找不到 `SER_TEL2_BAUD`**，说明这个 PX4 版本的 TELEM2 波特率是编译期固定的
> （常见 921600）—— 那就反过来改我们这边：RASC 里把 SCI6 波特率改成对应值重生成，协议层代码一行不用动。

### 5.2 接线（我们这边 TX = PC14，RX = PC13）

```
RA8P1 PC14 (TXD6_C, 发) ──→ 飞控 TELEM2 RX
RA8P1 PC13 (RXD6_C, 收) ←── 飞控 TELEM2 TX
GND ──────────────────────── 飞控 TELEM2 GND   ← 必须共地
```

Pixhawk 标准 6 针 TELEM 座（GH1.25/DF13，具体以你板子上丝印为准）：
`1=+5V  2=TX  3=RX  4=CTS  5=RTS  6=GND`。

### 5.3 `RX b = 0` 的排障顺序（✅ 已按此定位并解决）

> **根因（2026-10-16）：两根数据线没有交叉。** 自环测试先把范围切成"我们这边全通"，
> 从而指向板外接线；把 TX/RX 对调后立刻通了。
>
> **通了的样子**（连续跑，无任何错误）：
> ```
> [MAV] RX b=28365 m=480 bad=0 drop=0 e=0/0x0 | TX m=15 err=0 | FC hb=11 sys=1 type=2 UP
> ```
> `sys=1` + `type=2`（`MAV_TYPE_QUADROTOR`）= 收到的确实是 Pixhawk 的心跳；
> `bad=0` 说明波特率也对（否则 CRC 不会全对）。

排障顺序（照这个走最快）：

1. **自环自检（不需要飞控，决定性）**：拔掉接飞控的信号线，把 PC13 和 PC14 短接 →
   控制台 `RX b` 应该飞快增长。**这一步做过、通过**，所以后面只可能是板外问题。
   ⚠️ 做之前一定先拔掉飞控那两根线，否则飞控的 TX 会和我们的 TX 顶在一起。
2. **确认飞控到底在不在发**：拔掉我们的板子，把 TELEM2 接 USB-TTL 到 PC，QGC 里加一条串口 comm link（COMx @57600）。
   QGC 能连上机型 ⇒ TELEM2 活着；PC 也收不到 ⇒ 飞控侧参数没生效。
3. **TX/RX 是否交叉**（PC14→飞控RX、PC13←飞控TX）。**← 本次就是这一条**。
4. **波特率两端一致**（都 57600）。
5. **共地**（不共地是最常见的"收不到"）。

### 5.4 硬件侧要拍板的冲突

改板表里 **J6-3 `PC14` 已经分给了 `CTRL_IRQ`（nRF24L01 中断）**（见 `J6_全引脚分配.csv`），
而固件现在把 **PC13/PC14 整组给了 SCI6**（选了这个复用就等于 SCI6_A 的 P908/P909 不给用）。二者不能共存：
要么给 SCI6 换一组脚，要么给 nRF 的 IRQ 换脚。画板前必须拍板。

### 5.5 怎么确认两个方向都真的通了（✅ 已用这个方法验证）

QGC 里 **Analyze Tools → Mavlink 控制台**，敲一条只读命令：

```
mavlink status
```

它会按实例列出端口、波特率、模式，以及**每个实例"从哪些 sysid 收到过消息"**。实测（2026-10-16，我们的板子接在 TELEM2）：

```
instance #1:
	type:       GENERIC LINK OR RADIO
	rx: 21.0 B/s
	Received Messages:
		sysid: 2, compid:191, Total: 155 (lost: 252)
	mode: Onboard
	Forwarding: Off
	transport protocol: serial (/dev/ttyS2 @57600)
```

- **`sysid:2, compid:191` = 飞控确实收到了我们发的消息** ⇒ 我们→飞控 这个方向有硬证据（不必再靠 `err=0` 推断）。
- `rx: 21.0 B/s` ≈ 我们 1 Hz 心跳 + 5 s 一条 STATUSTEXT 的速率，对得上。
- `tx: 2328.0 B/s` ≈ 板子侧 `RX b` ≈ 2.2 kB/s，两个方向的数字互相印证。
- `mode: Onboard` ⇒ `MAV_1_MODE` 确实生效了（这一条之前是悬着的）。
- `(lost: 252)` 不用管：板子每次重启/重烧后 MAVLink 序号从 0 重来，飞控会记一大笔"丢失"。

> ⚠️ **不要用 MAVLink Inspector 查这件事。** TELEM2 实例是 `Forwarding: Off` ⇒ 飞控**不**把 TELEM2 收到的消息转发给 QGC 那条链路，所以 Inspector 里只看得到 sysid 1，永远看不到 2。

**反向核对（防误判，已做过）：** 把板子**断电**再跑一次 `mavlink status` ⇒ instance #1 的 `Received Messages:` **整段直接消失**。

| | 板子上电 | 板子断电 |
|---|---|---|
| instance #1 `Received Messages` | `sysid: 2, compid:191, Total: 155` | 没有这一段 |
| instance #1 `rx` | 21.0 B/s | 117.9 B/s |

- 判据是**时间**：飞控重启后过了 ~100 s（由 instance #2 从 QGC 收到的条数 ¥1 Hz 推得），板子若在发，这 100 s 里必然已出现上百条心跳。
- 断电报文为空、但 `rx` 仍有 ~118 B/s 字节且 `rx loss: nan%` ⇒ 那是 TELEM2 的 **RX 线在驱动端（我们的板子）断电后悬空收到的噪声**，永远拼不成合法报文。这反过来也说明那根线确实接在我们的板子上。

> ℹ️ 飞控重启会让所有实例计数归零、QGC 控制台清空，还会弹一次 "Configuration tasks remain before this vehicle is ready to fly" 提示框（纯信息，点 OK 即可）。做前后对比时要把这一点算进去。

---

## 6. 待定事项（需拍板）

| # | 问题 | 影响 |
|---|---|---|
| 1 | **飞控接在哪个座子 / 哪个 SCI** | 决定 FSP 里配哪个 UART |
| 2 | **检测结果发给飞控做什么用** | 决定消息集：`LANDING_TARGET`（精准降落）/ `OBSTACLE_DISTANCE`（避障）/ `STATUSTEXT`（仅上报）/ 自定义 |
| 3 | **比赛是否有指定协议** | 若有，按比赛文档的消息定义来 |
| 4 | 波特率 | 决定 SCI 的分频配置 |

---

## 7. 附：常用 MAVLink 消息速查（主控视角）

**主控 → 飞控**
| 消息 | 用途 |
|---|---|
| `HEARTBEAT` | 必须，声明自身存在（type=ONBOARD_CONTROLLER） |
| `STATUSTEXT` | 文本上报，调试/告警 |
| `LANDING_TARGET` | 精准降落：给飞控目标的相对位置 |
| `OBSTACLE_DISTANCE` | 避障：360° 距离数组 |
| `DISTANCE_SENSOR` | 单点测距 |
| `COMMAND_ACK` | 回应飞控下发的命令 |

**飞控 → 主控**
| 消息 | 用途 |
|---|---|
| `HEARTBEAT` | 确认链路 |
| `ATTITUDE` / `VFR_HUD` | 姿态/速度，用于图像稳定或目标换算 |
| `GLOBAL_POSITION_INT` | 位置，用于目标地理标定 |
| `RC_CHANNELS` | 遥控通道，可用来触发/切换识别模式 |
| `COMMAND_LONG` | 飞控下发指令 |
