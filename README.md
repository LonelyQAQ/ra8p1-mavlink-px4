# RA8P1 MAVLink ↔ PX4/Pixhawk

Renesas **RA8P1（R7KA8P1KFLCAC，Cortex-M85）** 作为机载计算机，通过 **SCI6 @57600** 与 PX4 飞控（Pixhawk 2.4.8）跑 MAVLink v2；FSP 6.6 + FreeRTOS + Keil MDK (AC6)。

> **English summary:** RA8P1 (Cortex-M85) acting as a companion computer speaking MAVLink v2 to a PX4/Pixhawk flight controller over SCI6. FSP 6.6 + FreeRTOS + Keil AC6. The link is verified in both directions (the FC's own `mavlink status` sees the traffic), and `ATTITUDE` is decoded on target.

## 验证结果

| 项目 | 结果 |
|---|---|
| 板 → 飞控 | 1 Hz `HEARTBEAT`（sysid 2 / comp 191，onboard controller）+ 5 s `STATUSTEXT` |
| 飞控 → 板 | `HEARTBEAT` / `ATTITUDE` / `COMMAND_LONG` 正常解析 |
| 双向确认 | 飞控自己的 `mavlink status` 能看到本机流量；`g_mav_fc_hb > 0` |
| 姿态数据 | `g_mav_att_count` 持续增长，`g_mav_att_roll/pitch/yaw` 在板端可读（弧度） |

## 硬件连接

```
RA8P1 SCI6 (PC13 = RXD6_C, PC14 = TXD6_C)  ── TX/RX 交叉 ──  PX4 TELEM2 (57600)
RA8P1 SCI9 (P208/P209)                     ── 文本控制台 115200
```

注意两点（都是本板实测过的）：

- **SCI6 的另一组引脚是 `P908`/`P909`（`TXD6_A`/`RXD6_A`）。** 一个 SCI 通道只能使能一组引脚，选了 `PC13`/`PC14` 这组，`P908`/`P909` 就不能再作 SCI6 使用。
- **`PC14` 在本板上同时是 nRF 的 `CTRL_IRQ` 网络**，占用前务必和硬件确认，否则会出现"串口能发不能收"之类的假象。这也是最初 `RX b=0` 的排查背景之一；另外 RX/TX 未交叉同样表现为收不到任何字节。

## 设计要点

1. **绝不调用 `R_SCI_B_UART_Read()`。** MAVLink 是未知长度字节流；FSP 的 `UART_EVENT_RX_CHAR` 只在"没有活动 read"时才上报。这里只 `Open()`，让每个字节从回调进来，塞进 **512 字节无锁环形缓冲**（ISR 单生产者 / 任务单消费者，容量取 2 的幂）。
2. **只有一个发送线程。** MAVLink 帧必须一条一条发，多发送方会互相踩；`mav_tx_thread` 是唯一调用 `mav_app_send()` 的地方。FSP 的 `Write` 是异步的，上一帧未发完会返回 `FSP_ERR_IN_USE`，这里用 1 tick 让出重试。
3. **日志走 STATUSTEXT。** 不额外占用串口，PC 侧（QGC 或 pymavlink）直接能看到板子的状态输出。
4. **错误计数要排除 `UART_EVENT_TX_DATA_EMPTY`。** 同一个 UART 回调也会收到发送相关事件，把它当错误会让 `g_mav_rx_err` 被发送节奏灌满。

## 目录结构

```
.
├── src/
│   ├── mavlink_app.c/.h          # 环形缓冲、UART 打开、发送、调试计数
│   ├── mav_rx_thread_entry.c     # 喂解析器 + 分发 HEARTBEAT/ATTITUDE/COMMAND_LONG
│   ├── mav_tx_thread_entry.c     # 1 Hz HEARTBEAT + 5 s STATUSTEXT
│   ├── console.c/.h              # 文本控制台（SCI9）
│   └── console_thread_entry.c
├── scripts/fetch_mavlink.sh      # 拉取 MAVLink c_library_v2（common 方言，pin 到 commit）
├── host/fake_fc.py               # PC 侧假飞控（pymavlink，串口或 UDP）
└── docs/MAVLink通信_环境与测试流程.md
```

`src/mavlink/` 不入库，按下面第 1 步生成。

## 快速开始

### 1. 拉取 MAVLink 库

```bash
./scripts/fetch_mavlink.sh
```

会在 `src/mavlink/` 下生成 `common/` 和顶层头文件（`mavlink_types.h`、`mavlink_helpers.h` 等）。上游是 `mavlink/c_library_v2`，MIT 许可。

### 2. RASC 配置

| 配置 | 值 |
|---|---|
| 板卡/器件 | Custom User Board，`R7KA8P1KFLCAC`，CPU0，FreeRTOS |
| UART #1 | SCI6，`PC13`/`PC14`，57600 8N1，回调 `mav_uart_callback`，实例 `g_uart_mav` |
| UART #2 | SCI9，`P208`/`P209`，115200，回调 `console_uart_callback`，实例 `g_uart_console` |
| 线程 | `mav_rx_thread` / `mav_tx_thread` / `console_thread` |

UART 高级项保持默认（FIFO/DTC 关闭）；本项目靠 `UART_EVENT_RX_CHAR` 逐字节接收。

### 3. Keil

- 把 `src/` 下的 `.c` 加入工程的 `<Files>` 组
- Include Path 加 `./src`（`src/mavlink` 在它下面）
- 编译（首次加入新文件需要编译两次，让 RASC 的 gpdsc 更新）

### 4. 烧录与联调

PC 侧用假飞控先验证，再接真飞控：

```bash
pip install pymavlink
python host/fake_fc.py --master COM3 --baud 57600     # 串口
python host/fake_fc.py --master udpin:0.0.0.0:14550   # UDP 自测
```

真飞控：TELEM2 接板子（TX/RX 交叉、共地），默认 57600；QGC 里也能看到本机心跳。

## 板端调试变量

| 变量 | 含义 |
|---|---|
| `g_mav_rx_bytes` | 收到的原始字节数（不涨 = 物理/引脚问题） |
| `g_mav_rx_msgs` | 解析成功且 CRC 正确的消息数 |
| `g_mav_rx_bad` | CRC 错误 / 丢弃帧 |
| `g_mav_rx_dropped` | 环形缓冲溢出（任务没跟上） |
| `g_mav_rx_err` / `g_mav_rx_last_evt` | 真正的线路错误次数 / 最近一次事件编码 |
| `g_mav_tx_msgs` / `g_mav_tx_err` | 已发送消息数 / 发送失败次数 |
| `g_mav_fc_hb` / `g_mav_fc_sysid` / `g_mav_fc_type` | 对端心跳计数 / system id / 机型 |
| `g_mav_att_roll/pitch/yaw`、`g_mav_att_count` | 最新姿态（弧度）与累计条数 |

排错顺序：`g_mav_rx_bytes` 不动 → 先查接线/交叉/引脚复用；字节在涨但 `g_mav_rx_msgs` 不涨 → 波特率或 CRC；`g_mav_rx_dropped` 在涨 → 任务优先级/环形缓冲。

## 已验证版本

| 组件 | 版本 |
|---|---|
| MCU | R7KA8P1KFLCAC（Cortex-M85） |
| FSP / RASC | 6.6.0 |
| FreeRTOS | v11.1.0（FSP 内置） |
| Keil MDK / AC6 | Arm Compiler 6.24 |
| MAVLink C 库 | `c_library_v2` @ `8c63a49`，`common` 方言 |
| 飞控 | Pixhawk 2.4.8 / PX4 |
| PC 侧 | pymavlink 2.4.50 |

## 许可与致谢

- 本仓库代码采用 **Apache-2.0**（见 [LICENSE](LICENSE)）。
- MAVLink C 库 `mavlink/c_library_v2` 为 **MIT**，由 `scripts/fetch_mavlink.sh` 拉取，不随仓库分发。
- `host/fake_fc.py` 使用 [pymavlink](https://github.com/ArduPilot/pymavlink)（MIT）。
- 详见 [NOTICE](NOTICE)。
