/*
 * MAVLink 应用层 —— 与飞控通信
 *
 * 分工：
 *   mav_rx_thread  : 拥有 UART，Open + 启动接收，解析 MAVLink 并分发
 *   mav_tx_thread  : 唯一的发送方，1 Hz HEARTBEAT + 周期 STATUSTEXT
 *
 * 关键设计（沿用本项目已验证的 FSP UART 用法）：
 *   UART_EVENT_RX_CHAR 只在“没有活动 read()”时上报。
 *   MAVLink 是未知长度流 —— 所以**绝不调用 R_SCI_B_UART_Read()**，
 *   而是让 ISR 每来一个字节就回调一次，回调里塞进无锁环形缓冲。
 */
#ifndef MAVLINK_APP_H
#define MAVLINK_APP_H

#include "hal_data.h"
#include "mavlink/common/mavlink.h"

/* ---------------- 本机 MAVLink 身份 ---------------- */
/* 飞控通常是 sysid=1/comp=1；主控用 sysid=2，避免和飞控撞号 */
#define MAV_SYS_ID                  (2U)
#define MAV_COMP_ID                 (191U)   /* MAV_COMP_ID_ONBOARD_COMPUTER */

/* ---------------- 缓冲尺寸 ---------------- */
#define MAV_RX_RING_SIZE            (512U)   /* 必须是 2 的幂 —— ISR→任务 无锁环形缓冲 */
#define MAV_TX_MAX_FRAME            (288U)   /* MAVLink v2 最大帧（含签名） */

/* ---------------- 对外接口 ---------------- */

/** 初始化协议层（建通信对象）。必须在调度器启动后、用 UART 之前调用一次。 */
void mav_app_init(void);

/** 打开 UART 并开始接收。由 mav_rx_thread 调用。 */
fsp_err_t mav_app_uart_open(void);

/** 发送一条 MAVLink 消息（只有 mav_tx_thread 应该调它）。 */
fsp_err_t mav_app_send(const mavlink_message_t *p_msg);

/** 供 ISR 调用：把一个收到的字节放进环形缓冲。 */
void mav_app_rx_byte_from_isr(uint8_t byte);

/** 从环形缓冲取一个字节。返回 false 表示当前为空。 */
bool mav_app_rx_byte(uint8_t *p_byte);

/** UART 是否已经打开（tx 线程据此等待） */
bool mav_app_uart_is_ready(void);

/* ---------------- 调试可观测状态（在调试器 Watch 里看） ---------------- */
extern volatile uint32_t g_mav_rx_bytes;      /* 收到的原始字节数 */
extern volatile uint32_t g_mav_rx_msgs;       /* 解析出的完整消息数（CRC 正确） */
extern volatile uint32_t g_mav_rx_bad;        /* CRC 错误 / 丢弃 */
extern volatile uint32_t g_mav_rx_dropped;    /* 环形缓冲溢出丢弃 */
extern volatile uint32_t g_mav_rx_err;        /* 线路错误事件计数（帧错/奇偶错/溢出/断路）—— 只统计真错误，不含 TX 事件 */
extern volatile uint32_t g_mav_rx_last_evt;   /* 最近一次“非 RX_CHAR”事件的原始编码，见 UART_EVENT_* */
extern volatile uint32_t g_mav_tx_msgs;      /* 已发出的消息数 */
extern volatile uint32_t g_mav_tx_err;       /* 发送失败次数 */
extern volatile uint32_t g_mav_fc_hb;        /* 收到的“对端心跳”计数（>0 说明链路双向通） */
extern volatile uint8_t  g_mav_fc_sysid;     /* 对端 system id */
extern volatile uint8_t  g_mav_fc_type;      /* 对端 MAV_TYPE */
extern volatile float    g_mav_att_roll;     /* ATTITUDE.roll（弧度） */
extern volatile float    g_mav_att_pitch;    /* ATTITUDE.pitch（弧度） */
extern volatile float    g_mav_att_yaw;      /* ATTITUDE.yaw（弧度） */
extern volatile uint32_t g_mav_att_count;    /* 收到的 ATTITUDE 条数 —— 不涨说明飞控的流里没有姿态数据 */

#endif /* MAVLINK_APP_H */
