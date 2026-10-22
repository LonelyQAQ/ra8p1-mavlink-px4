/*
 * MAVLink 应用层实现
 *
 * 中断里只做“取字节 + 塞环形缓冲”，其余全在任务里做 —— 本项目一贯的原则。
 */
#include "mavlink_app.h"

/* ================= 无锁环形缓冲（ISR 单生产者 / 任务单消费者） ================= */
/* MAV_RX_RING_SIZE 是 2 的幂，用掩码代替取模 */
#define RX_RING_MASK    (MAV_RX_RING_SIZE - 1U)

static volatile uint8_t  s_rx_ring[MAV_RX_RING_SIZE];
static volatile uint16_t s_rx_head;   /* ISR 写 */
static volatile uint16_t s_rx_tail;   /* 任务读 */

/* ================= 调试可观测状态 ================= */
volatile uint32_t g_mav_rx_bytes;
volatile uint32_t g_mav_rx_msgs;
volatile uint32_t g_mav_rx_bad;
volatile uint32_t g_mav_rx_dropped;
volatile uint32_t g_mav_rx_err;
volatile uint32_t g_mav_rx_last_evt;
volatile uint32_t g_mav_tx_msgs;
volatile uint32_t g_mav_tx_err;
volatile uint32_t g_mav_fc_hb;
volatile uint8_t  g_mav_fc_sysid;
volatile uint8_t  g_mav_fc_type;
volatile float    g_mav_att_roll;
volatile float    g_mav_att_pitch;
volatile float    g_mav_att_yaw;
volatile uint32_t g_mav_att_count;

/* ================= 内部状态 ================= */
static volatile bool s_uart_ready;      /* tx 线程据此等待 UART 就绪 */
static uint16_t      s_rx_tail_local;   /* 任务侧读指针的本地副本（减少 volatile 访问） */

/* 发送用的行缓冲（只有 tx 线程会碰它，无需保护） */
static uint8_t  s_tx_buf[MAV_TX_MAX_FRAME];
static uint16_t s_tx_len;

/* ================= 中断侧 ================= */

/**
 * UART 回调（中断上下文）。
 * 名字必须是 FSP 配置里填的那个 —— 见 ra_gen/hal_data.c 的 .p_callback。
 *
 * ⚠️ 这里只允许做“取字节 + 入队”，绝不能阻塞、打印、或调用非 FromISR 的 API。
 */
void mav_uart_callback(uart_callback_args_t *p_args)
{
    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        mav_app_rx_byte_from_isr((uint8_t) p_args->data);
    }
    else if (0U != (p_args->event & (UART_EVENT_ERR_PARITY | UART_EVENT_ERR_FRAMING |
                                     UART_EVENT_ERR_OVERFLOW | UART_EVENT_BREAK_DETECT)))
    {
        /* 只统计真正的线路错误（帧错/奇偶错/接收溢出/断路）。
         * ⚠️ 同一个回调也会收到 UART_EVENT_TX_DATA_EMPTY（发送 FIFO 空）—— 那是正常事件，
         *    必须排除，否则计数会被发送节奏灌满，看着像"一直在出错"。 */
        g_mav_rx_err++;
        g_mav_rx_last_evt = (uint32_t) p_args->event;
    }
}

void mav_app_rx_byte_from_isr(uint8_t byte)
{
    uint16_t next = (uint16_t) ((s_rx_head + 1U) & RX_RING_MASK);

    if (next == s_rx_tail)
    {
        /* 缓冲满 —— 对端发太快或任务没跟上。丢字节并计数，绝不阻塞。 */
        g_mav_rx_dropped++;
        return;
    }

    s_rx_ring[s_rx_head] = byte;
    s_rx_head = next;
    g_mav_rx_bytes++;
}

/* ================= 任务侧 ================= */

bool mav_app_rx_byte(uint8_t *p_byte)
{
    if (s_rx_tail_local == (uint16_t) s_rx_head)
    {
        return false;                       /* 空 */
    }

    *p_byte = s_rx_ring[s_rx_tail_local];
    s_rx_tail_local = (uint16_t) ((s_rx_tail_local + 1U) & RX_RING_MASK);
    s_rx_tail = s_rx_tail_local;            /* 回写给 ISR 看，用于判断“满” */
    return true;
}

void mav_app_init(void)
{
    s_rx_head       = 0U;
    s_rx_tail       = 0U;
    s_rx_tail_local = 0U;
    s_uart_ready    = false;

    g_mav_rx_bytes   = 0U;
    g_mav_rx_msgs    = 0U;
    g_mav_rx_bad     = 0U;
    g_mav_rx_dropped = 0U;
    g_mav_rx_err     = 0U;
    g_mav_rx_last_evt = 0U;
    g_mav_tx_msgs    = 0U;
    g_mav_tx_err     = 0U;
    g_mav_fc_hb      = 0U;
    g_mav_fc_sysid   = 0U;
    g_mav_fc_type    = 0U;
    g_mav_att_roll   = 0.0f;
    g_mav_att_pitch  = 0.0f;
    g_mav_att_yaw    = 0.0f;
    g_mav_att_count  = 0U;
}

fsp_err_t mav_app_uart_open(void)
{
    /* ⚠️ 本项目已验证的坑：MAVLink 是未知长度流，
     *    绝不要调用 R_SCI_B_UART_Read() —— 一旦有活动 read()，
     *    ISR 就不再上报 UART_EVENT_RX_CHAR，而是在填缓冲。
     *    这里只 Open，让每个字节走回调。 */
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart_mav_ctrl, &g_uart_mav_cfg);

    if (FSP_SUCCESS == err)
    {
        s_uart_ready = true;
    }
    return err;
}

bool mav_app_uart_is_ready(void)
{
    return s_uart_ready;
}

fsp_err_t mav_app_send(const mavlink_message_t *p_msg)
{
    fsp_err_t err;

    if (!s_uart_ready)
    {
        g_mav_tx_err++;
        return FSP_ERR_NOT_OPEN;
    }

    /* 序列化成字节流（MAVLink v2，含帧头/CRC） */
    s_tx_len = mavlink_msg_to_send_buffer(s_tx_buf, p_msg);

    /* FSP 的 Write 是异步的：立刻返回，字节在 TXI/TEI 中断里发出。
     * 上一个还没发完会返回 FSP_ERR_IN_USE —— 这里重试。
     * （本阶段只有 tx 线程一个发送方、1 Hz 节奏，几乎不会撞上； 
     *   将来多发送方时应改成 TX 环形缓冲 + 中断驱动，而不是重试。） */
    for (uint32_t retry = 0U; retry < 100U; retry++)
    {
        err = R_SCI_B_UART_Write(&g_uart_mav_ctrl, s_tx_buf, s_tx_len);
        if (FSP_ERR_IN_USE != err)
        {
            if (FSP_SUCCESS == err)
            {
                g_mav_tx_msgs++;
            }
            else
            {
                g_mav_tx_err++;
            }
            return err;
        }
        vTaskDelay(1);                      /* 让出 1 tick，等上一帧发完 */
    }

    g_mav_tx_err++;
    return FSP_ERR_IN_USE;
}
