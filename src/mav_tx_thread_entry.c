/*
 * mav_tx_thread —— 唯一的 MAVLink 发送方
 *
 * 职责：
 *   1. 等 UART 就绪（由 mav_rx_thread 负责 Open）
 *   2. 1 Hz 发 HEARTBEAT，声明自己是一个 onboard computer
 *   3. 每 5 秒发一条 STATUSTEXT，把链路状态/计数报出去
 *      —— 这样 PC 侧不需要第二个串口就能看到日志
 *
 * 为什么 TX 单独一个线程：MAVLink 只能一条一条发，多发送方会互相踩。
 * 现阶段只有本线程发送，所以 mav_app_send() 里的重试足够；
 * 将来要加别的发送方时，改成“队列 + 本线程写”即可。
 */
#include "mav_tx_thread.h"
#include "mavlink_app.h"

#define HB_PERIOD_MS        (1000U)
#define STATUS_PERIOD_MS    (5000U)
#define TX_WAIT_MS          (10U)

void mav_tx_thread_entry (void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    mavlink_message_t msg;

    /* 等 mav_rx_thread 把 UART 打开 —— 单核同一颗 CPU，volatile 标志足够 */
    while (!mav_app_uart_is_ready())
    {
        vTaskDelay(pdMS_TO_TICKS(TX_WAIT_MS));
    }

    TickType_t next_hb     = xTaskGetTickCount();
    TickType_t next_status = xTaskGetTickCount() + pdMS_TO_TICKS(500U);

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        /* ---------- 1 Hz 心跳 ---------- */
        if ((int32_t) (now - next_hb) >= 0)
        {
            mavlink_msg_heartbeat_pack(
                MAV_SYS_ID, MAV_COMP_ID, &msg,
                MAV_TYPE_ONBOARD_CONTROLLER,   /* 我们是机载计算机，不是飞控 */
                MAV_AUTOPILOT_INVALID,         /* 不冒充自动驾驶仪 */
                0U, 0U,                        /* base_mode / custom_mode */
                MAV_STATE_ACTIVE);

            (void) mav_app_send(&msg);
            next_hb = now + pdMS_TO_TICKS(HB_PERIOD_MS);
        }

        /* ---------- 周期状态上报 ---------- */
        if ((int32_t) (now - next_status) >= 0)
        {
            char text[50];

            /* 简短、别超过 50 字符（MAVLink STATUSTEXT 的硬限制） */
            (void) snprintf(text, sizeof(text), "RX%d/%d TX%d FC%d D%d",
                            (int) g_mav_rx_bytes,
                            (int) g_mav_rx_msgs,
                            (int) g_mav_tx_msgs,
                            (int) g_mav_fc_hb,
                            (int) g_mav_rx_dropped);

            mavlink_msg_statustext_pack(
                MAV_SYS_ID, MAV_COMP_ID, &msg,
                MAV_SEVERITY_INFO, text, 0U, 0U);

            (void) mav_app_send(&msg);
            next_status = now + pdMS_TO_TICKS(STATUS_PERIOD_MS);
        }

        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}
