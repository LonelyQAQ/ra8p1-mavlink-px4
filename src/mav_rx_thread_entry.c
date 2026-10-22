/*
 * mav_rx_thread —— 接收 + 解析 MAVLink
 *
 * 职责：
 *   1. 打开 UART（本工程里它拥有这个 UART 的 Open）
 *   2. 把 ISR 塞进环形缓冲的字节喂给 MAVLink 解析器
 *   3. 分发解析出来的消息
 *
 * 注意：**本线程不开文本控制台**。日志走 MAVLink 的 STATUSTEXT 消息，
 *       由 mav_tx_thread 发出去 —— 这样 MAVLink 本身既是协议通道也是日志通道。
 */
#include "mav_rx_thread.h"
#include "mavlink_app.h"

/* ---------------- 消息分发 ---------------- */

static void handle_message(const mavlink_message_t *p_msg)
{
    switch (p_msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            /* 对端心跳 —— 收到就说明链路双向通 */
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(p_msg, &hb);

            g_mav_fc_hb++;
            g_mav_fc_sysid = (uint8_t) p_msg->sysid;
            g_mav_fc_type  = hb.type;
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            /* 飞控下发的指令 —— 现阶段只解析不算，后续按需实现 */
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(p_msg, &cmd);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
            /* 飞控估计出的姿态角（弧度）。只保留最新值，不做历史。 */
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(p_msg, &att);

            g_mav_att_roll  = att.roll;
            g_mav_att_pitch = att.pitch;
            g_mav_att_yaw   = att.yaw;
            g_mav_att_count++;
            break;
        }

        case MAVLINK_MSG_ID_STATUSTEXT:
        case MAVLINK_MSG_ID_RC_CHANNELS:
        default:
        {
            /* 其余消息现阶段不处理。
             * 需要看内容时：在这个 switch 里加 case，
             * 或者用调试器看 g_mav_rx_msgs 的增速。 */
            break;
        }
    }
}

/* ---------------- 线程主体 ---------------- */

void mav_rx_thread_entry (void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    mav_app_init();

    fsp_err_t err = mav_app_uart_open();
    if (FSP_SUCCESS != err)
    {
        /* UART 打不开 —— 没有可用的恢复手段，停在这里。
         * 调试时看到本线程不再推进，就是 Open 失败了（看 err 的取值）。 */
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    mavlink_message_t msg;
    mavlink_status_t  status;

    for (;;)
    {
        bool drained = false;
        uint8_t byte;

        /* 把环形缓冲里积压的字节一次喂完 */
        while (mav_app_rx_byte(&byte))
        {
            drained = true;

            uint8_t framing = mavlink_frame_char(MAVLINK_COMM_0, byte, &msg, &status);

            if (MAVLINK_FRAMING_OK == framing)
            {
                g_mav_rx_msgs++;
                handle_message(&msg);
            }
            else if (MAVLINK_FRAMING_BAD_CRC == framing || MAVLINK_FRAMING_BAD_SIGNATURE == framing)
            {
                g_mav_rx_bad++;
            }
        }

        if (!drained)
        {
            /* 没数据就睡 1 tick，别空转占 CPU */
            vTaskDelay(1);
        }
        else
        {
            /* 还有活干，让同优先级任务有机会跑（也避免长占 CPU） */
            taskYIELD();
        }
    }
}
