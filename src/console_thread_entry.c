/*
 * console_thread —— 1 Hz 打印 MAVLink 链路状态
 *
 * 这一行就够判断"和飞控连上了没有"：
 *
 *   [MAV] RX b=1234 m=5 bad=0 drop=0 | TX m=12 err=0 | FC hb=3 sys=1 type=2 UP | ATT n=152 r=+3 p=-12 y=1375
 *
 *   RX b/m   : 收到的原始字节数 / 解析出的完整消息数（CRC 正确）
 *   RX bad   : CRC 错 —— 涨说明波特率或电平不对
 *   RX drop  : 环形缓冲溢出丢字节（正常应恒为 0）
 *   TX m/err : 发出去的消息数 / 发送失败次数
 *   FC hb    : 收到的对端心跳数 —— >0 即链路双向通
 *   sys/type : 对端 system id（PX4 飞控 = 1）和机型
 *   UP/--    : hb>0 ? "UP" : "--"
 *   ATT n    : 收到的 ATTITUDE 条数 —— 不涨说明飞控的流里没带姿态数据
 *   r/p/y    : roll/pitch/yaw，单位 0.1°（弧度 × 572.9578 后四舍五入）
 */
#include "console_thread.h"
#include "console.h"
#include "mavlink_app.h"

#define CONSOLE_PERIOD_MS   (1000U)

/* 弧度 → 0.1 度（整数打印，避开 %f 的浮点格式化路径） */
static int rad_to_deci_deg (float rad)
{
    float d = rad * 572.957795f;            /* 180/pi * 10 */
    return (int) ((d >= 0.0f) ? (d + 0.5f) : (d - 0.5f));
}

void console_thread_entry (void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    if (FSP_SUCCESS != console_init())
    {
        /* 控制台打不开 —— 没别的办法，停住（调试时看本线程是否卡在此处） */
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    console_write("\r\n[console] RA8P1 MAVLink monitor, 1 Hz\r\n");

    for (;;)
    {
        console_printf("[MAV] RX b=%lu m=%lu bad=%lu drop=%lu e=%lu/0x%lx | TX m=%lu err=%lu | FC hb=%lu sys=%u type=%u %s | ATT n=%lu r=%d p=%d y=%d\r\n",
                       (unsigned long) g_mav_rx_bytes,
                       (unsigned long) g_mav_rx_msgs,
                       (unsigned long) g_mav_rx_bad,
                       (unsigned long) g_mav_rx_dropped,
                       (unsigned long) g_mav_rx_err,
                       (unsigned long) g_mav_rx_last_evt,
                       (unsigned long) g_mav_tx_msgs,
                       (unsigned long) g_mav_tx_err,
                       (unsigned long) g_mav_fc_hb,
                       (unsigned int)  g_mav_fc_sysid,
                       (unsigned int)  g_mav_fc_type,
                       (g_mav_fc_hb > 0U) ? "UP" : "--",
                       (unsigned long) g_mav_att_count,
                       rad_to_deci_deg(g_mav_att_roll),
                       rad_to_deci_deg(g_mav_att_pitch),
                       rad_to_deci_deg(g_mav_att_yaw));

        vTaskDelay(pdMS_TO_TICKS(CONSOLE_PERIOD_MS));
    }
}
