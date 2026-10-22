#include "console.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static volatile bool s_console_ready;
static char          s_out_buf[192];    /* 1 Hz 状态行最长约 163 字符（计数器全满位时） */

/**
 * 控制台 UART 的 FSP 回调。
 * 名字必须和 RASC 里填的一致 —— 否则链接报 L6218E（这个坑刚踩过）。
 *
 * 目前控制台只发不收，所以是个空实现。
 * 以后要在控制台上做命令行（收 PC 发的命令），字节就从这里进。
 */
void console_uart_callback(uart_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
}

fsp_err_t console_init(void)
{
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart_console_ctrl, &g_uart_console_cfg);

    if (FSP_SUCCESS == err)
    {
        s_console_ready = true;
    }
    return err;
}

void console_write(const char *p_str)
{
    if (!s_console_ready || (NULL == p_str))
    {
        return;
    }

    size_t len = strlen(p_str);
    if (0U == len)
    {
        return;
    }

    /* FSP 的 Write 是异步的；上一帧没发完会返回 FSP_ERR_IN_USE。
     * 控制台只有 console_thread 在写，1 Hz 节奏，重试足够。 */
    for (uint32_t retry = 0U; retry < 200U; retry++)
    {
        fsp_err_t err = R_SCI_B_UART_Write(&g_uart_console_ctrl,
                                           (uint8_t const *) p_str, len);
        if (FSP_ERR_IN_USE != err)
        {
            return;
        }
        vTaskDelay(1);
    }
}

void console_printf(const char *p_fmt, ...)
{
    va_list ap;

    va_start(ap, p_fmt);
    (void) vsnprintf(s_out_buf, sizeof(s_out_buf), p_fmt, ap);
    va_end(ap);

    console_write(s_out_buf);
}
