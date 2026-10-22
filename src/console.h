/*
 * console —— SCI9 文本控制台（CH343 → COM24 → PC 串口助手）
 *
 * 用途：把 MAVLink 链路的计数打成人能看的文字，方便调试。
 * 与 MAVLink 完全分开：MAVLink 走 SCI6（纯二进制），控制台走 SCI9（纯文本）。
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include "hal_data.h"

/** 打开控制台 UART。成功后才能调用下面两个函数。 */
fsp_err_t console_init(void);

/** 发一个以 '\0' 结尾的字符串（内部会等上一帧发完）。 */
void console_write(const char *p_str);

/** 格式化后发出。⚠️ 内部用一个共享缓冲，**不要从多个任务同时调用**。 */
void console_printf(const char *p_fmt, ...);

#endif /* CONSOLE_H */
