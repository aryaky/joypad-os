// xbone_inject.h - 在透传流里插入自动化按键序列
//
// 板子的本职是中转器：手柄的原始输入报告应该原样转发给主机。注入要做的不是
// 另起一套报告生成，而是在**转发的那一份报文上改一个键**，手柄自己的其他按键、
// 摇杆全部保留 —— 也就是"按键插入手柄其他按键"。
//
// 当前注入内容：固定间隔按一次左扳机。
// 三个组合键：启动/停止、间隔 +1s、间隔 -1s。

#ifndef XBONE_INJECT_H
#define XBONE_INJECT_H

#include <stdint.h>
#include <stdbool.h>

// 默认 10 秒一次；每次按下的持续时间
#define XBONE_INJECT_DEFAULT_INTERVAL_MS 10000u
#define XBONE_INJECT_DEFAULT_PULSE_MS    500u

#define XBONE_INJECT_MIN_INTERVAL_MS     1000u
#define XBONE_INJECT_MAX_INTERVAL_MS   600000u

void xbone_inject_init(void);

// 每个准备转发给主机的手柄输入报告都先过这里。返回 true 表示 report 被改写过。
// 只在"正在按下"的那段时间改写；松开期间一个字节都不动，手柄真实的扳机值原样通过。
bool xbone_inject_apply(uint8_t* report, uint16_t len, uint32_t now_ms);

// 组合键检测。jp_buttons 是手柄原始按键字经 gip_button_map 翻译后的 JP_BUTTON_* 掩码。
void xbone_inject_feed_buttons(uint16_t jp_buttons, uint32_t now_ms);

bool     xbone_inject_is_running(void);
uint32_t xbone_inject_interval_ms(void);
uint32_t xbone_inject_pulse_ms(void);
void     xbone_inject_set_running(bool on);
void     xbone_inject_set_interval_ms(uint32_t ms);
void     xbone_inject_set_pulse_ms(uint32_t ms);

#endif // XBONE_INJECT_H
