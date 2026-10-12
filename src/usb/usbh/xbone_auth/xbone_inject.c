// xbone_inject.c - 透传流上的按键注入
//
// 组合键判定复用上游 core/services/hotkeys：它已经做了"掩码内所有键都按下"的
// 匹配和 tap/hold 区分，自己再写一遍只会多一份要维护的状态机。

#include "xbone_inject.h"

#include "core/buttons.h"
#include "core/services/hotkeys/hotkeys.h"
#include "gip_button_map.h"
#include "platform/platform.h"

#include <stdio.h>

// ============================================================================
// 组合键定义
// ============================================================================
//
// 三个都拿 View(S1) 打头：Minecraft 里单独按 View 没有动作，误触代价最小。
// 要换物理按键只改这三行。
#define COMBO_TOGGLE (JP_BUTTON_S1 | JP_BUTTON_B1)  // 启动 / 停止循环发送
#define COMBO_SLOWER (JP_BUTTON_S1 | JP_BUTTON_B3)  // 间隔 +1s
#define COMBO_FASTER (JP_BUTTON_S1 | JP_BUTTON_B2)  // 间隔 -1s

// 短于这个时长算"点一下"，避免长按期间反复触发
#define COMBO_TAP_MS 700u

// ============================================================================
// 输入报告里的左扳机位置
// ============================================================================
//
// GIP 输入报告布局：头 4 字节 + 按键字 2 字节 + 左扳机 2 字节(0-1023)...
// 实测手柄报文字节序与 usb/usbd/descriptors/xbone_descriptors.h 里
// gip_input_report_t 一致（cmd=0x03 那条是 03 20 12 04 80 00 00 00）。
#define LT_OFFSET    6
#define LT_BYTES     2
#define LT_PRESSED   1023

// ============================================================================
// 状态
// ============================================================================

static bool     running;
static uint32_t interval_ms = XBONE_INJECT_DEFAULT_INTERVAL_MS;
static uint32_t pulse_ms    = XBONE_INJECT_DEFAULT_PULSE_MS;

static uint32_t cycle_start_ms;
static bool     pressing;          // 当前这一刻是否正按着左扳机
static bool     combo_registered;

// ============================================================================
// 组合键回调
// ============================================================================

static void on_combo_toggle(uint8_t player, uint32_t held_ms)
{
    (void)player; (void)held_ms;
    xbone_inject_set_running(!running);
}

static void on_combo_slower(uint8_t player, uint32_t held_ms)
{
    (void)player; (void)held_ms;
    xbone_inject_set_interval_ms(interval_ms + 1000u);
}

static void on_combo_faster(uint8_t player, uint32_t held_ms)
{
    (void)player; (void)held_ms;
    xbone_inject_set_interval_ms(interval_ms - 1000u);
}

static void register_combos(void)
{
    if (combo_registered) return;

    HotkeyDef defs[3] = {
        { .buttons     = COMBO_TOGGLE,
          .duration_ms = COMBO_TAP_MS,
          .trigger     = HOTKEY_TRIGGER_ON_TAP,
          .callback    = on_combo_toggle,
          .global      = false },
        { .buttons     = COMBO_SLOWER,
          .duration_ms = COMBO_TAP_MS,
          .trigger     = HOTKEY_TRIGGER_ON_TAP,
          .callback    = on_combo_slower,
          .global      = false },
        { .buttons     = COMBO_FASTER,
          .duration_ms = COMBO_TAP_MS,
          .trigger     = HOTKEY_TRIGGER_ON_TAP,
          .callback    = on_combo_faster,
          .global      = false },
    };

    for (int i = 0; i < 3; i++) {
        if (hotkeys_register(&defs[i]) < 0) {
            printf("[inject] WARN: combo %d failed to register\n", i);
        }
    }
    combo_registered = true;
}

// ============================================================================
// 对外接口
// ============================================================================

void xbone_inject_init(void)
{
    running = false;
    interval_ms = XBONE_INJECT_DEFAULT_INTERVAL_MS;
    pulse_ms = XBONE_INJECT_DEFAULT_PULSE_MS;
    cycle_start_ms = platform_time_ms();
    pressing = false;
    register_combos();
}

uint16_t xbone_inject_trigger_value(uint32_t now_ms)
{
    if (!running) return 0;

    uint32_t phase = now_ms - cycle_start_ms;
    if (phase >= interval_ms) {
        // 周期到了就重新起算。跳过了多个周期（日志/阻塞导致）时不要积压补发，
        // 直接对齐到当前时刻，否则会连续按下一串。
        cycle_start_ms = now_ms;
        phase = 0;
    }

    bool should_press = (phase < pulse_ms);

    if (should_press != pressing) {
        pressing = should_press;
        printf("[inject] left trigger %s (interval=%ums pulse=%ums)\n",
               should_press ? "DOWN" : "up",
               (unsigned)interval_ms, (unsigned)pulse_ms);
    }

    return should_press ? LT_PRESSED : 0;
}

bool xbone_inject_apply(uint8_t* report, uint16_t len, uint32_t now_ms)
{
    if (report == NULL || len < (LT_OFFSET + LT_BYTES)) return false;

    uint16_t value = xbone_inject_trigger_value(now_ms);
    if (value == 0) {
        // 松开期间一个字节都不动：手柄自己真实的扳机值必须原样到主机
        return false;
    }

    report[LT_OFFSET]     = (uint8_t)(value & 0xFF);
    report[LT_OFFSET + 1] = (uint8_t)((value >> 8) & 0xFF);
    return true;
}

void xbone_inject_feed_buttons(uint16_t jp_buttons, uint32_t now_ms)
{
    (void)now_ms;
    if (!combo_registered) register_combos();
    hotkeys_check((uint32_t)jp_buttons, 0);
}

bool xbone_inject_is_running(void)
{
    return running;
}

uint32_t xbone_inject_interval_ms(void)
{
    return interval_ms;
}

uint32_t xbone_inject_pulse_ms(void)
{
    return pulse_ms;
}

void xbone_inject_set_running(bool on)
{
    if (on == running) return;
    running = on;
    pressing = false;
    cycle_start_ms = platform_time_ms();
    printf("[inject] %s (interval=%ums pulse=%ums)\n",
           on ? "STARTED" : "STOPPED",
           (unsigned)interval_ms, (unsigned)pulse_ms);
}

void xbone_inject_set_interval_ms(uint32_t ms)
{
    if (ms < XBONE_INJECT_MIN_INTERVAL_MS) ms = XBONE_INJECT_MIN_INTERVAL_MS;
    if (ms > XBONE_INJECT_MAX_INTERVAL_MS) ms = XBONE_INJECT_MAX_INTERVAL_MS;
    if (ms == interval_ms) return;
    interval_ms = ms;
    cycle_start_ms = platform_time_ms();
    printf("[inject] interval=%ums\n", (unsigned)interval_ms);
}

void xbone_inject_set_pulse_ms(uint32_t ms)
{
    if (ms == 0 || ms > interval_ms) return;
    pulse_ms = ms;
    printf("[inject] pulse=%ums\n", (unsigned)pulse_ms);
}
