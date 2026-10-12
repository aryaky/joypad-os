// xbone_auth.c - Xbox One Auth Dongle Host Driver
// SPDX-License-Identifier: MIT
// Based on GP2040-CE implementation (gp2040-ce.info)

#include "xbone_auth.h"
#include "xbone_inject.h"
#include "usb/usbd/drivers/xgip_protocol.h"
#include "usb/usbd/drivers/tud_xbone.h"
#include "gip_button_map.h"
#include "platform/platform.h"
#include "tusb.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define REPORT_QUEUE_SIZE      32
// ms between host-port sends. A chunked upload bursts far faster than this, so
// a slow drain meant ACKs were dropped and the controller resent the same window
// of chunks forever (observed: stuck resending from logical offset 430).
// 3 ms was too aggressive: the pad's OUT transfers take longer than that to
// retire, so most attempts hit "endpoint busy" and the ACK that mattered got
// pushed back behind retries.
#define REPORT_QUEUE_INTERVAL  8

// Power-on and rumble commands for dongle initialization
static const uint8_t xb1_power_on[] = {
    0x06, 0x62, 0x45, 0xb8, 0x77, 0x26, 0x2c, 0x55,
    0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f
};
static const uint8_t xb1_power_on_single[] = { 0x00 };
static const uint8_t xb1_rumble_on[] = {
    0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xeb
};

// ============================================================================
// STATE
// ============================================================================

typedef struct {
    uint8_t report[64];
    uint16_t len;
} report_queue_item_t;

static uint8_t xbone_dev_addr = 0;
static uint8_t xbone_instance = 0;
static bool dongle_ready = false;

// Handshake kick: some controllers (PDP and other licensed pads) never send
// the GIP announce on their own when attached to a non-Xbox USB host. Waiting
// for an announce that never comes deadlocks the handshake and leaves
// dongle_ready false forever, so we ask for the device descriptor ourselves.
#define DESCRIPTOR_KICK_MAX 8
static uint32_t dongle_registered_ms = 0;
static uint8_t descriptor_kicks = 0;

// v35: the pad DOES send a well-formed GIP announce here (cmd 0x02 with the
// 28-byte gip_pkt_announce payload) - what it never sends is the identify /
// descriptor reply (cmd 0x04). Because dongle_ready was only ever set by 0x04,
// the handshake fell through to the "assume already powered" fallback and ran
// one step behind the pad from then on. Treat the announce itself as progress.
static bool announce_seen = false;
static uint32_t announce_seen_ms = 0;
#define ANNOUNCE_DESCRIPTOR_TIMEOUT_MS 1500

// v35: xone (vendor/xone, gip_auth_send_complete) proves the *host* owes the
// device a GIP_AUTH control packet - context CONTROL (0x01), control COMPLETE
// (0x00), wire bytes "06 20 <seq> 02 01 00" - once the exchange is over. The
// console never sends it (it only sends 01 01 / 01 02), so nobody ever told
// the pad the handshake was finished and it never started emitting input.
#define GIP_AUTH_CTX_CONTROL    0x01
#define GIP_AUTH_CTRL_COMPLETE  0x00
#define AUTH_COMPLETE_MAX       4
#define AUTH_COMPLETE_INTERVAL  2000
static bool console_exchange_done = false;
static uint8_t auth_complete_seq = 1;
static uint8_t auth_complete_sends = 0;
static uint32_t last_auth_complete_ms = 0;

// Safety net: a wired GIP pad that has been authenticated by the console still
// has to be told to start streaming input reports. If we never see a single
// GIP_INPUT_REPORT after the handshake, re-issue the power-on sequence.
#define INPUT_NUDGE_IDLE_MS   3000
#define INPUT_NUDGE_INTERVAL  3000
#define INPUT_NUDGE_MAX       5

// Escalation: if the pad still has not spoken after every power-on retry, it is
// stuck in the auth state machine. Re-plugging is what clears it in practice,
// but VBUS is hardwired on this board so we cannot cycle power. Dropping off
// the console bus for a moment restarts the console-side handshake with the pad
// already attached - the same effect without the user touching anything.
#define HANDSHAKE_RESTART_MAX    3
#define HANDSHAKE_RESTART_WAIT   8000
static uint32_t last_handshake_restart_ms = 0;
static uint8_t handshake_restarts = 0;

static uint32_t last_input_report_ms = 0;

// 转发给主机的输入报告的日志状态。输入报告是 250 Hz，逐包打印会淹掉串口也会
// 拖慢主循环（v3x 上试过，循环直接被拖住），所以"变化即打 + 每秒一条摘要"。
static uint16_t relay_last_buttons = 0xFFFF;
static uint16_t relay_last_lt      = 0xFFFF;
static uint32_t relay_last_log_ms;
static uint32_t relay_input_count;
static uint32_t relay_fail_tick;

// 最近一次真的从手柄收到输入报告的时刻。设备侧靠它判断"手柄这条流还活着吗" ——
// 手柄沉默时不能干等，注入得由我们自己把报告流撑起来。
static uint32_t relay_last_rx_ms;
static uint32_t last_input_nudge_ms = 0;
static uint8_t input_nudges = 0;

static xgip_t incoming_xgip;
static xgip_t outgoing_xgip;

// Last auth packet relayed to the console, used to swallow retransmissions
static uint8_t last_relayed[64];
static uint16_t last_relayed_len = 0;

// Implemented by the Xbox One device driver; weak stub keeps other apps linking
__attribute__((weak))
bool tud_xbone_send_raw(uint8_t const* data, uint16_t len)
{
    (void)data; (void)len;
    return false;
}

// Report queue
static report_queue_item_t report_queue[REPORT_QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;
static uint32_t last_report_queue_sent = 0;
static uint8_t consecutive_send_failures = 0;

// ACKs get their own lane, ahead of everything else in the queue.
//
// The pad replays one window of chunks while it waits to be acknowledged, and it
// never leaves the auth state until an ACK lands. Anything else we queue -
// console passthrough, the power-on sequence, AUTH COMPLETE - can wait; the ACK
// cannot. Behind the main FIFO it does wait, and v35 showed what that costs: the
// pad spoke 16 times all session, our side logged ten failures of exactly that
// 13-byte report, and no input ever flowed. See the ack-starved-behind-queue
// entry in tests/fixtures/gip_sessions/problem_ledger.json.
//
// It also removes the second way an ACK used to vanish: the main queue only
// rescued one if it could find an earlier ACK to overwrite, so a full queue of
// ordinary traffic discarded it with no message at all.
#define ACK_LANE_SIZE 4
static report_queue_item_t ack_lane[ACK_LANE_SIZE];
static uint8_t ack_lane_count = 0;

// ACKs are idempotent - only the newest one matters, because an older one
// answers a window the pad has already replayed - so a full lane discards the
// oldest rather than the one that just arrived.
static void queue_ack(uint8_t* report, uint16_t len)
{
    if (ack_lane_count >= ACK_LANE_SIZE) {
        memmove(&ack_lane[0], &ack_lane[1],
                sizeof(ack_lane[0]) * (ACK_LANE_SIZE - 1));
        ack_lane_count = ACK_LANE_SIZE - 1;
    }
    memcpy(ack_lane[ack_lane_count].report, report, len);
    ack_lane[ack_lane_count].len = len;
    ack_lane_count++;
}

static bool take_ack(uint8_t* out, uint16_t* len)
{
    if (ack_lane_count == 0) {
        return false;
    }
    *len = ack_lane[0].len;
    memcpy(out, ack_lane[0].report, ack_lane[0].len);
    return true;
}

static void drop_head_ack(void)
{
    memmove(&ack_lane[0], &ack_lane[1], sizeof(ack_lane[0]) * (ACK_LANE_SIZE - 1));
    ack_lane_count--;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void queue_host_report(uint8_t* report, uint16_t len)
{
    // An ACK never waits behind anything. It has its own lane (see above), which
    // is what keeps a pad waiting to be acknowledged from being starved by
    // whatever console traffic happens to be queued at the time.
    if (report != NULL && report[0] == GIP_ACK_RESPONSE) {
        queue_ack(report, len);
        return;
    }

    if (queue_count >= REPORT_QUEUE_SIZE) {
        printf("[xbone_auth] host queue full - dropping cmd=0x%02x\n", report ? report[0] : 0);
        return;
    }

    memcpy(report_queue[queue_tail].report, report, len);
    report_queue[queue_tail].len = len;

    queue_tail = (queue_tail + 1) % REPORT_QUEUE_SIZE;
    queue_count++;
}

// Forward declaration - implemented by xinput host or similar
extern bool tuh_xinput_send_report(uint8_t dev_addr, uint8_t instance,
                                   uint8_t const* report, uint16_t len);
extern bool tuh_xinput_abort_out(uint8_t dev_addr, uint8_t instance);

bool xbone_auth_send_raw_to_controller(uint8_t const* report, uint16_t len)
{
    if (len == 0 || len > sizeof(report_queue[0].report)) {
        return false;
    }
    if (queue_count >= REPORT_QUEUE_SIZE) {
        printf("[xbone_auth] raw passthrough queue full, dropping cmd=0x%02x\n", report[0]);
        return false;
    }
    // 只入队，还没发。措辞必须是 QUEUED：实际发送在 process_report_queue() 里异步
    // 进行且可能失败（端点忙），说 "to controller" 会让人以为已经到手柄了。
    printf("[xbone_auth] Raw passthrough QUEUED for controller: cmd=0x%02x len=%d\n",
           report[0], len);
    memcpy(report_queue[queue_tail].report, report, len);
    report_queue[queue_tail].len = len;
    queue_tail = (queue_tail + 1) % REPORT_QUEUE_SIZE;
    queue_count++;
    return true;
}

// Power-on sequence shared by the descriptor-reply path, the announce timeout
// and the input watchdog so all three stay identical.
static void send_dongle_power_on_sequence(void)
{
    xgip_reset(&outgoing_xgip);
    xgip_set_attributes(&outgoing_xgip, GIP_POWER_MODE_DEVICE_CONFIG, 2, 1, false, 0);
    xgip_set_data(&outgoing_xgip, xb1_power_on, sizeof(xb1_power_on));
    queue_host_report(xgip_generate_packet(&outgoing_xgip),
                      xgip_get_packet_length(&outgoing_xgip));

    xgip_reset(&outgoing_xgip);
    xgip_set_attributes(&outgoing_xgip, GIP_POWER_MODE_DEVICE_CONFIG, 3, 1, false, 0);
    xgip_set_data(&outgoing_xgip, xb1_power_on_single, sizeof(xb1_power_on_single));
    queue_host_report(xgip_generate_packet(&outgoing_xgip),
                      xgip_get_packet_length(&outgoing_xgip));

    xgip_reset(&outgoing_xgip);
    xgip_set_attributes(&outgoing_xgip, GIP_CMD_RUMBLE, 1, 0, false, 0);
    xgip_set_data(&outgoing_xgip, xb1_rumble_on, sizeof(xb1_rumble_on));
    queue_host_report(xgip_generate_packet(&outgoing_xgip),
                      xgip_get_packet_length(&outgoing_xgip));
}

// Send the GIP_AUTH control packet that tells the pad the handshake is over.
// Wire format (see vendor/xone gip_auth_send_complete):
//   06 20 <seq> 02 01 00
//   06    GIP_AUTH
//   20    options = GIP_OPT_INTERNAL
//   <seq> sequence
//   02    packet_length = 2 (7-bit varint)
//   01    context = GIP_AUTH_CTX_CONTROL
//   00    control = GIP_AUTH_CTRL_COMPLETE
void xbone_auth_send_auth_complete(void)
{
    if (auth_complete_sends >= AUTH_COMPLETE_MAX) {
        return;
    }

    // No room means this packet cannot go out at all: xbone_auth_send_raw_to_controller()
    // drops it and returns, having sent nothing. Counting the attempt anyway spends a
    // retry on nothing, which is how serial36.log ended up with four attempts, three
    // drops, and no fifth send once the queue drained - the budget was already gone.
    if (queue_count >= REPORT_QUEUE_SIZE) {
        printf("[xbone_auth] AUTH COMPLETE held - queue full, %u/%u retries left\n",
               (unsigned)(AUTH_COMPLETE_MAX - auth_complete_sends),
               (unsigned)AUTH_COMPLETE_MAX);
        return;
    }

    uint8_t pkt[6] = {
        GIP_AUTH,
        0x20,
        auth_complete_seq++,
        0x02,
        GIP_AUTH_CTX_CONTROL,
        GIP_AUTH_CTRL_COMPLETE
    };

    auth_complete_sends++;
    printf("[xbone_auth] Sending AUTH COMPLETE (06 20 %02x 02 01 00) to controller (send %u/%u)\n",
           pkt[2], auth_complete_sends, AUTH_COMPLETE_MAX);
    xbone_auth_send_raw_to_controller(pkt, sizeof(pkt));
}

// Called by the USB device driver when the console reports the auth exchange is
// closed (auth marker 01 02). The console will not send COMPLETE on our behalf.
void xbone_auth_notify_console_exchange_done(void)
{
    console_exchange_done = true;
}

// A chunked auth upload arrives one packet every few milliseconds. Every printf
// blocks on UART, and that stall is what made the host port report "endpoint
// busy" (41 times in one session) and ACK too late, so the controller restarted
// the upload over and over. Only log 1 in 16 of the high-frequency packets.
static uint16_t auth_log_tick = 0;
// Which (progress, chunk counter) pairs have already been reported for this
// sequence.
//
// Comparing each arriving chunk against only its predecessor is defeated by any
// rotation longer than one entry: a pad wedged at got=696/1024 cycled through
// 430, 488, 674, 732, 14848, 29696 - every one different from the one before -
// and so printed all of them forever, 1751 lines in one session. What matters is
// whether that exact pair has been said already, not what came in last.
#define LOGGED_CHUNK_MAX 64

static uint32_t last_auth_chunk_ms = 0;
static bool     seen_auth_chunk = false;

typedef struct {
    uint16_t progress;
    uint16_t counter;
} logged_chunk_t;

static logged_chunk_t logged_chunks[LOGGED_CHUNK_MAX];
static uint8_t        logged_chunk_count = 0;
static uint8_t        logged_chunk_seq = 0xFF;

static bool chunk_already_logged(uint8_t seq, uint16_t progress, uint16_t counter)
{
    // A new sequence is a new upload, so previous pairs say nothing about it.
    if (seq != logged_chunk_seq) {
        logged_chunk_seq = seq;
        logged_chunk_count = 0;
    }
    for (uint8_t i = 0; i < logged_chunk_count; i++) {
        if (logged_chunks[i].progress == progress &&
            logged_chunks[i].counter == counter) {
            return true;
        }
    }
    if (logged_chunk_count < LOGGED_CHUNK_MAX) {
        logged_chunks[logged_chunk_count].progress = progress;
        logged_chunks[logged_chunk_count].counter = counter;
        logged_chunk_count++;
    }
    return false;
}

// True while the pad is streaming a chunked upload. Callers that log per packet
// use it to stay quiet for the duration: the second print site, in
// xinput_host.c, fired 1789 times in serial36.log because it never checked.
#define UPLOAD_IDLE_MS 1000

bool xbone_auth_upload_in_flight(void)
{
    if (!seen_auth_chunk) {
        return false;
    }
    return (platform_time_ms() - last_auth_chunk_ms) < UPLOAD_IDLE_MS;
}

static uint32_t last_auth_progress_ms = 0;
static uint16_t last_auth_progress_bytes = 0;

// Logical byte counts are what keep the console happy, and they work for most
// streams. But this pad also gets stuck retransmitting one window of chunks
// forever while we report the same count, so if a stream makes no progress at
// all we try acknowledging it in its own wire accounting and see if that moves
// it. Flipped back off when the stream is done.
static bool ack_wire_currency = false;

// Highest byte count we have reported to the controller for the current upload.
// Never answer a replayed chunk with anything lower than what we already
// claimed: the controller conservatively resends from the last figure it trusts.
static uint16_t ack_high_water = 0;

// Per-sequence record of the uploads we already reassembled and forwarded.
// The controller keeps resending an upload until it is satisfied, so a
// retransmission has to be acknowledged for the full amount we hold - but it
// must never be forwarded a second time. Forwarding them again (v26: seq=3 was
// forwarded 21 times, the later copies padded to 1012 bytes by duplicates)
// hands the console a payload that is no longer the signature.
static bool     forwarded_seq[256];
static uint16_t forwarded_seq_len[256];

// How long a chunked upload may go without taking in a single new byte before
// we consider it stuck and try the other accounting.
#define AUTH_STALL_MS 800

// If a chunked response goes quiet for this long, take what we have as the
// complete response. Streams we join mid-way have no declared total, and the
// controller only sends its end-of-chunk marker after every ACK lands - one
// late ACK and it never does, leaving us holding a response we never forward
// (the console then retransmits the same challenge forever).
#define AUTH_STREAM_IDLE_MS 200

static void forward_auth_response(void);

static bool send_to_dongle(uint8_t* report, uint16_t len)
{
    if (xbone_dev_addr == 0) {
        printf("[xbone_auth] send_to_dongle: no controller registered!\n");
        return false;
    }
    bool result = tuh_xinput_send_report(xbone_dev_addr, xbone_instance, report, len);
    if (!result) {
        printf("[xbone_auth] send_to_dongle FAILED: dev=%d inst=%d len=%d\n",
               xbone_dev_addr, xbone_instance, len);
    }
    return result;
}

// ============================================================================
// PUBLIC API
// ============================================================================

uint32_t xbone_auth_last_pad_relay_ms(void)
{
    return relay_last_rx_ms;
}

void xbone_auth_init(void)
{
    xgip_init(&incoming_xgip);
    xgip_init(&outgoing_xgip);

    // 注入引擎在这里挂上来：它活在"手柄 -> 主机"的转发路径上，而转发就在这个
    // 模块里做，所以初始化跟它绑在一起。
    xbone_inject_init();

    xbone_dev_addr = 0;
    xbone_instance = 0;
    dongle_ready = false;

    announce_seen = false;
    announce_seen_ms = 0;
    console_exchange_done = false;
    auth_complete_sends = 0;
    auth_complete_seq = 1;
    last_auth_complete_ms = 0;

    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    ack_lane_count = 0;
    consecutive_send_failures = 0;
    last_report_queue_sent = 0;
    last_auth_chunk_ms = 0;
    seen_auth_chunk = false;
    logged_chunk_count = 0;
    logged_chunk_seq = 0xFF;
}

bool xbone_auth_is_available(void)
{
    return dongle_ready && xbone_dev_addr != 0;
}

bool xbone_auth_is_complete(void)
{
    return xbone_auth_is_completed();
}

// OUT 端点连续忙碌的起点（ms），0 = 当前不忙。
//
// 为什么不能只靠 consecutive_send_failures：它一成功就清零，而实测的失败形态是零星
// 穿插的 —— v39 那轮 15 次发送失败里，计数只到过 1（10 次）和 2（5 次），一次都没
// 够到下面那条 8 连失败的兜底，abort 全程触发 0 次。按时间窗判定才能覆盖这种形态。
static uint32_t out_ep_busy_since_ms = 0;

// 端点连续忙多久就认定那笔传输卡住了，去取消它。正常 OUT 传输远快于这个量级。
#define OUT_EP_BUSY_ABORT_MS 500

static void process_report_queue(void)
{
    uint32_t now = platform_time_ms();

    // Hammering a busy OUT endpoint makes "endpoint busy" self-sustaining: the
    // retry storm keeps the endpoint claimed so the next attempt fails too.
    // Back off hard once we start losing reports.
    uint32_t interval = REPORT_QUEUE_INTERVAL;
    if (consecutive_send_failures >= 3) {
        interval = REPORT_QUEUE_INTERVAL * 8;
    } else if (consecutive_send_failures > 0) {
        interval = REPORT_QUEUE_INTERVAL * 2;
    }

    if (ack_lane_count == 0 && queue_count == 0) {
        return;
    }
    if ((now - last_report_queue_sent) <= interval) {
        return;
    }

    uint8_t report[64];
    uint16_t len = 0;

    // An ACK is served before anything else: it is what lets the pad stop
    // replaying its upload and move on to input reports.
    bool is_ack = take_ack(report, &len);
    if (!is_ack) {
        len = report_queue[queue_head].len;
        memcpy(report, report_queue[queue_head].report, len);
    }

    if (send_to_dongle(report, len)) {
        consecutive_send_failures = 0;
        out_ep_busy_since_ms = 0;
        last_report_queue_sent = now;
        if (is_ack) {
            drop_head_ack();
        } else {
            queue_head = (queue_head + 1) % REPORT_QUEUE_SIZE;
            queue_count--;
        }
        return;
    }

    consecutive_send_failures++;
    printf("[xbone_auth] Failed to send report to controller (%u in a row)\n",
           consecutive_send_failures);

    // 时间窗兜底：只取消那笔卡住的传输，**不动队列头**。
    //
    // 丢队列头才是让手柄反复重放同一个 825 字节窗口的原因，所以这里绝不丢 —— 队头
    // 留在原地，下一轮照常重试。下面那条 8 连失败的兜底才负责丢弃，且只对非 ACK
    // 生效。两者职责分开，避免把"取消传输"和"放弃这条报告"混为一谈。
    if (out_ep_busy_since_ms == 0) {
        out_ep_busy_since_ms = now;
    } else if ((now - out_ep_busy_since_ms) >= OUT_EP_BUSY_ABORT_MS) {
        printf("[xbone_auth] OUT endpoint busy %ums - cancelling stuck transfer (head kept)\n",
               (unsigned)(now - out_ep_busy_since_ms));
        tuh_xinput_abort_out(xbone_dev_addr, xbone_instance);
        out_ep_busy_since_ms = 0;
    }

    // A stuck OUT transfer keeps usbh_edpt_claim() busy forever, and the
    // head of the queue then blocks every other report behind it, which stalls
    // the controller's chunked upload and no input ever flows. Give up on it:
    // cancel the transfer and let the queue move on.
    //
    // Never, however, give up on an ACK. Abandoning one is precisely what made
    // the pad replay the same 825-byte window forever, so it stays pending and
    // is retried until it lands.
    if (!is_ack && consecutive_send_failures >= 8) {
        tuh_xinput_abort_out(xbone_dev_addr, xbone_instance);
        queue_head = (queue_head + 1) % REPORT_QUEUE_SIZE;
        queue_count--;
        consecutive_send_failures = 0;
    }
    platform_sleep_ms(REPORT_QUEUE_INTERVAL);
}

void xbone_auth_task(void)
{
    // Forward auth from console to controller
    xbone_auth_state_t state = xbone_auth_get_state();

    // Drain the queue FIRST and unconditionally. The descriptor kicks below are
    // queued while dongle_ready is still false, so draining after the early
    // return would strand them forever and the controller would never see them.
    process_report_queue();

    // The upload-stall escalation that used to flip us into the controller's
    // wire accounting is gone on purpose. Two facts rule it out:
    //   - the console acknowledges US in plain logical bytes (recv=58 rem=32,
    //     then recv=90 rem=0), so that is the convention on this link;
    //   - in wire currency the ACK is derived from whichever chunk arrived
    //     last. The controller replays a whole window of chunks while it waits,
    //     so our ACK bounced (732 / 918 / 976 / 1162) and could never converge -
    //     we ended up reporting 732 while holding 928 bytes.
    // Reporting real logical progress is what the one working session did.

    // A chunked response that stopped arriving is finished as far as we're
    // concerned - see AUTH_STREAM_IDLE_MS.
    if (xgip_is_chunked(&incoming_xgip) && incoming_xgip.total_chunk_length == 0 &&
        incoming_xgip.data != NULL && incoming_xgip.actual_data_received > 0 &&
        !xgip_end_of_chunk(&incoming_xgip) &&
        !forwarded_seq[xgip_get_sequence(&incoming_xgip)] &&
        (platform_time_ms() - last_auth_chunk_ms) > AUTH_STREAM_IDLE_MS) {
        printf("[xbone_auth] Auth stream idle - completing with %u bytes\n",
               (unsigned)incoming_xgip.actual_data_received);
        xgip_force_end_of_chunk(&incoming_xgip);
        forward_auth_response();
    }

    // dongle_ready is set ONLY after the real announce → descriptor → POWER_ON
    // handshake completes (see xbone_auth_report_received).
    //
    // Controllers that never volunteer an announce (observed on PDP/wired pads
    // on a non-Xbox host) would otherwise sit here forever, so nudge the
    // handshake by requesting the GIP device descriptor ourselves, a few times.
    if (!dongle_ready && xbone_dev_addr != 0) {
        uint32_t now_ms = platform_time_ms();
        if (announce_seen) {
            // The pad announced, so the old "no announce seen" kick budget does
            // not apply. It just never answers IDENTIFY on this host, so give it
            // a short window and then power it on ourselves.
            if (now_ms - announce_seen_ms > ANNOUNCE_DESCRIPTOR_TIMEOUT_MS) {
                printf("[xbone_auth] ANNOUNCE seen but no descriptor reply - powering pad on\n");
                send_dongle_power_on_sequence();
                dongle_ready = true;
            }
        } else if (descriptor_kicks < DESCRIPTOR_KICK_MAX) {
            if (now_ms - dongle_registered_ms > (400U + (uint32_t)descriptor_kicks * 700U)) {
                descriptor_kicks++;
                printf("[xbone_auth] No announce seen - requesting GIP device descriptor (try %u)\n",
                       descriptor_kicks);
                xgip_reset(&outgoing_xgip);
                xgip_set_attributes(&outgoing_xgip, GIP_DEVICE_DESCRIPTOR, 1, 1, false, 0);
                queue_host_report(xgip_generate_packet(&outgoing_xgip),
                                  xgip_get_packet_length(&outgoing_xgip));
            }
        } else if (now_ms - dongle_registered_ms >
                   (400U + (uint32_t)DESCRIPTOR_KICK_MAX * 700U + 2000U)) {
            // Wired pads are already powered/configured by xboxone_init and simply
            // never speak ANNOUNCE on a non-Xbox host. Stop waiting for it and let
            // the console auth challenge through, otherwise the handshake never
            // starts at all.
            printf("[xbone_auth] No ANNOUNCE/descriptor reply - assuming wired pad already powered, proceeding\n");
            dongle_ready = true;
        }
    }

    if (!dongle_ready) {
        // Throttle the warning to once per second so we don't flood the log
        if (state == XBONE_AUTH_SEND_CONSOLE_TO_DONGLE) {
            static uint32_t last_warn_ms = 0;
            uint32_t now_ms = platform_time_ms();
            if (now_ms - last_warn_ms > 1000) {
                printf("[xbone_auth] WARNING: Auth request pending but dongle not initialized yet (dev_addr=%d)\n",
                       xbone_dev_addr);
                last_warn_ms = now_ms;
            }
        }
        return;
    }

    // Input watchdog: no input reports at all → re-issue power-on to the pad.
    if (dongle_ready && xbone_dev_addr != 0 &&
        input_nudges < INPUT_NUDGE_MAX &&
        (descriptor_kicks >= DESCRIPTOR_KICK_MAX || announce_seen)) {
        uint32_t now_ms = platform_time_ms();
        if ((now_ms - last_input_report_ms) > INPUT_NUDGE_IDLE_MS &&
            (now_ms - last_input_nudge_ms) > INPUT_NUDGE_INTERVAL) {
            input_nudges++;
            last_input_nudge_ms = now_ms;
            printf("[xbone_auth] No input reports seen - re-sending power-on to controller (try %u)\n",
                   input_nudges);
            send_dongle_power_on_sequence();
        }
    }

    // v35: the console closed the auth exchange but the pad still emits no input.
    // xone shows the host owes the pad a GIP_AUTH COMPLETE control packet at this
    // point, so keep re-issuing it (capped) until input actually shows up.
    if (dongle_ready && xbone_dev_addr != 0 &&
        console_exchange_done &&
        auth_complete_sends < AUTH_COMPLETE_MAX) {
        uint32_t now_ms = platform_time_ms();
        if ((now_ms - last_input_report_ms) > 1000U &&
            (now_ms - last_auth_complete_ms) > AUTH_COMPLETE_INTERVAL) {
            last_auth_complete_ms = now_ms;
            xbone_auth_send_auth_complete();
        }
    }

    // Every power-on retry has been spent and the pad is still silent. Restart
    // the console-side handshake so the pad gets challenged again while it is
    // already attached and already authenticated.
    if (dongle_ready && xbone_dev_addr != 0 &&
        input_nudges >= INPUT_NUDGE_MAX &&
        handshake_restarts < HANDSHAKE_RESTART_MAX) {
        uint32_t now_ms = platform_time_ms();
        if ((now_ms - last_input_report_ms) > INPUT_NUDGE_IDLE_MS &&
            (now_ms - last_handshake_restart_ms) > HANDSHAKE_RESTART_WAIT) {
            handshake_restarts++;
            last_handshake_restart_ms = now_ms;
            printf("[xbone_auth] Still no input after %u power-on retries - restarting console handshake (%u/%u)\n",
                   input_nudges, handshake_restarts, HANDSHAKE_RESTART_MAX);
            // The console runs a fresh exchange, so arm the COMPLETE sender again.
            console_exchange_done = false;
            auth_complete_sends = 0;
            auth_complete_seq = 1;
            tud_xbone_request_reenumeration();
        }
    }

    if (state == XBONE_AUTH_SEND_CONSOLE_TO_DONGLE) {
        printf("[xbone_auth] Forwarding auth challenge to controller: type=0x%02x len=%d seq=%d\n",
               xbone_auth_get_type(), xbone_auth_get_length(), xbone_auth_get_sequence());

        uint8_t is_chunked = (xbone_auth_get_length() > GIP_MAX_CHUNK_SIZE);
        uint8_t needs_ack = (xbone_auth_get_length() > 2);

        xgip_reset(&outgoing_xgip);
        xgip_set_attributes(&outgoing_xgip, xbone_auth_get_type(),
                           xbone_auth_get_sequence(), 1, is_chunked, needs_ack);
        xgip_set_data(&outgoing_xgip, xbone_auth_get_buffer(), xbone_auth_get_length());

        // Update state to waiting
        xbone_auth_set_data(xbone_auth_get_buffer(), xbone_auth_get_length(),
                           xbone_auth_get_sequence(), xbone_auth_get_type(),
                           XBONE_AUTH_WAIT_CONSOLE_TO_DONGLE);

    } else if (state == XBONE_AUTH_WAIT_CONSOLE_TO_DONGLE) {
        uint8_t* packet = xgip_generate_packet(&outgoing_xgip);
        uint8_t packet_len = xgip_get_packet_length(&outgoing_xgip);
        printf("[xbone_auth] Sending auth packet to controller: len=%d\n", packet_len);
        queue_host_report(packet, packet_len);

        if (!xgip_is_chunked(&outgoing_xgip) || xgip_end_of_chunk(&outgoing_xgip)) {
            printf("[xbone_auth] Auth challenge sent, waiting for controller response\n");
            xbone_auth_set_data(xbone_auth_get_buffer(), xbone_auth_get_length(),
                               xbone_auth_get_sequence(), xbone_auth_get_type(),
                               XBONE_AUTH_IDLE);
        }
    }

    // Flush anything the state machine above queued this tick
    process_report_queue();
}

void xbone_auth_register(uint8_t dev_addr, uint8_t instance)
{
    printf("[xbone_auth] Registering Xbox One controller for auth: dev_addr=%d, instance=%d\n", dev_addr, instance);
    xbone_dev_addr = dev_addr;
    xbone_instance = instance;

    dongle_registered_ms = platform_time_ms();
    descriptor_kicks = 0;
    announce_seen = false;
    announce_seen_ms = 0;
    console_exchange_done = false;
    auth_complete_sends = 0;
    auth_complete_seq = 1;
    last_auth_complete_ms = 0;
    last_input_report_ms = platform_time_ms();
    last_input_nudge_ms = 0;
    input_nudges = 0;
    last_handshake_restart_ms = 0;
    handshake_restarts = 0;

    xgip_reset(&incoming_xgip);
    xgip_reset(&outgoing_xgip);

    // Ready only after the real announce → descriptor → POWER_ON handshake
    // completes (matches GP2040-CE). The source drives this at its own pace.
    dongle_ready = false;
    printf("[xbone_auth] Controller mounted, waiting for ANNOUNCE handshake...\n");

    // If the console already started (or finished) the exchange, this
    // controller missed it and will never be challenged. Restart the
    // console-side handshake so it gets a turn.
    if (tud_xbone_handshake_started()) {
        tud_xbone_request_reenumeration();
    }
}

void xbone_auth_unregister(uint8_t dev_addr)
{
    if (xbone_dev_addr == dev_addr) {
        printf("[xbone_auth] Unregistering dongle: dev_addr=%d\n", dev_addr);
        // Don't reset dongle_ready - Magic-X may remount but still be ready
        xbone_dev_addr = 0;
        xbone_instance = 0;
    }
}

void xbone_auth_xmount(uint8_t dev_addr, uint8_t instance,
                       uint8_t controller_type, uint8_t subtype)
{
    // Check if this looks like an Xbox One auth dongle
    // Controller type 0xFF with certain subtypes indicates auth dongle
    xbone_auth_register(dev_addr, instance);
}

// Hand the reassembled controller response to the device driver, which
// re-chunks it for the console.
static void forward_auth_response(void)
{
    uint16_t plen = xgip_is_chunked(&incoming_xgip)
                        ? (uint16_t)incoming_xgip.actual_data_received
                        : (uint16_t)xgip_get_data_length(&incoming_xgip);

    if (plen == 0) {
        // Stray end-of-chunk terminator with no payload. Forwarding an empty
        // auth response made the console drop the pad (guide LED came on, then
        // went out), so swallow it.
        printf("[xbone_auth] Ignoring empty auth chunk from controller\n");
        xgip_reset(&incoming_xgip);
        return;
    }

    // The controller resends an upload it considers unacknowledged. We already
    // handed this sequence to the console, so acknowledge it again but keep it
    // to ourselves - a second copy would arrive padded with duplicate bytes.
    if (xgip_is_chunked(&incoming_xgip) && forwarded_seq[xgip_get_sequence(&incoming_xgip)]) {
        printf("[xbone_auth] Sequence %d already forwarded (%u bytes) - not resending\n",
               xgip_get_sequence(&incoming_xgip), plen);
        xgip_reset(&incoming_xgip);
        last_auth_progress_bytes = 0;
        return;
    }

    printf("[xbone_auth] Forwarding auth response to console: len=%d seq=%d (chunked=%d)\n",
           plen, xgip_get_sequence(&incoming_xgip), xgip_is_chunked(&incoming_xgip));

    // Remember this upload. The controller keeps resending the same window
    // until it is satisfied, and because the reassembly is reset here the fresh
    // stream only ever reached 580 of the 825 bytes, so every ACK
    // under-reported and the pad retransmitted forever - it never left the auth
    // state and never streamed input. Its retransmissions are answered from
    // this record, and are never forwarded a second time.
    if (xgip_is_chunked(&incoming_xgip)) {
        forwarded_seq[xgip_get_sequence(&incoming_xgip)] = true;
        forwarded_seq_len[xgip_get_sequence(&incoming_xgip)] = plen;
    }

    xbone_auth_set_data(xgip_get_data(&incoming_xgip),
                       plen,
                       xgip_get_sequence(&incoming_xgip),
                       xgip_get_command(&incoming_xgip),
                       XBONE_AUTH_SEND_DONGLE_TO_CONSOLE);
    xgip_reset(&incoming_xgip);

    // Next stream starts again in logical bytes.
    ack_wire_currency = false;
    ack_high_water = 0;
    last_auth_progress_bytes = 0;
}

void xbone_auth_report_received(uint8_t dev_addr, uint8_t instance,
                                uint8_t const* report, uint16_t len)
{
    bool throttled = ((auth_log_tick++ & 0x0F) != 0);
    bool burst_cmd = (report[0] == GIP_AUTH || report[0] == GIP_FINAL_AUTH ||
                      report[0] == GIP_ACK_RESPONSE || report[0] == 0x03);
    if (!throttled || !burst_cmd) {
        printf("[xbone_auth] Report received from controller: dev=%d inst=%d len=%d cmd=0x%02x bytes=%02x %02x %02x %02x %02x %02x %02x %02x\n",
               dev_addr, instance, len, report[0],
               report[0],
               len > 1 ? report[1] : 0,
               len > 2 ? report[2] : 0,
               len > 3 ? report[3] : 0,
               len > 4 ? report[4] : 0,
               len > 5 ? report[5] : 0,
               len > 6 ? report[6] : 0,
               len > 7 ? report[7] : 0);
    }

    if (dev_addr != xbone_dev_addr || instance != xbone_instance) {
        printf("[xbone_auth] Ignoring - not our registered controller\n");
        return;
    }

    xgip_parse(&incoming_xgip, report, len);

    if (!xgip_validate(&incoming_xgip)) {
        printf("[xbone_auth] Invalid packet, resetting\n");
        // First packet may be invalid, wait for dongle boot
        platform_sleep_ms(50);
        xgip_reset(&incoming_xgip);
        return;
    }

    // A chunk with counter 0 opens a new upload - including a retransmission of
    // one we already forwarded - so the high-water mark restarts with it.
    if (xgip_is_chunked(&incoming_xgip) && incoming_xgip.chunk_counter == 0) {
        ack_high_water = 0;
    }

    uint8_t cmd = xgip_get_command(&incoming_xgip);

    if (cmd == GIP_INPUT_REPORT) {
        last_input_report_ms = platform_time_ms();
    }

    // One compact line per *new* chunk counter. The controller cycles the same
    // window of chunks while it waits for an ACK, so deduping turns hundreds of
    // lines into ~15 - enough to see the whole stream without stalling the loop
    // the way full per-packet logging did.
    if (xgip_is_chunked(&incoming_xgip)) {
        last_auth_chunk_ms = platform_time_ms();
        seen_auth_chunk = true;
        if (incoming_xgip.actual_data_received != last_auth_progress_bytes) {
            last_auth_progress_bytes = incoming_xgip.actual_data_received;
            last_auth_progress_ms = last_auth_chunk_ms;
        }
        if (!chunk_already_logged(xgip_get_sequence(&incoming_xgip),
                                  incoming_xgip.actual_data_received,
                                  incoming_xgip.chunk_counter)) {
            printf("[xbone_auth] chunk wire=%u/%u got=%u/%u seq=%u dup=%d\n",
                   (unsigned)incoming_xgip.chunk_counter,
                   (unsigned)incoming_xgip.total_chunk_length,
                   (unsigned)incoming_xgip.actual_data_received,
                   (unsigned)incoming_xgip.data_length,
                   (unsigned)xgip_get_sequence(&incoming_xgip),
                   incoming_xgip.last_chunk_duplicate ? 1 : 0);
        }
    }

    // Send ACK if required by header, OR for any chunked GIP_AUTH response.
    // The MS Xbox Wireless controller (PID 0x0B12) stops sending mid-stream
    // unless we ACK every chunk during auth, even when its own needs_ack=0.
    bool force_ack = xgip_is_chunked(&incoming_xgip) &&
                     (cmd == GIP_AUTH || cmd == GIP_FINAL_AUTH);

    // Deliberately NOT answering a retransmission for the remembered length.
    //
    // Two real sessions say the remembered length is the wrong thing to report.
    // The one session that produced input ACKed 825 -> 883 -> 883, i.e. the ACK
    // tracked what we had actually reassembled as the stream grew. Every silent
    // session pinned the ACK at the remembered 825 while really holding 58, 116,
    // 174 ... bytes (serial31.log: "got=58/1024 ... ack=825"). The pad cannot
    // reconcile a constant ACK with the chunks it is still sending, so it restarts
    // the upload instead of finishing it, and shrinks by one chunk per attempt
    // (245 -> 187 -> 129 -> 71 -> 13).
    //
    // Reporting the remembered length was itself only ever reachable because
    // v26-v28 inflated actual_data_received past 825 (883, 1012), which disabled
    // this branch. Fixing the inflation (v29) switched the branch on and the pad
    // stopped leaving the auth state - so report real progress, which is what the
    // working session did.
    //
    // The override below stays wired up for the console-facing path, but a
    // retransmitted upload is never answered with a length we do not hold.

    if (xgip_ack_required(&incoming_xgip) || force_ack) {
        // Monotonicity guard. A duplicate chunk carries no new bytes, but the
        // controller cannot tell that we already counted those bytes - it only
        // sees the number we report. Answering with anything below our
        // high-water mark makes it resend from there forever.
        if (incoming_xgip.actual_data_received > ack_high_water) {
            ack_high_water = incoming_xgip.actual_data_received;
        }
        if (ack_high_water > incoming_xgip.actual_data_received) {
            incoming_xgip.ack_override_valid = true;
            incoming_xgip.ack_received_override = ack_high_water;
        }
        // Get "how much is left" right. A retransmitted upload arrives with no
        // total in its headers, so the stream length falls back to the buffer
        // size and we would claim 734 bytes outstanding on an 825-byte upload -
        // the controller then keeps resending bytes we already have. The length
        // we reassembled the first time round is the total it is working to.
        uint8_t fseq = xgip_get_sequence(&incoming_xgip);
        uint16_t saved_data_length = incoming_xgip.data_length;

        // 下面两处都是跨链路记账：拿另一端记住的长度 / 另一端的 ACK 计数器来改
        // 这一端要报的 ACK。它们只在"两边分片不一致"时才需要 —— 那是我们自己造
        // 出来的不一致。RELAY 模式下分片照抄，两个链路各算各的账，这些都不要。
        if (xbone_get_chunk_mode() == XBONE_CHUNK_RECHUNK) {
            if (forwarded_seq[fseq] && forwarded_seq_len[fseq] > incoming_xgip.data_length) {
                incoming_xgip.data_length = forwarded_seq_len[fseq];
            }
            incoming_xgip.ack_echo_peer_counter = ack_wire_currency;
        }
        uint8_t* ack = xgip_generate_ack(&incoming_xgip);
        incoming_xgip.data_length = saved_data_length;
        uint16_t ack_val = (uint16_t)(ack[7] | ((uint16_t)ack[8] << 8));
        if (!throttled) {
            printf("[xbone_auth] ACK to controller (forced=%d): got=%u/%u wire=%u/%u ack=%u\n",
                   force_ack && !xgip_ack_required(&incoming_xgip),
                   (unsigned)incoming_xgip.actual_data_received,
                   (unsigned)incoming_xgip.data_length,
                   (unsigned)incoming_xgip.chunk_counter,
                   (unsigned)incoming_xgip.total_chunk_length,
                   (unsigned)ack_val);
        }
        queue_host_report(ack, xgip_get_packet_length(&incoming_xgip));
    }

    switch (cmd) {
        case GIP_ANNOUNCE:
            // Dongle announced - request descriptor
            announce_seen = true;
            announce_seen_ms = platform_time_ms();
            printf("[xbone_auth] ANNOUNCE seen (cmd=0x02) - requesting GIP device descriptor\n");
            xgip_reset(&outgoing_xgip);
            xgip_set_attributes(&outgoing_xgip, GIP_DEVICE_DESCRIPTOR, 1, 1, false, 0);
            queue_host_report(xgip_generate_packet(&outgoing_xgip),
                             xgip_get_packet_length(&outgoing_xgip));
            break;

        case GIP_DEVICE_DESCRIPTOR:
            // Got descriptor - power on dongle
            if (xgip_end_of_chunk(&incoming_xgip) || !xgip_is_chunked(&incoming_xgip)) {
                send_dongle_power_on_sequence();
                dongle_ready = true;
                printf("[xbone_auth] Dongle ready!\n");
            }
            break;

        case GIP_AUTH:
        case GIP_FINAL_AUTH:
            // Byte-level passthrough: hand the controller's auth packet to the
            // console exactly as we received it and let the console do its own
            // chunk reassembly.
            //
            // Reassembling here requires knowing the chunked stream's total
            // length, which we only get from the chunk_start packet - and this
            // controller (wired PDP pad) regularly hands us a stream whose start
            // we never see, so we mis-derive the total, the ACK reports a byte
            // count the controller disagrees with, and it retransmits the same
            // chunks forever (observed: 58-byte and 13-byte chunks looping while
            // its own counters were 1522/1708 vs our 303/316). Relaying verbatim
            // makes all of that irrelevant.
            if (!throttled) {
                printf("[xbone_auth] Auth response from controller: cmd=0x%02x len=%d chunked=%d\n",
                       cmd, len, xgip_is_chunked(&incoming_xgip));
            }

            // 两条路，运行时可切：
            //
            // RELAY：逐包原样转发手柄的分片。手柄直插主机时用的就是这个分片大小，
            //   主机天然接受，所以两个链路的分片本来就该是一致的。
            //
            // RECHUNK：先重组整段，再按我们自己的分片发给主机（历史做法）。
            //
            // 注释里"试过直接转发原始分片、主机就不认了"那次实验，是在 ACK 记账
            // 错误、AUTH COMPLETE 还没合成等问题都还在的时候做的，结论被污染了。
            // 现在做成开关重做一次，而不是把那次结果当定论。
            if (xbone_get_chunk_mode() == XBONE_CHUNK_RELAY) {
                uint8_t pkt[64];
                uint16_t n = (len > sizeof(pkt)) ? (uint16_t)sizeof(pkt) : len;
                memcpy(pkt, report, n);
                bool sent = tud_xbone_send_raw(pkt, n);
                if (!throttled || !sent) {
                    printf("[xbone_auth] Auth chunk RELAY -> console: cmd=0x%02x len=%u wire=%u %s\n",
                           cmd, n, (unsigned)incoming_xgip.chunk_counter,
                           sent ? "sent" : "FAILED");
                }
                break;
            }

            // Reassemble the controller's chunked response, then hand the
            // payload to the device driver which re-chunks it for the console.
            if (!xgip_is_chunked(&incoming_xgip) || xgip_end_of_chunk(&incoming_xgip)) {
                forward_auth_response();
            }
            break;

        case GIP_INPUT_REPORT:
            // 主干：手柄的输入报告原样转发给主机。
            //
            // 这条链路上以前根本不存在转发。上游 joypad-os 的范式是"输入归一化
            // 再转换成输出协议"，没有"输入原样透传给输出"这条路 —— 所以手柄就算
            // 开口说话，数据也到不了主机。板子是中转器，缺的就是这一环。
            {
                uint8_t pkt[64];
                uint16_t pkt_len = (len > sizeof(pkt)) ? (uint16_t)sizeof(pkt) : len;
                memcpy(pkt, report, pkt_len);

                // 组合键用手柄自己的按键字判定，读的是改写前的原始值，所以注入
                // 出来的扳机不会被当成组合键输入。
                if (pkt_len >= 6) {
                    uint16_t raw_btns = (uint16_t)(pkt[4] | ((uint16_t)pkt[5] << 8));
                    xbone_inject_feed_buttons(
                        gip_button_map_apply(gip_button_map_current(), raw_btns),
                        platform_time_ms());
                }

                // 注入：只有"正在按下"的那段时间改写左扳机，其余字节一个都不动
                bool injected = xbone_inject_apply(pkt, pkt_len, platform_time_ms());

                bool sent = tud_xbone_send_raw(pkt, pkt_len);
                relay_input_count++;
                relay_last_rx_ms = platform_time_ms();

                uint16_t btns = (pkt_len >= 6)
                                    ? (uint16_t)(pkt[4] | ((uint16_t)pkt[5] << 8)) : 0;
                uint16_t lt   = (pkt_len >= 8)
                                    ? (uint16_t)(pkt[6] | ((uint16_t)pkt[7] << 8)) : 0;
                uint32_t now  = platform_time_ms();

                if (!sent) {
                    if ((relay_fail_tick++ & 0x3F) == 0) {
                        printf("[xbone_auth] Relay input FAILED: n=%u\n",
                               (unsigned)relay_input_count);
                    }
                } else if ((btns != relay_last_buttons) || (lt != relay_last_lt) ||
                           injected || (now - relay_last_log_ms) >= 1000u) {
                    printf("[xbone_auth] Relay input -> console: n=%u btns=0x%04x lt=%u%s\n",
                           (unsigned)relay_input_count, btns, lt,
                           injected ? " [INJECTED-LT]" : "");
                    relay_last_buttons = btns;
                    relay_last_lt = lt;
                    relay_last_log_ms = now;
                }
            }
            break;

        case GIP_ACK_RESPONSE:
            // 手柄发给我们的 ACK，是"我们发给它"的回执，主机那边的账不是同一本
            break;

        default:
            // 丢弃必须可见 —— 静默丢弃等于我们永远看不见自己漏了什么
            printf("[xbone_auth] Unhandled report from controller: cmd=0x%02x len=%d bytes=%02x %02x %02x %02x\n",
                   cmd, len,
                   len > 0 ? report[0] : 0, len > 1 ? report[1] : 0,
                   len > 2 ? report[2] : 0, len > 3 ? report[3] : 0);
            break;
    }
}

// Weak implementation if xinput_host doesn't provide this
__attribute__((weak))
bool tuh_xinput_send_report(uint8_t dev_addr, uint8_t instance,
                            uint8_t const* report, uint16_t len)
{
    (void)dev_addr;
    (void)instance;
    (void)report;
    (void)len;
    printf("[xbone_auth] ERROR: Using weak tuh_xinput_send_report stub!\n");
    return false;
}
