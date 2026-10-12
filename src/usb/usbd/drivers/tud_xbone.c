// tud_xbone.c - Xbox One TinyUSB device driver
// SPDX-License-Identifier: MIT
// Based on GP2040-CE implementation (gp2040-ce.info)

#include "tud_xbone.h"
#include "platform/platform.h"
#include <string.h>
#include <stdio.h>
#include "../tusb_compat.h"  // usbd_edpt_xfer is_isr shim (0.20.0 vs master)
#include "../../usbh/xbone_auth/xbone_auth.h"
#include "../../usbh/xbone_auth/xbone_inject.h"

// Forward declaration for auth passthrough check (weak - returns false if not linked)
__attribute__((weak)) bool xbone_auth_is_available(void) { return false; }

// Weak stubs so the driver still links when the auth / inject modules aren't
// part of the build.
__attribute__((weak)) uint32_t xbone_auth_last_pad_relay_ms(void) { return 0; }
__attribute__((weak)) bool xbone_inject_is_running(void) { return false; }
__attribute__((weak)) uint16_t xbone_inject_trigger_value(uint32_t now_ms)
{
    (void)now_ms;
    return 0;
}

// Weak stub so the driver still links when the auth module isn't part of the build
__attribute__((weak)) bool xbone_auth_send_raw_to_controller(uint8_t const* report, uint16_t len)
{
    (void)report; (void)len;
    return false;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

#define XBONE_OUT_SIZE         64
#define XBONE_ITF_COUNT        8
#define XBONE_TX_BUFSIZE       64
#define XBONE_RX_BUFSIZE       64

#define REPORT_QUEUE_SIZE      16
#define REPORT_QUEUE_INTERVAL  15    // ms between queued reports
#define ANNOUNCE_DELAY         500   // ms minimum before sending announce
#define ANNOUNCE_MAX_WAIT      5000  // ms maximum wait for auth controller
#define ACK_WAIT_TIMEOUT       2000  // ms to wait for ACK
#define IDLE_REPORT_INTERVAL   25    // ms between idle reports during auth
#define KEEPALIVE_INTERVAL     15000 // ms between keep-alive packets after auth

// 手柄沉默多久算"这条流断了"。真手柄是 250Hz，给 50ms 的余量足够宽松。
#define PAD_STREAM_IDLE_MS     50
// 手柄不说话时，我们自己发报告的节拍（和 auth 期间的 idle 报告一致）
#define INJECT_REPORT_INTERVAL IDLE_REPORT_INTERVAL

// Vendor request types
#define USB_SETUP_DEVICE_TO_HOST       0x80
#define USB_SETUP_HOST_TO_DEVICE       0x00
#define USB_SETUP_TYPE_VENDOR          0x40
#define USB_SETUP_RECIPIENT_DEVICE     0x00

#define REQ_GET_OS_FEATURE_DESCRIPTOR  0x20
#define DESC_EXTENDED_COMPATIBLE_ID    0x0004

// ----------------------------------------------------------------------------
// 两个运行时可切的协议策略（定义在文件前部：下面的转发分发要用）
// ----------------------------------------------------------------------------

static xbone_chunk_mode_t chunk_mode = XBONE_CHUNK_RELAY;
static xbone_desc_mode_t  desc_mode  = XBONE_DESC_SELF;

void xbone_set_chunk_mode(xbone_chunk_mode_t mode)
{
    chunk_mode = mode;
    printf("[tud_xbone] chunk mode = %s\n",
           mode == XBONE_CHUNK_RELAY ? "RELAY (verbatim)" : "RECHUNK");
}

xbone_chunk_mode_t xbone_get_chunk_mode(void)
{
    return chunk_mode;
}

void xbone_set_desc_mode(xbone_desc_mode_t mode)
{
    desc_mode = mode;
    printf("[tud_xbone] descriptor mode = %s\n",
           mode == XBONE_DESC_SELF ? "SELF (hardcoded)" : "FROM_PAD (relayed)");
}

xbone_desc_mode_t xbone_get_desc_mode(void)
{
    return desc_mode;
}

// ============================================================================
// STATE
// ============================================================================

typedef struct {
    uint8_t itf_num;
    uint8_t ep_in;
    uint8_t ep_out;
    CFG_TUSB_MEM_ALIGN uint8_t epin_buf[XBONE_TX_BUFSIZE];
    CFG_TUSB_MEM_ALIGN uint8_t epout_buf[XBONE_RX_BUFSIZE];
} xbone_interface_t;

typedef struct {
    uint8_t report[XBONE_ENDPOINT_SIZE];
    uint16_t len;
} report_queue_item_t;

// Interface tracking
static xbone_interface_t xbone_itf[XBONE_ITF_COUNT];

// Driver state
static xbone_driver_state_t driver_state = XBONE_STATE_IDLE;
static bool xbox_powered_on = false;
static bool waiting_ack = false;
static uint32_t waiting_ack_timeout = 0;

// Console-side attach observability.
//
// Why this exists: the device port had NO way to tell whether anything was even
// attached. There is no VBUS sense printed anywhere and no tud_mount_cb in the
// tree, so a session with zero "Console packet" lines and a session where the
// console simply stopped talking looked byte-for-byte identical in the log.
// That ambiguity cost a 15-minute live session: the board ran fine, printed a
// heartbeat every 2s, and we could not tell "Xbox not plugged in" from "Xbox
// plugged in but ignoring us".
//
// tud_mount_cb()/tud_umount_cb() below log the host-driven events, and the
// heartbeat reports tud_mounted()/tud_connected() plus how long it has been
// since the last packet. Together that separates all three cases: never
// attached / attached but silent / attached and talking.
static uint32_t console_rx_count = 0;
static uint32_t console_last_rx_ms = 0;

// How many input reports we have actually put on the wire to the console. The
// pad-side counter lives in the host driver, so with these two numbers a silent
// pad, a pad we fail to parse, and a pad we fail to forward stop looking alike.
static uint32_t input_reports_sent_to_console = 0;
static uint32_t timer_announce = 0;
static uint32_t last_report_queue_sent = 0;

// XGIP protocol handlers
static xgip_t outgoing_xgip;
static xgip_t incoming_xgip;

// Report queue (simple ring buffer)
static report_queue_item_t report_queue[REPORT_QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;

// Auth passthrough
static xbone_auth_t auth_data = { 0 };

// Auth ready marker
// The two auth markers the console actually sends. The old constant was
// {0x01, 0x00} - that is the payload of xboxone_pdp_auth[], the auth request
// *we* send the pad, pasted in as if it were the console's reply. The console
// never sends it: across 23 recorded sessions (45x 01 01, 15x 01 02, never
// 01 00) the comparison never once matched, so this branch was unreachable.
// 01 01 follows an accepted response, 01 02 closes the exchange - every
// session, including the ones the console rejected, so neither one proves the
// signature was accepted. auth_completed is therefore left alone; only the
// powered-on flag follows, because the console sends power-on right after.
#define AUTH_MARKER_RESPONSE_ACCEPTED  0x01
#define AUTH_MARKER_EXCHANGE_DONE      0x02

// Rolling sequence number for input reports + keep-alive (skips 0 on rollover)
static uint8_t input_report_sequence = 1;
static uint8_t keepalive_sequence = 1;
static uint32_t last_idle_report_ms = 0;
static uint32_t last_keepalive_ms = 0;

// 注入自己撑起报告流时的节拍与计数（只在手柄沉默的那条路上用）
static uint32_t last_inject_report_ms = 0;
static uint32_t inject_reports_sent = 0;

// Idle gamepad payload sent during auth — matches GP2040-CE xboneIdle (32 data bytes)
static const uint8_t xbone_idle_payload[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static uint8_t get_index_by_itfnum(uint8_t itf_num)
{
    for (uint8_t i = 0; i < XBONE_ITF_COUNT; i++) {
        if (itf_num == xbone_itf[i].itf_num) return i;
    }
    return 0xFF;
}

// Returns false when the packet was dropped. A drop used to be silent - the
// queue just returned - which made every "the console never saw it" look like a
// console-side problem. Throttled because the input path calls this at 250 Hz.
static bool queue_report(uint8_t* report, uint16_t len)
{
    if (queue_count >= REPORT_QUEUE_SIZE) {
        static uint32_t drop_tick = 0;
        if ((drop_tick++ & 0x3F) == 0) {
            printf("[tud_xbone] DROP: queue full, cmd=0x%02x len=%u dropped_so_far=%u\n",
                   report[0], (unsigned)len, (unsigned)drop_tick);
        }
        return false;
    }

    memcpy(report_queue[queue_tail].report, report, len);
    report_queue[queue_tail].len = len;

    queue_tail = (queue_tail + 1) % REPORT_QUEUE_SIZE;
    queue_count++;
    return true;
}

static bool dequeue_report(uint8_t* report, uint16_t* len)
{
    if (queue_count == 0) {
        return false;
    }

    memcpy(report, report_queue[queue_head].report, report_queue[queue_head].len);
    *len = report_queue[queue_head].len;

    queue_head = (queue_head + 1) % REPORT_QUEUE_SIZE;
    queue_count--;
    return true;
}

static void set_ack_wait(void)
{
    waiting_ack = true;
    waiting_ack_timeout = platform_time_ms();
}

static bool send_report_internal(uint8_t* report, uint16_t len)
{
    xbone_interface_t* p_xbone = NULL;

    for (uint8_t i = 0; i < XBONE_ITF_COUNT; i++) {
        if (xbone_itf[i].ep_in != 0) {
            p_xbone = &xbone_itf[i];
            break;
        }
    }

    if (p_xbone == NULL) {
        return false;
    }

    if (tud_ready() && (p_xbone->ep_in != 0) && (!usbd_edpt_busy(0, p_xbone->ep_in))) {
        usbd_edpt_claim(0, p_xbone->ep_in);
        bool ret = usbd_edpt_xfer(0, p_xbone->ep_in, report, len);
        usbd_edpt_release(0, p_xbone->ep_in);
        return ret;
    }

    return false;
}

// ============================================================================
// TINYUSB CLASS DRIVER CALLBACKS
// ============================================================================

static void xbone_init(void)
{
    xgip_init(&outgoing_xgip);
    xgip_init(&incoming_xgip);

    timer_announce = platform_time_ms();
    xbox_powered_on = false;

    // Clear queue
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;

    // Reset auth + sequence state so a re-enumeration starts clean
    memset(&auth_data, 0, sizeof(auth_data));
    input_report_sequence = 1;
    keepalive_sequence = 1;
    last_idle_report_ms = 0;
    last_keepalive_ms = 0;

    driver_state = XBONE_STATE_READY_ANNOUNCE;
    memset(&xbone_itf, 0, sizeof(xbone_itf));
}

static void xbone_reset(uint8_t rhport)
{
    (void)rhport;
    xbone_init();
}

static uint16_t xbone_open(uint8_t rhport, tusb_desc_interface_t const* itf_desc, uint16_t max_len)
{
    uint16_t drv_len = 0;

    if (itf_desc->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC) {
        return 0;
    }

    drv_len = sizeof(tusb_desc_interface_t) +
              (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));

    if (max_len < drv_len) {
        return 0;
    }

    // Find available interface slot
    xbone_interface_t* p_xbone = NULL;
    for (uint8_t i = 0; i < XBONE_ITF_COUNT; i++) {
        if (xbone_itf[i].ep_in == 0 && xbone_itf[i].ep_out == 0) {
            p_xbone = &xbone_itf[i];
            break;
        }
    }

    if (p_xbone == NULL) {
        return 0;
    }

    uint8_t const* p_desc = (uint8_t const*)itf_desc;

    // Check for Xbox One interface (subclass 0x47, protocol 0xD0)
    if (itf_desc->bInterfaceSubClass == 0x47 && itf_desc->bInterfaceProtocol == 0xD0) {
        p_desc = tu_desc_next(p_desc);
        TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, itf_desc->bNumEndpoints,
                                       TUSB_XFER_INTERRUPT, &p_xbone->ep_out, &p_xbone->ep_in), 0);

        p_xbone->itf_num = itf_desc->bInterfaceNumber;

        // Prepare OUT endpoint for receiving
        if (p_xbone->ep_out) {
            if (!usbd_edpt_xfer(rhport, p_xbone->ep_out, p_xbone->epout_buf, sizeof(p_xbone->epout_buf))) {
                TU_LOG1("XBONE: Failed to start OUT transfer\r\n");
            }
        }
    }

    return drv_len;
}

// Class-type / standard requests at INTERFACE recipient land here. Vendor-type
// requests are dispatched directly by TinyUSB to tud_vendor_control_xfer_cb,
// not here, so this can stay a no-op.
static bool xbone_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                  tusb_control_request_t const* request)
{
    (void)rhport; (void)stage; (void)request;
    return true;
}

static bool xbone_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void)result;

    xbone_interface_t* p_xbone = NULL;
    for (uint8_t i = 0; i < XBONE_ITF_COUNT; i++) {
        if (ep_addr == xbone_itf[i].ep_out || ep_addr == xbone_itf[i].ep_in) {
            p_xbone = &xbone_itf[i];
            break;
        }
    }

    if (p_xbone == NULL) {
        return false;
    }

    if (ep_addr == p_xbone->ep_out) {
        // Parse incoming packet
        xgip_parse(&incoming_xgip, p_xbone->epout_buf, xferred_bytes);

        // Send ACK if required
        if (xgip_ack_required(&incoming_xgip)) {
            queue_report(xgip_generate_ack(&incoming_xgip), xgip_get_packet_length(&incoming_xgip));
        }

        uint8_t cmd = xgip_get_command(&incoming_xgip);

        // Counting for the heartbeat. Without it, "there is no Console packet
        // line here" cannot be told apart from "there were hundreds a minute
        // ago and then it went quiet".
        console_rx_count++;
        console_last_rx_ms = platform_time_ms();

        printf("[tud_xbone] Console packet: cmd=0x%02x len=%u", cmd, (unsigned)xferred_bytes);
        if (cmd == GIP_ACK_RESPONSE) {
            // Ground truth: the console is a Microsoft GIP implementation, so
            // how it counts the bytes we send it tells us which accounting to
            // use when we ACK the controller.
            uint16_t ack_recv = (uint16_t)(p_xbone->epout_buf[7] | (p_xbone->epout_buf[8] << 8));
            uint16_t ack_rem = (uint16_t)(p_xbone->epout_buf[11] | (p_xbone->epout_buf[12] << 8));
            printf(" [console ACK: seq=%u recv=%u rem=%u]", p_xbone->epout_buf[2], ack_recv, ack_rem);
        } else {
            // ACKs are far too noisy to dump; everything else is rare enough
            // that seeing the payload pays for itself (e.g. is this 0x05 a
            // power-on or a power-off?).
            printf(" bytes=");
            for (uint16_t i = 0; i < xferred_bytes && i < 16; i++) {
                printf("%02x ", p_xbone->epout_buf[i]);
            }
        }
        printf("\n");

        if (cmd == GIP_ACK_RESPONSE) {
            waiting_ack = false;
        } else if (cmd == GIP_DEVICE_DESCRIPTOR) {
            // Console requested descriptor.
            //
            // SELF answers with our hardcoded descriptor; FROM_PAD forwards the
            // request to the pad so the console gets the pad's *real* one. The
            // pad is an ordinary Xbox pad, so its descriptor is exactly what the
            // console would have read from it directly - relaying it is the
            // honest answer for a pass-through box, and it removes the guess
            // that our hardcoded copy happens to match.
            if (desc_mode == XBONE_DESC_FROM_PAD) {
                bool fwd = xbone_auth_send_raw_to_controller(p_xbone->epout_buf,
                                                             (uint16_t)xferred_bytes);
                printf("[tud_xbone] Descriptor request -> controller: %s\n",
                       fwd ? "sent" : "FAILED");
            } else {
                printf("[tud_xbone] Descriptor request -> answering with hardcoded descriptor\n");
                xgip_reset(&outgoing_xgip);
                xgip_set_attributes(&outgoing_xgip, GIP_DEVICE_DESCRIPTOR,
                                   xgip_get_sequence(&incoming_xgip), 1, 1, 0);
                xgip_set_data(&outgoing_xgip, xbone_gip_descriptor, sizeof(xbone_gip_descriptor));
                driver_state = XBONE_STATE_SEND_DESCRIPTOR;
            }
        } else if (cmd == GIP_POWER_MODE_DEVICE_CONFIG || cmd == GIP_CMD_WAKEUP || cmd == GIP_CMD_RUMBLE) {
            xbox_powered_on = true;
            // The console is telling "us" (the virtual pad) to power on / rumble.
            // A wired GIP pad only starts emitting 0x20 input reports after it
            // receives these from its host, so pass them through untouched -
            // otherwise auth succeeds (guide LED solid) but no input ever flows.
            bool fwd = xbone_auth_send_raw_to_controller(p_xbone->epout_buf,
                                                         (uint16_t)xferred_bytes);
            printf("[tud_xbone] Passthrough -> controller: cmd=0x%02x len=%u %s\n",
                   cmd, (unsigned)xferred_bytes, fwd ? "sent" : "FAILED");
        } else if (cmd == GIP_AUTH || cmd == GIP_FINAL_AUTH) {
            // Check for auth complete marker. These are non-chunked packets, for
            // which the parser never populates the data pointer, so read the raw
            // bytes: cmd 0x06, length 2, payload 01 01.
            if (xferred_bytes >= 6 && p_xbone->epout_buf[3] == 0x02 &&
                p_xbone->epout_buf[4] == 0x01) {
                if (p_xbone->epout_buf[5] == AUTH_MARKER_EXCHANGE_DONE) {
                    // The console closed the auth exchange and sends power-on
                    // next, so stop waiting for an explicit power-mode command.
                    xbox_powered_on = true;
                    printf("[tud_xbone] Auth marker 01 02 - console closed the exchange\n");
                    // xone: the *host* has to close the handshake with a GIP_AUTH
                    // COMPLETE control packet (06 20 <seq> 02 01 00). The console
                    // only ever sends 01 01 / 01 02, so nothing ever told the pad
                    // it was authenticated - send it ourselves on the console's
                    // behalf, otherwise the pad never emits input reports.
                    xbone_auth_notify_console_exchange_done();
                    xbone_auth_send_auth_complete();
                } else if (p_xbone->epout_buf[5] == AUTH_MARKER_RESPONSE_ACCEPTED) {
                    printf("[tud_xbone] Auth marker 01 01 - console accepted a response\n");
                } else {
                    printf("[tud_xbone] Auth marker 01 %02x from console\n",
                           p_xbone->epout_buf[5]);
                }
            }

            // Reassemble the console's chunked challenge first: relaying the raw
            // chunks straight to the controller was tried and the console stopped
            // accepting us, so keep the reassemble-then-rechunk path.
            if (chunk_mode == XBONE_CHUNK_RELAY && xgip_is_chunked(&incoming_xgip)) {
                // 照抄主机发来的分片，逐包原样转发给手柄。
                //
                // 手柄直插 Xbox 时用的就是它自己的分片大小，主机天然接受。所以
                // "两边分片不一致"是我们自己造出来的：之前是重组再重新分片，然后
                // 为了让两边账目对得上，又长出一整堆 ACK 记账补丁。照抄之后这些
                // 都不需要了。
                //
                // 只转发分片数据；非分片的认证标记（01 01 / 01 02）是主机发给
                // *我们*的控制包，转给手柄没有意义，仍走下面的本地处理。
                bool fwd = xbone_auth_send_raw_to_controller(p_xbone->epout_buf,
                                                             (uint16_t)xferred_bytes);
                printf("[tud_xbone] Auth chunk RELAY -> controller: wire=%u/%u %s\n",
                       (unsigned)incoming_xgip.chunk_counter,
                       (unsigned)incoming_xgip.total_chunk_length,
                       fwd ? "sent" : "FAILED");
                xgip_reset(&incoming_xgip);
            } else if (!xgip_is_chunked(&incoming_xgip) ||
                (xgip_is_chunked(&incoming_xgip) && xgip_end_of_chunk(&incoming_xgip))) {

                uint16_t plen = xgip_is_chunked(&incoming_xgip)
                                    ? (uint16_t)incoming_xgip.actual_data_received
                                    : (uint16_t)xgip_get_data_length(&incoming_xgip);

                if (plen == 0) {
                    // Stray end-of-chunk terminator with no payload. Forwarding
                    // an empty auth challenge upsets the controller, so drop it.
                    printf("[tud_xbone] Ignoring empty auth chunk from console\n");
                } else {
                    xbone_auth_set_data(xgip_get_data(&incoming_xgip),
                                       plen,
                                       xgip_get_sequence(&incoming_xgip),
                                       xgip_get_command(&incoming_xgip),
                                       XBONE_AUTH_SEND_CONSOLE_TO_DONGLE);
                }
                xgip_reset(&incoming_xgip);
            }
        } else {
            // 白名单之外一律原样转发给手柄。
            //
            // 这一支以前是空的：命令被静默丢弃，连日志都没有。于是"我们漏掉了
            // 什么"这件事本身是不可见的 —— 每次排查都只能靠猜。转发并打印之后，
            // 至少能看见主机到底在跟我们要什么。
            bool fwd = xbone_auth_send_raw_to_controller(p_xbone->epout_buf,
                                                         (uint16_t)xferred_bytes);
            printf("[tud_xbone] Unlisted cmd -> controller: cmd=0x%02x len=%u %s\n",
                   cmd, (unsigned)xferred_bytes, fwd ? "sent" : "FAILED");
        }

        // Ready for next packet
        TU_ASSERT(usbd_edpt_xfer(rhport, p_xbone->ep_out, p_xbone->epout_buf,
                                  sizeof(p_xbone->epout_buf)));
    }

    return true;
}

// ============================================================================
// CLASS DRIVER STRUCT
// ============================================================================

static const usbd_class_driver_t xbone_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "XBONE",
#endif
    .init = xbone_init,
    .reset = xbone_reset,
    .open = xbone_open,
    .control_xfer_cb = xbone_control_xfer_cb,
    .xfer_cb = xbone_xfer_cb,
    .sof = NULL
};

const usbd_class_driver_t* tud_xbone_class_driver(void)
{
    return &xbone_driver;
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool tud_xbone_ready(void)
{
    for (uint8_t i = 0; i < XBONE_ITF_COUNT; i++) {
        if (xbone_itf[i].ep_in != 0) {
            return tud_ready() && !usbd_edpt_busy(0, xbone_itf[i].ep_in);
        }
    }
    return false;
}

bool tud_xbone_send_report(gip_input_report_t* report)
{
    // Set up GIP header
    report->header.command = GIP_INPUT_REPORT;
    report->header.internal = 0;
    report->header.client = 0;
    report->header.needs_ack = 0;
    report->header.chunk_start = 0;
    report->header.chunked = 0;
    report->header.sequence = input_report_sequence;
    report->header.length = sizeof(gip_input_report_t) - sizeof(gip_header_t);

    if (send_report_internal((uint8_t*)report, sizeof(gip_input_report_t))) {
        input_reports_sent_to_console++;
        input_report_sequence++;
        if (input_report_sequence == 0) input_report_sequence = 1;
        return true;
    }
    return false;
}

bool tud_xbone_send_raw(uint8_t const* data, uint16_t len)
{
    if (data == NULL || len == 0 || len > XBONE_ENDPOINT_SIZE) {
        return false;
    }

    uint8_t buf[XBONE_ENDPOINT_SIZE];
    memcpy(buf, data, len);
    // 打印交给调用方：输入报告是 250 Hz，在这里打会淹掉串口
    return queue_report(buf, len);
}



bool tud_xbone_send_virtual_keycode(uint8_t keycode, bool pressed)
{
    // GIP virtual keycode packet: header(4) + payload(2)
    //   payload[0] = pressed flag (0x01 = down, 0x00 = up)
    //   payload[1] = keycode (Guide = 0x5B)
    // needs_ack=1 + internal=1 matches the form real controllers send. The
    // bit in the standard input report is informational; consoles only open
    // the Guide overlay on receipt of this packet pair.
    uint8_t pkt[sizeof(gip_header_t) + 2] = { 0 };
    gip_header_t* hdr = (gip_header_t*)pkt;
    hdr->command = GIP_VIRTUAL_KEYCODE;
    hdr->client = 0;
    hdr->needs_ack = 1;
    hdr->internal = 1;
    hdr->chunk_start = 0;
    hdr->chunked = 0;
    hdr->sequence = input_report_sequence;
    hdr->length = 2;
    pkt[sizeof(gip_header_t) + 0] = pressed ? 0x01 : 0x00;
    pkt[sizeof(gip_header_t) + 1] = keycode;

    if (send_report_internal(pkt, sizeof(pkt))) {
        input_report_sequence++;
        if (input_report_sequence == 0) input_report_sequence = 1;
        return true;
    }
    return false;
}

bool tud_xbone_send_guide(bool pressed)
{
    return tud_xbone_send_virtual_keycode(0x5B, pressed);
}

// ============================================================================
// LATE CONTROLLER HANDSHAKE RESTART
// ============================================================================
//
// The console drives announce -> descriptor -> auth exactly once, when our
// device appears on its bus. A controller that is plugged in afterwards never
// receives a challenge: it stays in its own auth state, never switches to input
// reports, and nothing we send afterwards makes the console repeat the
// exchange. The only reliable way to get such a controller into the exchange is
// to leave the console's bus and come back, so the console walks the whole
// handshake again with the controller already attached.

#define REENUM_OFF_MS 250

static uint32_t reenum_start_ms = 0;

bool tud_xbone_handshake_started(void)
{
    return driver_state != XBONE_STATE_IDLE &&
           driver_state != XBONE_STATE_READY_ANNOUNCE;
}

void tud_xbone_request_reenumeration(void)
{
    if (reenum_start_ms != 0) {
        return;  // one already in flight
    }
    printf("[tud_xbone] Controller mounted after handshake started - re-enumerating on console bus\n");
    tud_disconnect();
    reenum_start_ms = platform_time_ms();
}

// TinyUSB declares these as weak symbols in device/usbd.c, so defining them
// here is how we get notified. Nothing else in this tree defines them
// (checked before adding - a second strong definition would be a link error).
//
// tud_mount_cb fires once the console has sent SET_CONFIGURATION, i.e. it
// really did enumerate us. tud_umount_cb fires on bus reset / detach. That
// pair answers "is anything attached to our device port at all", which the
// absence of a VBUS sense line made unanswerable before.
void tud_mount_cb(void)
{
    printf("[tud_xbone] CONSOLE MOUNTED t=%lu (host finished enumerating us)\n",
           (unsigned long)platform_time_ms());
}

void tud_umount_cb(void)
{
    printf("[tud_xbone] CONSOLE UNMOUNTED t=%lu\n",
           (unsigned long)platform_time_ms());
}

void tud_xbone_update(void)
{
    uint32_t now = platform_time_ms();

    // Come back onto the console's bus after a re-enumeration was requested.
    if (reenum_start_ms != 0 && (now - reenum_start_ms) >= REENUM_OFF_MS) {
        reenum_start_ms = 0;
        printf("[tud_xbone] Re-enumeration: reconnecting to console\n");
        tud_connect();
    }

    // Heartbeat. It prints whether or not the pad is talking, so "the pad went
    // silent" can be told apart from "our loop stalled" or "we never forward".
    {
        static uint32_t last_status_ms = 0;
        if ((now - last_status_ms) >= 2000) {
            last_status_ms = now;
            printf("[tud_xbone] STATUS t=%lu state=%d powered=%d auth=%d input_sent=%lu"
                   " mounted=%d connected=%d cons_rx=%lu cons_idle=%lu\n",
                   (unsigned long)now, driver_state, xbox_powered_on ? 1 : 0,
                   auth_data.auth_completed ? 1 : 0,
                   (unsigned long)input_reports_sent_to_console,
                   tud_mounted() ? 1 : 0,
                   tud_connected() ? 1 : 0,
                   (unsigned long)console_rx_count,
                   (unsigned long)(console_rx_count ? (now - console_last_rx_ms) : now));
        }
    }

    // Keep the console from disconnecting (runs every iteration, independent of
    // ACK wait state — GP2040-CE process() does these unconditionally):
    //  - while auth is pending, push idle GIP_INPUT_REPORTs continuously
    //  - after auth completes, push GIP_KEEPALIVE every 15s
    if (driver_state == XBONE_STATE_SETUP_AUTH) {
        if (!auth_data.auth_completed) {
            if ((now - last_idle_report_ms) >= IDLE_REPORT_INTERVAL) {
                gip_input_report_t idle = { 0 };
                idle.header.command = GIP_INPUT_REPORT;
                idle.header.internal = 0;
                idle.header.sequence = input_report_sequence;
                idle.header.length = sizeof(gip_input_report_t) - sizeof(gip_header_t);
                memcpy(((uint8_t*)&idle) + sizeof(gip_header_t),
                       xbone_idle_payload, sizeof(xbone_idle_payload));
                if (send_report_internal((uint8_t*)&idle, sizeof(gip_input_report_t))) {
                    input_report_sequence++;
                    if (input_report_sequence == 0) input_report_sequence = 1;
                    last_idle_report_ms = now;
                }
            }
        } else {
            if ((now - last_keepalive_ms) >= KEEPALIVE_INTERVAL) {
                uint8_t pkt[8] = {
                    GIP_KEEPALIVE,
                    0x20,  // internal=1, client=0, ack=0, chunk=0
                    keepalive_sequence,
                    0x04,
                    0x80, 0x00, 0x00, 0x00
                };
                if (send_report_internal(pkt, sizeof(pkt))) {
                    keepalive_sequence++;
                    if (keepalive_sequence == 0) keepalive_sequence = 1;
                    last_keepalive_ms = now;
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // 注入：启动之后必须按时把左扳机发出去，在主机看来就是手柄自己按的。
    //
    // 手柄在说话时，虚拟扳机是**合并**进转发的包里的（见 xbone_auth.c 的
    // GIP_INPUT_REPORT 分支），手柄的按键和摇杆都保留。手柄沉默时没有包可合并，
    // 这里就必须由我们自己把报告流撑起来 —— 否则按了"启动"等于什么都没发生。
    // 两条路共用 xbone_inject_trigger_value()，不会各走一套计时。
    // ------------------------------------------------------------------------
    if (xbone_inject_is_running()) {
        uint32_t last_pad = xbone_auth_last_pad_relay_ms();
        bool pad_quiet = (last_pad == 0) || ((now - last_pad) >= PAD_STREAM_IDLE_MS);

        if (pad_quiet && (now - last_inject_report_ms) >= INJECT_REPORT_INTERVAL) {
            // 静止的中位摇杆 + 虚拟扳机。和 auth 期间那份 idle 报告同构。
            gip_input_report_t rep = { 0 };
            rep.left_trigger = xbone_inject_trigger_value(now);

            if (tud_xbone_send_report(&rep)) {
                last_inject_report_ms = now;
                inject_reports_sent++;
                if ((inject_reports_sent & 0x3F) == 0) {
                    printf("[tud_xbone] Inject driving report stream: n=%lu lt=%u (pad quiet %lums)\n",
                           (unsigned long)inject_reports_sent,
                           (unsigned)rep.left_trigger,
                           (unsigned long)(last_pad ? (now - last_pad) : now));
                }
            }
        }
    }

    // Process report queue
    if (queue_count > 0 && (now - last_report_queue_sent) > REPORT_QUEUE_INTERVAL) {
        uint8_t report[XBONE_ENDPOINT_SIZE];
        uint16_t len;

        if (dequeue_report(report, &len)) {
            if (send_report_internal(report, len)) {
                last_report_queue_sent = now;
            } else {
                // Failed - re-queue at front (by adjusting head back)
                queue_head = (queue_head + REPORT_QUEUE_SIZE - 1) % REPORT_QUEUE_SIZE;
                queue_count++;
                platform_sleep_ms(REPORT_QUEUE_INTERVAL);
            }
        }
    }

    // Don't proceed if waiting for ACK
    if (waiting_ack) {
        if ((now - waiting_ack_timeout) < ACK_WAIT_TIMEOUT) {
            return;
        }
        // ACK timeout - continue anyway
        waiting_ack = false;
    }

    switch (driver_state) {
        case XBONE_STATE_READY_ANNOUNCE:
            // Announce to the console unconditionally after the fixed delay
            // (matching GP2040-CE). Do NOT gate this on the auth source being
            // ready: the console drives announce → descriptor → auth and idles
            // at the auth stage (we keep it alive with idle reports), so the
            // dongle/controller gets as long as it needs to finish its own
            // init. Auth packets are only forwarded once the source is actually
            // ready (gated in xbone_auth_task on dongle_ready), so a not-yet-
            // ready source just delays the auth exchange instead of failing it.
            // Gating the announce here (plus a premature-ready timeout) caused a
            // race where auth was forwarded before the source was initialized —
            // worked on the Nth retry for dongles, never for a real controller.
            if (now - timer_announce > ANNOUNCE_DELAY) {
                printf("[tud_xbone] Announce to console\n");

                xgip_reset(&outgoing_xgip);
                xgip_set_attributes(&outgoing_xgip, GIP_ANNOUNCE, 1, 1, 0, 0);

                // Copy announce packet with timestamp
                uint8_t announce[sizeof(xbone_announce_packet)];
                memcpy(announce, xbone_announce_packet, sizeof(announce));
                memcpy(&announce[3], &now, 3);  // Insert timestamp

                xgip_set_data(&outgoing_xgip, announce, sizeof(announce));
                queue_report(xgip_generate_packet(&outgoing_xgip),
                            xgip_get_packet_length(&outgoing_xgip));

                driver_state = XBONE_STATE_WAIT_DESCRIPTOR_REQUEST;
            }
            break;

        case XBONE_STATE_SEND_DESCRIPTOR:
            queue_report(xgip_generate_packet(&outgoing_xgip),
                        xgip_get_packet_length(&outgoing_xgip));

            if (xgip_end_of_chunk(&outgoing_xgip)) {
                driver_state = XBONE_STATE_SETUP_AUTH;
            }

            if (xgip_get_packet_ack(&outgoing_xgip)) {
                set_ack_wait();
            }
            break;

        case XBONE_STATE_SETUP_AUTH:
            // Handle auth passthrough from dongle to console
            if (auth_data.state == XBONE_AUTH_SEND_DONGLE_TO_CONSOLE) {
                // The controller resends its response until it is satisfied,
                // and every resend produced another forward. Each one reset the
                // console-bound stream to its first chunk, so an 825-byte
                // signature was only ever emitted as 58 bytes and the console
                // sat at ACK recv=58 rem=767 forever. Defer the new one until
                // the stream already on the wire has finished.
                if (xgip_transfer_in_flight(&outgoing_xgip)) {
                    // Keep emitting the stream already on the wire and drop
                    // this forward - the controller resends whatever it still
                    // wants, so nothing is lost by waiting.
                    auth_data.state = XBONE_AUTH_WAIT_DONGLE_TO_CONSOLE;
                    break;
                }

                bool is_chunked = (auth_data.length > GIP_MAX_CHUNK_SIZE);
                xgip_reset(&outgoing_xgip);
                xgip_set_attributes(&outgoing_xgip, auth_data.auth_type,
                                   auth_data.sequence, 1, is_chunked, 1);
                xgip_set_data(&outgoing_xgip, auth_data.buffer, auth_data.length);
                auth_data.state = XBONE_AUTH_WAIT_DONGLE_TO_CONSOLE;
            } else if (auth_data.state == XBONE_AUTH_WAIT_DONGLE_TO_CONSOLE) {
                uint8_t* out = xgip_generate_packet(&outgoing_xgip);
                printf("[tud_xbone] chunk to console: len=%u payload=%u counter=%u\n",
                       (unsigned)xgip_get_packet_length(&outgoing_xgip),
                       (unsigned)out[3],
                       (unsigned)(out[4] | (out[5] << 8)));
                queue_report(out, xgip_get_packet_length(&outgoing_xgip));

                if (!xgip_is_chunked(&outgoing_xgip) || xgip_end_of_chunk(&outgoing_xgip)) {
                    auth_data.state = XBONE_AUTH_IDLE;
                }

                if (xgip_get_packet_ack(&outgoing_xgip)) {
                    set_ack_wait();
                }
            }
            break;

        case XBONE_STATE_IDLE:
        case XBONE_STATE_WAIT_DESCRIPTOR_REQUEST:
        default:
            break;
    }
}

bool tud_xbone_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                       tusb_control_request_t const* request)
{
    static uint8_t buf[255];

    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        uint16_t len = request->wLength;

        // Handle Windows OS descriptor request
        if (request->bmRequestType == (USB_SETUP_DEVICE_TO_HOST | USB_SETUP_RECIPIENT_DEVICE | USB_SETUP_TYPE_VENDOR) &&
            request->bRequest == REQ_GET_OS_FEATURE_DESCRIPTOR &&
            request->wIndex == DESC_EXTENDED_COMPATIBLE_ID) {

            if (len > sizeof(xbone_os_compat_descriptor)) {
                len = sizeof(xbone_os_compat_descriptor);
            }
            memcpy(buf, &xbone_os_compat_descriptor, len);
        }

        tud_control_xfer(rhport, request, buf, len);
    } else {
        tud_control_xfer(rhport, request, buf, request->wLength);
    }

    return true;
}

// ============================================================================
// AUTH PASSTHROUGH API
// ============================================================================

xbone_auth_state_t xbone_auth_get_state(void)
{
    return auth_data.state;
}

void xbone_auth_set_data(uint8_t* data, uint16_t len, uint8_t seq,
                         uint8_t type, xbone_auth_state_t new_state)
{
    if (len > XGIP_MAX_DATA_SIZE) {
        return;
    }

    memcpy(auth_data.buffer, data, len);
    auth_data.length = len;
    auth_data.sequence = seq;
    auth_data.auth_type = type;
    auth_data.state = new_state;
}

uint8_t* xbone_auth_get_buffer(void)
{
    return auth_data.buffer;
}

uint16_t xbone_auth_get_length(void)
{
    return auth_data.length;
}

uint8_t xbone_auth_get_sequence(void)
{
    return auth_data.sequence;
}

uint8_t xbone_auth_get_type(void)
{
    return auth_data.auth_type;
}

bool xbone_auth_is_completed(void)
{
    return auth_data.auth_completed;
}

void xbone_auth_set_completed(bool completed)
{
    auth_data.auth_completed = completed;
}

bool xbone_is_powered_on(void)
{
    return xbox_powered_on;
}
