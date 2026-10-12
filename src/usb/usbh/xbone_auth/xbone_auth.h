// xbone_auth.h - Xbox One Auth Dongle Host Driver
// SPDX-License-Identifier: MIT
// Based on GP2040-CE implementation (gp2040-ce.info)
//
// This driver detects Xbox One authentication dongles (like Magic-X)
// connected to the USB host port and handles auth passthrough between
// the dongle and an Xbox One console.

#ifndef XBONE_AUTH_H
#define XBONE_AUTH_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// PUBLIC API
// ============================================================================

// Initialize Xbox One auth passthrough
void xbone_auth_init(void);

// Check if an Xbox One auth dongle is available
bool xbone_auth_is_available(void);

// Check if auth is complete
bool xbone_auth_is_complete(void);

// Task to process auth passthrough (call from main loop)
void xbone_auth_task(void);

// Register a dongle on mount
void xbone_auth_register(uint8_t dev_addr, uint8_t instance);

// Unregister dongle on unmount
void xbone_auth_unregister(uint8_t dev_addr);

// Called when report received from dongle
void xbone_auth_report_received(uint8_t dev_addr, uint8_t instance,
                                uint8_t const* report, uint16_t len);

// Called on xinput mount to check for Xbox One dongle
void xbone_auth_xmount(uint8_t dev_addr, uint8_t instance,
                       uint8_t controller_type, uint8_t subtype);

// Queue a raw GIP packet (console → controller) for delivery to the controller.
// Used to pass through power-mode / rumble / wakeup commands that the console
// sends us: without them a wired pad never starts emitting input reports.
// Returns false when the packet could not be queued. Callers log it: a dropped
// forward used to be invisible, which made every "the pad never reacted" look
// like a pad-side problem.
bool xbone_auth_send_raw_to_controller(uint8_t const* report, uint16_t len);

// Tell the auth layer that the console closed the auth exchange (auth marker
// 01 02). The console never sends the GIP_AUTH COMPLETE control packet itself,
// so this is the trigger for us to send it on the console's behalf.
void xbone_auth_notify_console_exchange_done(void);

// Send the GIP_AUTH COMPLETE control packet (06 20 <seq> 02 01 00) to the
// controller. xone (vendor/xone, gip_auth_send_complete) shows a wired pad only
// starts emitting 0x20 input reports after its host sends this.
void xbone_auth_send_auth_complete(void);

// True while the pad is streaming a chunked auth upload - that is, from the last
// chunk until UPLOAD_IDLE_MS of silence. Serial logging during that window is
// what starves the host port (see tests/gip_native/test_xbone_auth_upload.c), so
// any other per-packet print site uses this to stay quiet.
bool xbone_auth_upload_in_flight(void);

#endif // XBONE_AUTH_H
