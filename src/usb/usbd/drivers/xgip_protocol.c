// xgip_protocol.c - Xbox Game Input Protocol handler
// SPDX-License-Identifier: MIT
// Based on GP2040-CE implementation (gp2040-ce.info)

#include "xgip_protocol.h"
#include <string.h>
#include <stdlib.h>

void xgip_init(xgip_t* xgip)
{
    memset(xgip, 0, sizeof(xgip_t));
    xgip->data = NULL;
}

// Has this exact chunk already been buffered? Chunks are identified by the
// counter the sender puts in bytes [4..5].
static bool xgip_chunk_buffered(xgip_t* xgip, uint16_t counter)
{
    for (uint8_t i = 0; i < xgip->buffered_counter_count; i++) {
        if (xgip->buffered_counters[i] == counter) {
            return true;
        }
    }
    return false;
}

static void xgip_remember_chunk(xgip_t* xgip, uint16_t counter)
{
    if (counter == 0 || xgip_chunk_buffered(xgip, counter)) {
        return;
    }
    if (xgip->buffered_counter_count < XGIP_CHUNK_RING) {
        xgip->buffered_counters[xgip->buffered_counter_count++] = counter;
    } else {
        // Ring is full: drop the oldest. A single transfer never gets here in
        // practice (a signature is ~15 chunks).
        memmove(&xgip->buffered_counters[0], &xgip->buffered_counters[1],
                sizeof(xgip->buffered_counters) - sizeof(uint16_t));
        xgip->buffered_counters[XGIP_CHUNK_RING - 1] = counter;
    }
}

void xgip_reset(xgip_t* xgip)
{
    // ack_echo_peer_counter is a property of the peer at the other end of this
    // instance, not of the transfer, so it survives a reset.
    bool ack_mode = xgip->ack_echo_peer_counter;

    memset(&xgip->header, 0, sizeof(gip_header_t));
    xgip->total_chunk_length = 0;
    xgip->chunk_counter = 0;
    xgip->buffered_counter_count = 0;
    memset(xgip->buffered_counters, 0, sizeof(xgip->buffered_counters));
    xgip->last_chunk_duplicate = false;
    // The override is per-transfer: a retransmission we answer for the full
    // amount says nothing about the next stream we start reassembling.
    xgip->ack_override_valid = false;
    xgip->ack_received_override = 0;
    xgip->actual_data_received = 0;
    xgip->total_chunk_received = 0;
    xgip->total_chunk_sent = 0;
    xgip->total_data_sent = 0;
    xgip->num_chunks_sent = 0;
    xgip->chunk_ended = false;
    xgip->is_valid = false;

    if (xgip->data != NULL) {
        free(xgip->data);
        xgip->data = NULL;
    }
    xgip->data_length = 0;

    memset(xgip->packet, 0, sizeof(xgip->packet));
    xgip->packet_length = 0;
    xgip->ack_echo_peer_counter = ack_mode;
}

bool xgip_parse(xgip_t* xgip, const uint8_t* buffer, uint16_t len)
{
    // Need at least header
    if (len < 4) {
        xgip_reset(xgip);
        xgip->is_valid = false;
        return false;
    }

    xgip->packet_length = len;
    // The override is set by the caller per ACK, so a new packet starts without
    // one - otherwise a stale remembered length leaks into a later transfer.
    xgip->ack_override_valid = false;
    gip_header_t* new_header = (gip_header_t*)buffer;

    // Handle ACK response
    if (new_header->command == GIP_ACK_RESPONSE) {
        if (len != 13 || new_header->internal != 0x01 || new_header->length != 0x09) {
            xgip_reset(xgip);
            xgip->is_valid = false;
            return false;
        }
        memcpy(&xgip->header, buffer, sizeof(gip_header_t));
        xgip->is_valid = true;
        return true;
    }

    // Handle chunked data
    if (new_header->chunked) {
        memcpy(&xgip->header, buffer, sizeof(gip_header_t));

        if (xgip->header.length == 0) {
            // End of chunk marker
            uint16_t end_chunk_size = (buffer[4] | (buffer[5] << 8));
            // total_chunk_length == 0 means we joined the stream mid-way and
            // never saw the chunk_start packet, so we have no total to check
            // against - trust the marker instead of failing the transfer.
            if (xgip->total_chunk_length != 0 && xgip->total_chunk_length != end_chunk_size) {
                xgip->is_valid = false;
                return false;
            }
            xgip->total_chunk_length = end_chunk_size;
            xgip->chunk_ended = true;
            xgip->is_valid = true;
            return true;
        }

        // Echo whatever cumulative accounting the sender reports in bytes [4..5].
        // Only the first chunk of a stream carries the *total*; continuation
        // chunks carry the running counter.
        xgip->chunk_counter = (uint16_t)(buffer[4] | (buffer[5] << 8));
        bool duplicate_chunk = false;

        // Some controllers (observed on a wired PDP pad, VID 0e6f PID 02e7)
        // send the first chunk of a response WITHOUT the chunk_start flag. The
        // parser would then treat it as a continuation, never allocate a buffer,
        // accumulate nothing, and ACK "0 bytes received" - which makes the
        // controller resend the same chunk forever. Treat a chunked packet that
        // arrives with no reassembly in progress as the start of a new chunk.
        if (xgip->header.chunk_start || xgip->data == NULL) {
            // Start of new chunk
            bool have_start = xgip->header.chunk_start;
            xgip_reset(xgip);
            memcpy(&xgip->header, buffer, sizeof(gip_header_t));

            if (have_start) {
                // Calculate total chunk length
                if (xgip->header.length > GIP_MAX_CHUNK_SIZE && buffer[4] == 0x00) {
                    // Single-byte mode
                    xgip->total_chunk_length = buffer[5];
                } else {
                    xgip->total_chunk_length = (buffer[4] | (buffer[5] << 8));
                }

                // Calculate actual data length
                xgip->data_length = xgip->total_chunk_length;
                if (xgip->total_chunk_length > 0x100) {
                    xgip->data_length = xgip->data_length - 0x100;
                    xgip->data_length = xgip->data_length - ((xgip->data_length / 0x100) * 0x80);
                }
            } else {
                // Joined mid-stream: bytes [4..5] hold the running counter here,
                // NOT the total. Don't guess - just buffer everything and let the
                // end-of-chunk marker tell us when we're done.
                xgip->total_chunk_length = 0;
                xgip->data_length = XGIP_MAX_DATA_SIZE;
            }

            // Allocate data buffer
            if (xgip->data != NULL) {
                free(xgip->data);
            }
            xgip->data = (uint8_t*)malloc(xgip->data_length);
            if (xgip->data == NULL) {
                xgip->is_valid = false;
                return false;
            }

            xgip->actual_data_received = 0;
            xgip->total_chunk_received = xgip->header.length;
        } else {
            xgip->total_chunk_received += xgip->header.length;

            // The controller resends chunks whose ACK it missed. Appending those
            // a second time inflated the reassembled payload well past the real
            // total (an 825-byte signature grew to 949+ bytes of duplicates) and
            // we then handed that garbage to the console. Every chunk carries a
            // counter that identifies it, so a repeat can be spotted exactly -
            // acknowledge it, but don't store it again.
            //
            // Matching on "counter <= highest seen" instead is wrong here: the
            // pad cycles back to its oldest unacknowledged chunk, so once a late
            // chunk had been buffered every earlier counter looked like a resend,
            // new chunks were dropped and the upload froze at 174 of 825 bytes.
            if (xgip->chunk_counter != 0 &&
                xgip_chunk_buffered(xgip, xgip->chunk_counter)) {
                duplicate_chunk = true;
            }
        }

        // Copy data
        uint16_t copy_len = xgip->header.length;
        if (copy_len > GIP_MAX_CHUNK_SIZE) {
            copy_len ^= 0x80;  // Remove encoding
        }

        if (xgip->data && !duplicate_chunk &&
            (xgip->actual_data_received + copy_len) <= xgip->data_length) {
            memcpy(&xgip->data[xgip->actual_data_received], &buffer[6], copy_len);
            xgip->actual_data_received += copy_len;
            xgip_remember_chunk(xgip, xgip->chunk_counter);
        }
        xgip->last_chunk_duplicate = duplicate_chunk;
        if (!duplicate_chunk) {
            xgip->num_chunks_sent++;
        }
        xgip->is_valid = true;

        // Some controllers (e.g. MS Xbox Wireless 0x045E:0x0B12) don't send an
        // explicit length=0 end-of-chunk marker — they rely on actual_data_received
        // reaching the declared data_length. Mark complete in that case too.
        if (xgip->data_length > 0 && xgip->actual_data_received >= xgip->data_length) {
            xgip->chunk_ended = true;
        }

        // Streams we join mid-way have no declared total, so neither check above
        // can ever fire for them. But a full-size chunk means "more to follow",
        // so a chunk carrying less than the maximum payload is the last one -
        // that's the end of the response (a 825-byte signature arrives as
        // 14 full chunks plus a final 13-byte one).
        if (xgip->total_chunk_length == 0 && copy_len < GIP_MAX_CHUNK_SIZE) {
            xgip->chunk_ended = true;
            // The stream ended, so the bytes we hold ARE the payload. Leaving
            // data_length at the XGIP_MAX_DATA_SIZE cap the orphan branch chose
            // means "received >= length" can never be true - an 825-byte
            // signature under a 1024 cap is never complete, so it is never
            // forwarded and the pad keeps retransmitting it instead of moving on
            // to input (v28: LED out, every button dead).
            xgip->data_length = xgip->actual_data_received;
        }

    } else {
        // Non-chunked data
        xgip_reset(xgip);
        memcpy(&xgip->header, buffer, sizeof(gip_header_t));

        if (xgip->header.length > 0) {
            xgip->data = (uint8_t*)malloc(xgip->header.length);
            if (xgip->data != NULL) {
                memcpy(xgip->data, &buffer[4], xgip->header.length);
            }
        }
        xgip->actual_data_received = xgip->header.length;
        xgip->data_length = xgip->actual_data_received;
        xgip->is_valid = true;
    }

    return false;
}

bool xgip_validate(xgip_t* xgip)
{
    return xgip->is_valid;
}

bool xgip_end_of_chunk(xgip_t* xgip)
{
    return xgip->chunk_ended;
}

void xgip_force_end_of_chunk(xgip_t* xgip)
{
    if (xgip->chunk_ended || xgip->actual_data_received == 0) {
        return;
    }
    // Streams we join mid-way have no declared total, so nothing can tell us
    // we're done: the controller only sends its end-of-chunk marker once it got
    // every ACK, and a late ACK can keep that from ever happening. If it has
    // gone quiet, whatever we hold is the response - take it at that length.
    if (xgip->actual_data_received < xgip->data_length) {
        xgip->data_length = xgip->actual_data_received;
    }
    xgip->chunk_ended = true;
}

bool xgip_ack_required(xgip_t* xgip)
{
    return xgip->header.needs_ack;
}

void xgip_set_attributes(xgip_t* xgip, uint8_t cmd, uint8_t seq,
                         uint8_t internal, uint8_t is_chunked, uint8_t needs_ack)
{
    xgip->header.command = cmd;
    xgip->header.sequence = seq;
    xgip->header.internal = internal;
    xgip->header.chunked = is_chunked;
    xgip->header.needs_ack = needs_ack;
}

bool xgip_set_data(xgip_t* xgip, const uint8_t* data, uint16_t len)
{
    if (len > XGIP_MAX_DATA_SIZE) {
        return false;
    }

    if (xgip->data != NULL) {
        free(xgip->data);
    }

    xgip->data = (uint8_t*)malloc(len);
    if (xgip->data == NULL) {
        return false;
    }

    memcpy(xgip->data, data, len);
    xgip->data_length = len;
    return true;
}

uint8_t* xgip_generate_packet(xgip_t* xgip)
{
    if (!xgip->header.chunked) {
        // Simple non-chunked packet
        xgip->header.length = (uint8_t)xgip->data_length;
        memcpy(xgip->packet, &xgip->header, sizeof(gip_header_t));
        if (xgip->data && xgip->data_length > 0) {
            memcpy(&xgip->packet[4], xgip->data, xgip->data_length);
        }
        xgip->packet_length = sizeof(gip_header_t) + xgip->data_length;
    } else {
        // Chunked packet handling
        if (xgip->num_chunks_sent > 0 && xgip->total_data_sent == xgip->data_length) {
            // Final chunk (end marker)
            xgip->header.needs_ack = 0;
            xgip->header.length = 0;
            memcpy(xgip->packet, &xgip->header, sizeof(gip_header_t));
            xgip->packet[4] = xgip->total_chunk_length & 0xFF;
            xgip->packet[5] = (xgip->total_chunk_length >> 8) & 0xFF;
            xgip->packet_length = sizeof(gip_header_t) + 2;
            xgip->chunk_ended = true;
        } else {
            if (xgip->num_chunks_sent == 0) {
                if (xgip->data_length < GIP_MAX_CHUNK_SIZE) {
                    // Small chunk - treat as non-chunked but still needs ACK
                    xgip->total_chunk_length = xgip->data_length;
                    xgip->header.chunk_start = 0;
                    xgip->header.chunked = 0;
                } else {
                    xgip->header.chunk_start = 1;

                    // Calculate chunk length with 0x80 boundary encoding
                    uint16_t remaining = xgip->data_length;
                    uint16_t chunk_total = 0;

                    while (remaining > 0) {
                        uint16_t chunk_size;
                        if (remaining < GIP_MAX_CHUNK_SIZE) {
                            chunk_size = remaining;
                        } else {
                            chunk_size = GIP_MAX_CHUNK_SIZE;
                        }

                        // Handle 0x100 boundary
                        if ((chunk_total + chunk_size > 0x80) && (chunk_total + chunk_size < 0x100)) {
                            chunk_total = chunk_total + chunk_size + 0x100;
                        } else if ((chunk_total / 0x100) != ((chunk_total + chunk_size) / 0x100)) {
                            chunk_total = chunk_total + (chunk_size | 0x80);
                        } else {
                            chunk_total = chunk_total + chunk_size;
                        }

                        remaining -= chunk_size;
                    }
                    xgip->total_chunk_length = chunk_total;
                }
            } else {
                xgip->header.chunk_start = 0;
            }

            // Set ACK flag on 1st and every 5th chunk
            if (xgip->num_chunks_sent == 0 || (xgip->num_chunks_sent + 1) % 5 == 0) {
                xgip->header.needs_ack = 1;
            } else {
                xgip->header.needs_ack = 0;
            }

            // Determine data to send in this chunk
            uint16_t data_to_send = GIP_MAX_CHUNK_SIZE;
            if ((xgip->data_length - xgip->total_data_sent) < data_to_send) {
                data_to_send = xgip->data_length - xgip->total_data_sent;
                xgip->header.needs_ack = 1;
            }

            // Leave this alone. Two "fixes" were tried against a real console
            // and both made things worse:
            //   always setting 0x80 (v21)  -> the 202-byte descriptor stalled at
            //                                 ACK recv=174 rem=28, console went
            //                                 silent and dropped the pad
            //   clearing it on the last chunk (v20) -> a 90-byte response
            //                                 stalled at ACK recv=58 rem=32
            // This exact form is what both transfers completed under.
            // Set length with 0x80 encoding if needed
            if (xgip->num_chunks_sent > 0 && xgip->total_chunk_sent < 0x100) {
                xgip->header.length = data_to_send | 0x80;
            } else if (xgip->num_chunks_sent == 0 && xgip->data_length > GIP_MAX_CHUNK_SIZE && xgip->data_length < 0x80) {
                xgip->header.length = data_to_send | 0x80;
            } else {
                xgip->header.length = data_to_send;
            }

            // Build packet
            memcpy(xgip->packet, &xgip->header, sizeof(gip_header_t));
            if (xgip->data) {
                memcpy(&xgip->packet[6], &xgip->data[xgip->total_data_sent], data_to_send);
            }
            xgip->packet_length = sizeof(gip_header_t) + 2 + data_to_send;

            // Set chunk offset value
            uint16_t chunk_value;
            if (xgip->num_chunks_sent == 0) {
                chunk_value = xgip->total_chunk_length;
            } else {
                chunk_value = xgip->total_chunk_sent;
            }

            if (chunk_value < 0x100) {
                xgip->packet[4] = 0x00;
                xgip->packet[5] = (uint8_t)chunk_value;
            } else {
                xgip->packet[4] = chunk_value & 0xFF;
                xgip->packet[5] = (chunk_value >> 8) & 0xFF;
            }

            // Update chunk tracking with 0x80 boundary encoding
            if (xgip->total_chunk_sent < 0x100 && (xgip->total_chunk_sent + data_to_send) > 0x80) {
                xgip->total_chunk_sent = xgip->total_chunk_sent + data_to_send + 0x100;
            } else if (((xgip->total_chunk_sent + data_to_send) / 0x100) > (xgip->total_chunk_sent / 0x100)) {
                xgip->total_chunk_sent = xgip->total_chunk_sent + (data_to_send | 0x80);
            } else {
                xgip->total_chunk_sent = xgip->total_chunk_sent + data_to_send;
            }

            xgip->total_data_sent += data_to_send;
            xgip->num_chunks_sent++;
        }
    }

    return xgip->packet;
}

uint8_t* xgip_generate_ack(xgip_t* xgip)
{
    xgip->packet[0] = 0x01;  // GIP_ACK_RESPONSE
    xgip->packet[1] = 0x20;
    xgip->packet[2] = xgip->header.sequence;
    xgip->packet[3] = 0x09;  // Length
    xgip->packet[4] = 0x00;
    xgip->packet[5] = xgip->header.command;
    xgip->packet[6] = 0x20;

    // Bytes received, in LOGICAL bytes - not the peer's wire count.
    //
    // Ground truth for this came from the console itself: we sent it a 90-byte
    // response as a 58-byte chunk + a 32-byte chunk, and it ACKed
    // "recv=58 rem=32" then "recv=90 rem=0". 58 + 32 = 90, the logical total.
    // So the peer reports plain data bytes, and bytes [4..5] of a chunk are NOT
    // a running counter we should echo (decoding them as one produced junk:
    // 0 / 14848 / 29696 / 430 ...). Echoing them stalled the controller's upload
    // at 14 of 15 chunks (812 of 825 bytes) every single time.
    uint16_t ack_received = xgip->ack_override_valid
                                ? xgip->ack_received_override
                                : xgip->actual_data_received;
    uint16_t ack_total = xgip->data_length;

    if (xgip->ack_echo_peer_counter && xgip->header.chunked) {
        // This peer keeps its own running total in bytes [4..5] and won't move
        // on until the ACK matches it - it retransmits the same chunks forever
        // otherwise (observed stuck at wire=918 while we reported 174).
        // bytes[4..5] is how much the peer had put on the wire BEFORE this
        // chunk, so we owe it the count including this chunk. Ground truth: a
        // 90-byte response arrived as a 58-byte chunk with counter 0 followed by
        // a 32-byte chunk with counter 186 - and 186 is 58|0x80, the first
        // chunk's on-wire length. The raw header length already carries that
        // encoding.
        ack_received = (uint16_t)(xgip->chunk_counter + xgip->header.length);
        ack_total = xgip->total_chunk_length;
    }

    xgip->packet[7] = ack_received & 0xFF;
    xgip->packet[8] = (ack_received >> 8) & 0xFF;
    xgip->packet[9] = 0x00;
    xgip->packet[10] = 0x00;

    // Bytes remaining (for chunked)
    if (xgip->header.chunked) {
        // Same logical accounting as the received count: the peer's own total is
        // carried in the wire encoding (1721 for an 825-byte payload), so use the
        // decoded logical total here. Mixing the two (1721 - 696) is what made the
        // controller stall after 12 chunks.
        uint16_t remaining = (ack_total > ack_received)
                                 ? (uint16_t)(ack_total - ack_received)
                                 : 0;
        xgip->packet[11] = remaining & 0xFF;
        xgip->packet[12] = (remaining >> 8) & 0xFF;
    } else {
        xgip->packet[11] = 0;
        xgip->packet[12] = 0;
    }

    xgip->packet_length = 13;
    return xgip->packet;
}

uint8_t xgip_get_command(xgip_t* xgip)
{
    return xgip->header.command;
}

uint8_t xgip_get_sequence(xgip_t* xgip)
{
    return xgip->header.sequence;
}

bool xgip_is_chunked(xgip_t* xgip)
{
    return xgip->header.chunked;
}

bool xgip_transfer_in_flight(xgip_t* xgip)
{
    if (xgip == NULL || !xgip->header.chunked) {
        return false;
    }
    // Chunked but not yet ended: the peer is still waiting for the rest, so
    // this stream must not be restarted from its first chunk.
    return !xgip->chunk_ended;
}

uint8_t xgip_get_packet_length(xgip_t* xgip)
{
    return xgip->packet_length;
}

bool xgip_get_packet_ack(xgip_t* xgip)
{
    return xgip->header.needs_ack;
}

uint8_t* xgip_get_data(xgip_t* xgip)
{
    return xgip->data;
}

uint16_t xgip_get_data_length(xgip_t* xgip)
{
    return xgip->data_length;
}

void xgip_increment_sequence(xgip_t* xgip)
{
    xgip->header.sequence++;
    if (xgip->header.sequence == 0) {
        xgip->header.sequence = 1;
    }
}
