// xgip_protocol.h - Xbox Game Input Protocol handler
// SPDX-License-Identifier: MIT
// Based on GP2040-CE implementation (gp2040-ce.info)
//
// XGIP (Xbox Game Input Protocol) is used for Xbox One controller communication.
// This implementation handles packet parsing, generation, and chunked transfers.

#ifndef XGIP_PROTOCOL_H
#define XGIP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "../descriptors/xbone_descriptors.h"

// Maximum data buffer size for auth packets
#define XGIP_MAX_DATA_SIZE 1024

// How many chunk counters we remember per transfer, for resend detection. A
// whole auth signature is ~15 chunks; the pad's retry window is well under this.
#define XGIP_CHUNK_RING 48

// XGIP Protocol state
typedef struct {
    gip_header_t header;

    // Chunk tracking
    uint16_t total_chunk_length;    // Total size of chunked data
    uint16_t chunk_counter;         // Running counter the sender puts in bytes
                                    // [4..5] of continuation chunks. Not to be
                                    // echoed back - the console's own ACKs show
                                    // the peer counts plain logical bytes - but
                                    // it does identify a chunk, so we can tell a
                                    // retransmission from a new chunk.
    // Counters of the chunks already buffered, so a resend can be recognised.
    // The pad cycles back to its oldest unacknowledged chunk, so these arrive
    // out of order - comparing against the highest counter seen instead of
    // matching exactly drops genuinely new chunks and freezes the upload.
    uint16_t buffered_counters[XGIP_CHUNK_RING];
    uint8_t buffered_counter_count;
    bool last_chunk_duplicate;      // Last parsed chunk was a resend we dropped
    bool ack_echo_peer_counter;     // ACK bytes-received in the peer's own wire
                                    // accounting instead of logical bytes. The
                                    // console counts logical bytes (its ACKs read
                                    // "recv=58 rem=32" for a 90-byte response);
                                    // our wired PDP pad counts its own running
                                    // wire counter and sits there retransmitting
                                    // the same chunks until we echo it.
    // Report this as "bytes received" in the ACK instead of
    // actual_data_received. Lets a caller answer a retransmission for the amount
    // it already holds without corrupting the reassembly: writing the remembered
    // length into actual_data_received (v26-v28) made every later chunk of a
    // fresh stream pile on top of it - a 58-byte chunk landed as 883 and the
    // stream topped out at 1012 instead of 825.
    uint16_t ack_received_override;
    bool     ack_override_valid;

    uint16_t actual_data_received;  // How much actual data received
    uint16_t total_chunk_received;  // Chunk counter (with 0x80 encoding)
    uint16_t total_chunk_sent;      // How much we've sent
    uint16_t total_data_sent;       // Actual data bytes sent
    uint16_t num_chunks_sent;       // Number of chunk packets sent
    bool chunk_ended;               // End of chunk reached

    // Output packet buffer
    uint8_t packet[64];
    uint8_t packet_length;

    // Data buffer (for incoming or outgoing data)
    uint8_t* data;
    uint16_t data_length;

    bool is_valid;                  // Is this a valid packet?
} xgip_t;

// Initialize XGIP protocol state
void xgip_init(xgip_t* xgip);

// Reset XGIP state for new packet
void xgip_reset(xgip_t* xgip);

// Parse incoming packet
// Returns true if packet is complete (non-chunked or end of chunk)
bool xgip_parse(xgip_t* xgip, const uint8_t* buffer, uint16_t len);

// Check if parsed packet is valid
bool xgip_validate(xgip_t* xgip);

// Check if we're at end of chunked data
bool xgip_end_of_chunk(xgip_t* xgip);
// Declare a reassembly complete when the peer has gone quiet and never sent an
// end-of-chunk marker (streams joined mid-way have no total to wait for).
void xgip_force_end_of_chunk(xgip_t* xgip);

// Check if last parsed packet requires ACK
bool xgip_ack_required(xgip_t* xgip);

// Set attributes for outgoing packet
void xgip_set_attributes(xgip_t* xgip, uint8_t cmd, uint8_t seq,
                         uint8_t internal, uint8_t is_chunked, uint8_t needs_ack);

// Set data for outgoing packet
// Returns false if data is too large
bool xgip_set_data(xgip_t* xgip, const uint8_t* data, uint16_t len);

// Generate next output packet (handles chunking automatically)
// Returns pointer to packet buffer
uint8_t* xgip_generate_packet(xgip_t* xgip);

// Generate ACK packet for last received packet
// Returns pointer to packet buffer
uint8_t* xgip_generate_ack(xgip_t* xgip);

// Get command from parsed packet
uint8_t xgip_get_command(xgip_t* xgip);

// Get sequence number from parsed packet
uint8_t xgip_get_sequence(xgip_t* xgip);

// Is packet chunked?
bool xgip_is_chunked(xgip_t* xgip);
bool xgip_transfer_in_flight(xgip_t* xgip);

// Get packet length of last generated output
uint8_t xgip_get_packet_length(xgip_t* xgip);

// Did last generated packet require ACK?
bool xgip_get_packet_ack(xgip_t* xgip);

// Get data from parsed packet
uint8_t* xgip_get_data(xgip_t* xgip);

// Get data length from parsed packet
uint16_t xgip_get_data_length(xgip_t* xgip);

// Increment sequence number
void xgip_increment_sequence(xgip_t* xgip);

#endif // XGIP_PROTOCOL_H
