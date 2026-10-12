// gip_map_flash.h - Keeps a downloaded GIP button map across power cycles.
//
// The main settings page (flash_t) is exactly 256 bytes with 5 spare, which is
// not room for a 16-entry button map, so this uses its own 4KB sector - the
// same arrangement ps4_auth_flash.c uses for key material. A corrupt or absent
// sector only costs the custom map: the caller falls back to the compiled-in
// layout.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef GIP_MAP_FLASH_H
#define GIP_MAP_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#include "gip_button_map.h"

#define GIP_MAP_FLASH_MAGIC 0x4749504D  // "GIPM"

typedef struct __attribute__((packed)) {
    uint32_t magic;             // GIP_MAP_FLASH_MAGIC
    uint32_t crc32;             // CRC-32 over everything after this field
    uint8_t  map[GIP_BUTTON_MAP_BITS];
    uint8_t  reserved[232];     // pad to 256 bytes (one flash page)
} gip_map_data_t;

// Read the stored map into *out. False when the sector is empty or corrupt.
bool gip_map_flash_load(gip_map_data_t* out);

// Write *data to the sector. Erases first, then verifies by reading back, and
// reports failure rather than assuming success.
bool gip_map_flash_save(const gip_map_data_t* data);

bool gip_map_flash_erase(void);

// Load the stored map (if any) into gip_button_map_current(). Called once at
// start-up; leaves the compiled-in layout in force when nothing is stored.
void gip_map_init(void);

#endif // GIP_MAP_FLASH_H
