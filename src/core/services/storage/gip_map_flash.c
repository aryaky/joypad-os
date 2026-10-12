// gip_map_flash.c - see gip_map_flash.h
//
// Flash layout (from end of flash):
//   [... firmware ...]
//   [GIP button map sector — 4KB]   <-- this module
//   [PS4 auth sector — 4KB]
//   [Settings Sector B — 4KB]
//   [Settings Sector A — 4KB]
//   [BTstack — 8KB]
//   [end of flash]
//
// The offset arithmetic is copied from ps4_auth_flash.c so the two stay in
// step; a static assert below catches it if the layout ever moves.

#include "gip_map_flash.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"

#include <string.h>
#include <stdio.h>

#define BTSTACK_FLASH_SIZE (FLASH_SECTOR_SIZE * 2)

#if PICO_RP2350 && PICO_RP2350_A2_SUPPORTED
#define _SECTOR_A_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE - BTSTACK_FLASH_SIZE - FLASH_SECTOR_SIZE)
#else
#define _SECTOR_A_OFFSET (PICO_FLASH_SIZE_BYTES - BTSTACK_FLASH_SIZE - FLASH_SECTOR_SIZE)
#endif

#define _SECTOR_B_OFFSET      (_SECTOR_A_OFFSET - FLASH_SECTOR_SIZE)
#define _PS4_AUTH_OFFSET      (_SECTOR_B_OFFSET - FLASH_SECTOR_SIZE)
#define FLASH_GIP_MAP_OFFSET  (_PS4_AUTH_OFFSET - FLASH_SECTOR_SIZE)

// The map sector must sit clear of the settings sectors.
_Static_assert(FLASH_GIP_MAP_OFFSET < _SECTOR_B_OFFSET,
               "GIP map sector must not overlap the settings sectors");

#define CRC_OFFSET 8                                     // skip magic + crc32
#define CRC_LENGTH (sizeof(gip_map_data_t) - CRC_OFFSET)

_Static_assert(sizeof(gip_map_data_t) == 256, "gip_map_data_t must be one page");

// ============================================================================
// CRC-32 (standard reflected polynomial 0xEDB88320)
// ============================================================================

static uint32_t crc32_compute(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

// ============================================================================
// Flash helpers
// ============================================================================

typedef struct {
    uint32_t offset;
    uint32_t length;
} erase_params_t;

typedef struct {
    uint32_t offset;
    const uint8_t* data;
    uint32_t length;
} program_params_t;

static void __no_inline_not_in_flash_func(do_flash_erase)(void* param)
{
    erase_params_t* p = (erase_params_t*)param;
    flash_range_erase(p->offset, p->length);
}

static void __no_inline_not_in_flash_func(do_flash_program)(void* param)
{
    program_params_t* p = (program_params_t*)param;
    flash_range_program(p->offset, p->data, p->length);
}

// Same reasoning as ps4_auth_flash.c: do not fall back to a direct erase with
// interrupts off when flash_safe_execute() fails, because Core 1 is not parked
// and would fault while executing from XIP.
static bool run_flash_op(void (*worker)(void*), void* param, const char* what)
{
    int result = flash_safe_execute(worker, param, UINT32_MAX);
    if (result == PICO_OK) return true;
    printf("[gip_map] flash_safe_execute failed for %s (%d)\n", what, result);
    return false;
}

static bool gip_map_flash_is_valid(const gip_map_data_t* data)
{
    if (!data) return false;
    if (data->magic != GIP_MAP_FLASH_MAGIC) return false;
    return data->crc32 ==
           crc32_compute((const uint8_t*)data + CRC_OFFSET, CRC_LENGTH);
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool gip_map_flash_load(gip_map_data_t* out)
{
    if (!out) return false;

    const gip_map_data_t* flash_ptr =
        (const gip_map_data_t*)(XIP_BASE + FLASH_GIP_MAP_OFFSET);

    if (flash_ptr->magic != GIP_MAP_FLASH_MAGIC) {
        return false;
    }

    memcpy(out, flash_ptr, sizeof(gip_map_data_t));
    if (!gip_map_flash_is_valid(out)) {
        memset(out, 0, sizeof(gip_map_data_t));
        return false;
    }
    return true;
}

bool gip_map_flash_save(const gip_map_data_t* data)
{
    if (!data) return false;

    static gip_map_data_t write_buf;
    memset(&write_buf, 0, sizeof(write_buf));
    memcpy(&write_buf, data, sizeof(gip_map_data_t));
    write_buf.magic = GIP_MAP_FLASH_MAGIC;
    write_buf.crc32 =
        crc32_compute((const uint8_t*)&write_buf + CRC_OFFSET, CRC_LENGTH);

    erase_params_t ep = { .offset = FLASH_GIP_MAP_OFFSET,
                          .length = FLASH_SECTOR_SIZE };
    if (!run_flash_op(do_flash_erase, &ep, "erase")) {
        printf("[gip_map] NOT saved - erase did not run\n");
        return false;
    }

    program_params_t pp = { .offset = FLASH_GIP_MAP_OFFSET,
                            .data = (const uint8_t*)&write_buf,
                            .length = sizeof(gip_map_data_t) };
    if (!run_flash_op(do_flash_program, &pp, "program")) {
        printf("[gip_map] NOT saved - program did not run\n");
        return false;
    }

    // A PICO_OK only says the worker ran. Take the verdict from the sector.
    const gip_map_data_t* flash_ptr =
        (const gip_map_data_t*)(XIP_BASE + FLASH_GIP_MAP_OFFSET);
    if (!gip_map_flash_is_valid(flash_ptr)) {
        printf("[gip_map] NOT saved - read-back failed validation\n");
        return false;
    }

    printf("[gip_map] Saved to flash (read-back verified)\n");
    return true;
}

bool gip_map_flash_erase(void)
{
    erase_params_t ep = { .offset = FLASH_GIP_MAP_OFFSET,
                          .length = FLASH_SECTOR_SIZE };
    if (!run_flash_op(do_flash_erase, &ep, "erase")) {
        return false;
    }
    const gip_map_data_t* flash_ptr =
        (const gip_map_data_t*)(XIP_BASE + FLASH_GIP_MAP_OFFSET);
    return flash_ptr->magic != GIP_MAP_FLASH_MAGIC;
}

void gip_map_init(void)
{
    gip_map_data_t stored;
    if (!gip_map_flash_load(&stored)) {
        printf("[gip_map] No stored button map - using the compiled-in layout\n");
        return;
    }
    if (!gip_button_map_validate(stored.map)) {
        printf("[gip_map] Stored button map is not usable - keeping defaults\n");
        return;
    }
    if (!gip_button_map_set(stored.map)) {
        printf("[gip_map] Stored button map rejected - keeping defaults\n");
        return;
    }
    printf("[gip_map] Button map loaded from flash\n");
}
