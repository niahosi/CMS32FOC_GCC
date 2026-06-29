#pragma once

#include <stdint.h>
#include "CMS32M6510_MemoryMap.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CMS32_DATA_BLOCK_MAGIC (0x33464344UL)
#define CMS32_DATA_BLOCK_VERSION (1U)
#define CMS32_DATA_BLOCK_PAYLOAD_SIZE (96U)
#define CMS32_DATA_BLOCK_A_ADDR (CMS32_DATA_FLASH_ORIGIN)
#define CMS32_DATA_BLOCK_B_ADDR (CMS32_DATA_FLASH_ORIGIN + CMS32_DATA_FLASH_SECTOR_SIZE)

#define CMS32_BOOT_REQUEST_MAGIC (0x544F4F42UL)
#define CMS32_APP_VALID_MAGIC (0x50415041UL)

typedef struct
{
    int16_t iu_offset_cnt;
    int16_t iv_offset_cnt;
    int16_t iw_offset_cnt;
    uint16_t pole_pairs;
    uint16_t encoder_cpr;
    int16_t encoder_offset;
    uint16_t reserved0;
    uint32_t boot_request;
    uint32_t app_valid;
    uint32_t app_size;
    uint32_t app_crc;
    uint32_t update_sequence;
    uint8_t reserved[CMS32_DATA_BLOCK_PAYLOAD_SIZE - 36U];
} CMS32_DataFlashPayload_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t sequence;
    CMS32_DataFlashPayload_t payload;
    uint32_t crc32;
} CMS32_DataFlashBlock_t;

#ifdef __cplusplus
}
#endif
