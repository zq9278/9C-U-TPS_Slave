#ifndef TPS_UPGRADE_H
#define TPS_UPGRADE_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define UP_APP_BASE 0x08004000UL
#define UP_APP_SIZE 110592UL
#define UP_META_A 0x0801F000UL
#define UP_META_B 0x0801F800UL
#define UP_RAM_END 0x20008FE0UL
#define UP_MAILBOX 0x20008FE0UL
#define UP_MAILBOX_MAGIC 0x31504455UL
#define UP_TARGET 0x00009C01UL
#define UP_BOOT_VERSION 0x00010000UL
#ifndef UP_HW_REV
#define UP_HW_REV 1U
#endif
#define UP_MAX_PAYLOAD 264U
#define UP_MAX_FRAME (18U + UP_MAX_PAYLOAD)
enum { UP_HELLO=1, UP_BEGIN, UP_DATA, UP_END, UP_RUN, UP_STATUS, UP_ABORT };
enum { UP_WAIT, UP_READY, UP_RECEIVING, UP_VERIFIED, UP_ERROR };
enum { UP_OK, UP_BAD_STATE, UP_BAD_LENGTH, UP_BAD_SESSION, UP_BAD_SEQUENCE,
       UP_BAD_TARGET, UP_BAD_VERSION, UP_OUT_OF_RANGE, UP_BAD_OFFSET,
       UP_FLASH_ERROR, UP_IMAGE_CRC_ERROR, UP_BAD_VECTOR, UP_BUSY, UP_BAD_HEADER };
typedef struct {
    uint8_t bytes[UP_MAX_FRAME];
    uint16_t used;
    uint32_t last_tick;
} UpParser;
typedef struct {
    uint8_t command;
    uint16_t seq, length;
    uint32_t session;
    const uint8_t *payload;
} UpFrame;
uint16_t Up_Get16(const uint8_t *p);
uint32_t Up_Get32(const uint8_t *p);
void Up_Put16(uint8_t *p, uint16_t v);
void Up_Put32(uint8_t *p, uint32_t v);
uint32_t Up_Crc32(const void *data, size_t length);
size_t Up_Encode(uint8_t *out, uint8_t cmd, uint16_t seq, uint32_t session,
                 const uint8_t *payload, uint16_t length);
/* Frame payload remains valid until the next parser call. */
int Up_Parse(UpParser *p, uint8_t byte, uint32_t now, UpFrame *frame);
int Up_CheckHeader(const uint8_t *header);
int Up_CheckVectors(const uint8_t *image, uint32_t size);
void Up_MakeMetadata(uint8_t out[64], const uint8_t header[64]);
int Up_CheckMetadata(const uint8_t record[64]);
#ifdef __cplusplus
}
#endif
#endif
