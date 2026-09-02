#ifndef V3_PROTOCOL_H
#define V3_PROTOCOL_H
#include <stdint.h>
#define V3_MAX_FRAME 200U
typedef void (*V3_FrameCallback)(const uint8_t *frame, uint16_t length);
typedef struct {
    uint8_t data[V3_MAX_FRAME];
    uint16_t used;
    uint32_t frames, crc_errors, header_errors, discarded;
    V3_FrameCallback callback;
} V3_Parser;
void V3_Init(V3_Parser *p, V3_FrameCallback callback);
void V3_Reset(V3_Parser *p);
void V3_Feed(V3_Parser *p, uint8_t byte);
uint16_t V3_Crc(const uint8_t *data, uint16_t length);
#endif
