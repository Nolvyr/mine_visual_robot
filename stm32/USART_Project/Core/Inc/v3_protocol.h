#ifndef V3_PROTOCOL_H
#define V3_PROTOCOL_H
#include <stdint.h>
#define V3_MAX_FRAME 200U
#define V3_MESSAGE_ACK 0x30U
#define V3_ACK_RECEIVED 0x00U
#define V3_ACK_FRAME_SIZE 21U
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
uint16_t V3_BuildAck(const uint8_t *received_frame, uint8_t *ack_frame);
#endif
