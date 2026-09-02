#include "v3_protocol.h"
#include <string.h>

uint16_t V3_Crc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xffffU, i;
    uint8_t bit;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}
uint16_t V3_BuildAck(const uint8_t *received_frame, uint8_t *ack_frame)
{
    uint16_t crc;
    uint8_t i;
    if (received_frame == 0 || ack_frame == 0 || received_frame[3] == V3_MESSAGE_ACK ||
        received_frame[12] != 0U || received_frame[11] < 4U)
        return 0;
    ack_frame[0]=0xaaU; ack_frame[1]=0x55U; ack_frame[2]=3U; ack_frame[3]=V3_MESSAGE_ACK;
    ack_frame[4]=received_frame[5]; ack_frame[5]=received_frame[4];
    for (i=0;i<4U;++i) ack_frame[6U+i]=received_frame[6U+i];
    ack_frame[10]=0U; ack_frame[11]=6U; ack_frame[12]=0U;
    for (i=0;i<4U;++i) ack_frame[13U+i]=received_frame[13U+i];
    ack_frame[17]=received_frame[3]; ack_frame[18]=V3_ACK_RECEIVED;
    crc=V3_Crc(ack_frame,19U); ack_frame[19]=(uint8_t)crc; ack_frame[20]=(uint8_t)(crc>>8);
    return V3_ACK_FRAME_SIZE;
}
void V3_Init(V3_Parser *p, V3_FrameCallback callback)
{
    memset(p, 0, sizeof(*p));
    p->callback = callback;
}
void V3_Reset(V3_Parser *p) { p->used = 0; }
static void discard(V3_Parser *p, uint16_t n)
{
    p->used -= n;
    memmove(p->data, p->data + n, p->used);
}
void V3_Feed(V3_Parser *p, uint8_t byte)
{
    uint16_t payload, total, received;
    p->data[p->used++] = byte;
    for (;;) {
        if (!p->used) return;
        if (p->data[0] != 0xaaU || (p->used >= 2 && p->data[1] != 0x55U)) {
            ++p->discarded; discard(p, 1); continue;
        }
        if (p->used < 3) return;
        if (p->data[2] != 3) { ++p->header_errors; discard(p, 1); continue; }
        if (p->used < 13) return;
        payload = (uint16_t)(p->data[11] | ((uint16_t)p->data[12] << 8));
        if (payload > 185 || p->data[4] == 0xffU) {
            ++p->header_errors; discard(p, 1); continue;
        }
        total = (uint16_t)(15 + payload);
        if (p->used < total) return;
        received = (uint16_t)(p->data[total-2] | ((uint16_t)p->data[total-1] << 8));
        if (V3_Crc(p->data, total-2) != received) {
            ++p->crc_errors; discard(p, 1); continue;
        }
        ++p->frames;
        if (p->callback) p->callback(p->data, total);
        discard(p, total);
    }
}
