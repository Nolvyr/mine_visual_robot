#ifndef UART_LINK_H
#define UART_LINK_H
#include "main.h"
#include "v3_protocol.h"
/* ASCII diagnostics on PA9; disable when the upstream RX becomes binary. */
#ifndef UART_LINK_DEBUG_HEX
#define UART_LINK_DEBUG_HEX 1
#endif
typedef struct {
    uint32_t rx_bytes, uart_errors, overflows, restarts, start_failures;
    uint32_t timeout_resets, debug_dropped, tx_errors;
    uint32_t frames, crc_errors, header_errors, last_sequence;
    uint16_t last_length;
    uint8_t last_frame[V3_MAX_FRAME];
} UART_LinkStats;
extern volatile UART_LinkStats uart_link_stats;
void UART_Link_Init(void);
void UART_Link_Poll(void);
/* Runs in main context. Frame memory is only valid during this call. */
void UART_Link_OnFrame(const uint8_t *frame, uint16_t length);
#endif
