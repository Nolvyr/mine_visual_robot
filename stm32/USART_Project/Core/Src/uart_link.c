#include "uart_link.h"
#include "usart.h"
#include <stdio.h>

#define DMA_SIZE 512U
#define RING_SIZE 2048U
#define RING_MASK (RING_SIZE - 1U)
#define PARTIAL_TIMEOUT_MS 250U
static uint8_t dma_rx[DMA_SIZE], ring[RING_SIZE];
static volatile uint16_t head, tail;
static uint16_t dma_position;
static volatile uint8_t fault;
static V3_Parser parser;
static uint32_t last_activity;
#if UART_LINK_DEBUG_HEX
static uint8_t debug_tx[700];
#endif
volatile UART_LinkStats uart_link_stats;

static void frame_received(const uint8_t *frame, uint16_t length)
{
    uint16_t i;
#if UART_LINK_DEBUG_HEX
    int n;
    static const char hex[] = "0123456789ABCDEF";
#endif
    uart_link_stats.last_length = length;
    uart_link_stats.last_sequence = (uint32_t)frame[6] | ((uint32_t)frame[7] << 8)
        | ((uint32_t)frame[8] << 16) | ((uint32_t)frame[9] << 24);
    for (i = 0; i < length; ++i) uart_link_stats.last_frame[i] = frame[i];
    UART_Link_OnFrame(frame, length);
#if UART_LINK_DEBUG_HEX
    /* Never overwrite a DMA TX buffer while hardware still owns it. */
    if (huart1.gState != HAL_UART_STATE_READY) { ++uart_link_stats.debug_dropped; return; }
    n = snprintf((char *)debug_tx, sizeof(debug_tx), "V3 len=%u seq=%lu CRC OK\r\n",
                 (unsigned)length, (unsigned long)uart_link_stats.last_sequence);
    if (n < 0 || n > 90) { ++uart_link_stats.tx_errors; return; }
    for (i = 0; i < length; ++i) {
        debug_tx[n++] = hex[frame[i] >> 4];
        debug_tx[n++] = hex[frame[i] & 15];
        debug_tx[n++] = ' ';
    }
    debug_tx[n++] = '\r'; debug_tx[n++] = '\n';
    if (HAL_UART_Transmit_DMA(&huart1, debug_tx, (uint16_t)n) != HAL_OK)
        ++uart_link_stats.tx_errors;
#endif
}

static void start_rx(void)
{
    dma_position = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx, DMA_SIZE) != HAL_OK) {
        ++uart_link_stats.start_failures; fault = 1;
    }
    /* Keep HT and TC enabled: continuous input need not produce IDLE. */
}
void UART_Link_Init(void)
{
    V3_Init(&parser, frame_received);
    head = tail = 0; fault = 0;
    last_activity = HAL_GetTick();
    start_rx();
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    uint16_t next;
    if (uart != &huart1 || size > DMA_SIZE || fault) return;
    /* Sample the live cursor: a pending HT callback can follow an IDLE
       callback and carry an older Size. Never rewind to that stale value.
       USART1 and RX DMA IRQs have the same preemption priority. */
    size = (uint16_t)(DMA_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx));
    if (size == DMA_SIZE) size = 0;
    while (dma_position != size) {
        next = (uint16_t)((head + 1U) & RING_MASK);
        if (next == tail) { ++uart_link_stats.overflows; fault = 1; return; }
        ring[head] = dma_rx[dma_position];
        __DMB();
        head = next;
        dma_position = (uint16_t)((dma_position + 1U) % DMA_SIZE);
        ++uart_link_stats.rx_bytes;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) { ++uart_link_stats.uart_errors; fault = 1; }
}
void UART_Link_Poll(void)
{
    uint32_t irq, now = HAL_GetTick();
    uint8_t byte;
    if (fault) {
        /* Stop RX before dropping the damaged stream. TX is independent. */
        HAL_UART_AbortReceive(&huart1);
        irq = __get_PRIMASK(); __disable_irq();
        head = tail = 0; fault = 0;
        __set_PRIMASK(irq);
        V3_Reset(&parser);
        ++uart_link_stats.restarts;
        start_rx();
        last_activity = now;
        return;
    }
    if (head == tail && parser.used && (uint32_t)(now-last_activity) >= PARTIAL_TIMEOUT_MS) {
        V3_Reset(&parser); ++uart_link_stats.timeout_resets;
    }
    while (tail != head && !fault) {
        byte = ring[tail];
        __DMB();
        tail = (uint16_t)((tail + 1U) & RING_MASK);
        V3_Feed(&parser, byte);
        last_activity = HAL_GetTick();
    }
    uart_link_stats.frames = parser.frames;
    uart_link_stats.crc_errors = parser.crc_errors;
    uart_link_stats.header_errors = parser.header_errors;
}
__weak void UART_Link_OnFrame(const uint8_t *frame, uint16_t length)
{
    (void)frame; (void)length;
}
