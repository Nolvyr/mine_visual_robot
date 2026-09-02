#include "uart_link.h"
#include "usart.h"

#define DMA_SIZE 512U
#define RING_SIZE 2048U
#define RING_MASK (RING_SIZE - 1U)
#define PARTIAL_TIMEOUT_MS 250U

static uint8_t dma_rx[DMA_SIZE];
static uint8_t receive_ring[RING_SIZE];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;
static uint16_t dma_position;
static volatile uint8_t receive_fault;
static V3_Parser parser;
static uint32_t last_activity;

/* A single pending ACK is sufficient for the current stop-and-observe test.
 * If traffic outruns USART TX, ack_dropped makes that visible. */
static uint8_t pending_ack[V3_ACK_FRAME_SIZE];
static uint16_t pending_ack_length;

volatile UART_LinkStats uart_link_stats;

static void QueueAck(const uint8_t *frame)
{
    if (frame[3] == V3_MESSAGE_ACK)
        return;

    if (pending_ack_length != 0U)
    {
        ++uart_link_stats.ack_dropped;
        return;
    }

    pending_ack_length = V3_BuildAck(frame, pending_ack);
}

static void FrameReceived(const uint8_t *frame, uint16_t length)
{
    uint16_t index;

    uart_link_stats.last_length = length;
    uart_link_stats.last_sequence = (uint32_t)frame[6]
        | ((uint32_t)frame[7] << 8)
        | ((uint32_t)frame[8] << 16)
        | ((uint32_t)frame[9] << 24);

    for (index = 0; index < length; ++index)
        uart_link_stats.last_frame[index] = frame[index];

    QueueAck(frame);
    UART_Link_OnFrame(frame, length);
}

static void StartReceive(void)
{
    dma_position = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx, DMA_SIZE) != HAL_OK)
    {
        ++uart_link_stats.start_failures;
        receive_fault = 1;
    }
}

static void SendPendingAck(void)
{
    uint16_t length;

    if (pending_ack_length == 0U || huart1.gState != HAL_UART_STATE_READY)
        return;

    length = pending_ack_length;
    if (HAL_UART_Transmit_DMA(&huart1, pending_ack, length) == HAL_OK)
    {
        pending_ack_length = 0U;
        ++uart_link_stats.ack_sent;
    }
    else
    {
        ++uart_link_stats.tx_errors;
    }
}

void UART_Link_Init(void)
{
    V3_Init(&parser, FrameReceived);
    ring_head = 0;
    ring_tail = 0;
    receive_fault = 0;
    pending_ack_length = 0;
    last_activity = HAL_GetTick();
    StartReceive();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    uint16_t next_head;

    if (uart != &huart1 || size > DMA_SIZE || receive_fault)
        return;

    /* Use the live DMA cursor. A delayed half-transfer callback may carry an
     * older Size value after an IDLE callback and must not rewind the stream. */
    size = (uint16_t)(DMA_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx));
    if (size == DMA_SIZE)
        size = 0;

    while (dma_position != size)
    {
        next_head = (uint16_t)((ring_head + 1U) & RING_MASK);
        if (next_head == ring_tail)
        {
            ++uart_link_stats.overflows;
            receive_fault = 1;
            return;
        }

        receive_ring[ring_head] = dma_rx[dma_position];
        __DMB();
        ring_head = next_head;
        dma_position = (uint16_t)((dma_position + 1U) % DMA_SIZE);
        ++uart_link_stats.rx_bytes;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        ++uart_link_stats.uart_errors;
        receive_fault = 1;
    }
}

static void RecoverReceivePath(uint32_t now)
{
    uint32_t interrupt_state;

    HAL_UART_AbortReceive(&huart1);
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    ring_head = 0;
    ring_tail = 0;
    receive_fault = 0;
    __set_PRIMASK(interrupt_state);

    V3_Reset(&parser);
    ++uart_link_stats.restarts;
    StartReceive();
    last_activity = now;
}

void UART_Link_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t byte;

    if (receive_fault)
    {
        RecoverReceivePath(now);
        return;
    }

    if (ring_head == ring_tail && parser.used
        && (uint32_t)(now - last_activity) >= PARTIAL_TIMEOUT_MS)
    {
        V3_Reset(&parser);
        ++uart_link_stats.timeout_resets;
    }

    while (ring_tail != ring_head && !receive_fault)
    {
        byte = receive_ring[ring_tail];
        __DMB();
        ring_tail = (uint16_t)((ring_tail + 1U) & RING_MASK);
        V3_Feed(&parser, byte);
        last_activity = HAL_GetTick();
    }

    uart_link_stats.frames = parser.frames;
    uart_link_stats.crc_errors = parser.crc_errors;
    uart_link_stats.header_errors = parser.header_errors;
    SendPendingAck();
}

__weak void UART_Link_OnFrame(const uint8_t *frame, uint16_t length)
{
    (void)frame;
    (void)length;
}
