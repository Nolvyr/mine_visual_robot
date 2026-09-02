/* HAL mock exercises the actual driver without pretending to test hardware. */
#define __MAIN_H
#define __USART_H__
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#define __weak
#define __DMB() ((void)0)
#define __disable_irq() ((void)0)
#define __get_PRIMASK() 0U
#define __set_PRIMASK(x) ((void)(x))
#define HAL_OK 0
#define HAL_UART_STATE_READY 0
typedef struct { int gState; void *hdmarx; } UART_HandleTypeDef;
static UART_HandleTypeDef huart1;
static uint16_t remaining=512, cursor;
static uint8_t *rx_memory;
static uint32_t tick;
static unsigned starts, aborts, sends;
static int fail_start;
#define __HAL_DMA_GET_COUNTER(x) (remaining)
static uint32_t HAL_GetTick(void) { return tick; }
static int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *b,uint16_t n)
{ (void)u; assert(n==512); rx_memory=b; cursor=0; remaining=n; ++starts; return fail_start; }
static int HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *b,uint16_t n)
{ assert(n<700 && memcmp(b,"V3 len=",7)==0); u->gState=1; ++sends; return HAL_OK; }
static int HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; ++aborts; return HAL_OK; }
#include "../Core/Src/uart_link.c"
static void inject(const uint8_t *b,unsigned n,int idle)
{
    unsigned i;
    for(i=0;i<n;++i) {
        rx_memory[cursor++]=b[i]; remaining=(uint16_t)(512-cursor);
        if(cursor==256 || cursor==512) HAL_UARTEx_RxEventCallback(&huart1,cursor);
        if(cursor==512) { cursor=0; remaining=512; }
    }
    if(idle) HAL_UARTEx_RxEventCallback(&huart1,cursor);
}
int main(void)
{
    uint8_t f[200]={0xaa,0x55,3,1,1,0xf0,1,0,0,0,3,185,0};
    uint16_t crc=V3_Crc(f,198);
    unsigned i;
    uint32_t count, bytes;
    f[198]=(uint8_t)crc; f[199]=(uint8_t)(crc>>8);
    UART_Link_Init(); assert(starts==1);
    inject(f,73,1); UART_Link_Poll(); assert(uart_link_stats.frames==0);
    inject(f+73,127,1); UART_Link_Poll(); assert(uart_link_stats.frames==1 && sends==1);
    assert(memcmp((const void *)uart_link_stats.last_frame,f,200)==0);
    inject(f,200,1); UART_Link_Poll(); assert(uart_link_stats.frames==2 && uart_link_stats.debug_dropped==1);
    bytes=uart_link_stats.rx_bytes;
    HAL_UARTEx_RxEventCallback(&huart1,256); /* delayed HT after IDLE */
    assert(uart_link_stats.rx_bytes==bytes);
    for(i=0;i<100;++i) { inject(f,200,0); UART_Link_Poll(); }
    HAL_UARTEx_RxEventCallback(&huart1,cursor); UART_Link_Poll();
    assert(uart_link_stats.frames==102 && uart_link_stats.crc_errors==0);
    inject(f,40,1); UART_Link_Poll(); tick+=251; UART_Link_Poll();
    assert(uart_link_stats.timeout_resets==1);
    HAL_UART_ErrorCallback(&huart1); UART_Link_Poll(); assert(aborts==1 && starts==2);
    inject(f,200,1); UART_Link_Poll(); assert(uart_link_stats.frames==103);
    for(i=0;i<15;++i) inject(f,200,1);
    assert(uart_link_stats.overflows==1); UART_Link_Poll();
    count=uart_link_stats.frames;
    inject(f,200,1); UART_Link_Poll(); assert(uart_link_stats.frames==count+1);
    fail_start=1; HAL_UART_ErrorCallback(&huart1); UART_Link_Poll();
    assert(uart_link_stats.start_failures==1);
    fail_start=0; UART_Link_Poll(); inject(f,200,1); UART_Link_Poll();
    assert(uart_link_stats.frames==count+2);
    puts("PASS: DMA splits/wrap/continuous input, stale HT, nonblocking debug, timeout, UART error/restart, overflow, start failure recovery");
    return 0;
}
