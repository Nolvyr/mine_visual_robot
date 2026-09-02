/* HAL mock executes the production UART link without claiming hardware coverage. */
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
static uint16_t remaining=512,cursor;
static uint8_t *rx_memory,transmitted[64];
static uint16_t transmitted_length;
static uint32_t tick;
static unsigned starts,aborts,sends;
static int fail_start;
#define __HAL_DMA_GET_COUNTER(x) (remaining)
static uint32_t HAL_GetTick(void){return tick;}
static int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*u,uint8_t*b,uint16_t n)
{(void)u;assert(n==512);rx_memory=b;cursor=0;remaining=n;++starts;return fail_start;}
static int HAL_UART_Transmit_DMA(UART_HandleTypeDef*u,uint8_t*b,uint16_t n)
{assert(n<=sizeof transmitted);memcpy(transmitted,b,n);transmitted_length=n;u->gState=1;++sends;return HAL_OK;}
static int HAL_UART_AbortReceive(UART_HandleTypeDef*u){(void)u;++aborts;return HAL_OK;}
#include "../Core/Src/uart_link.c"
static void inject(const uint8_t*b,unsigned n,int idle)
{unsigned i;for(i=0;i<n;++i){rx_memory[cursor++]=b[i];remaining=(uint16_t)(512-cursor);if(cursor==256||cursor==512)HAL_UARTEx_RxEventCallback(&huart1,cursor);if(cursor==512){cursor=0;remaining=512;}}if(idle)HAL_UARTEx_RxEventCallback(&huart1,cursor);}
static void finish(uint8_t*f,unsigned n){uint16_t c=V3_Crc(f,(uint16_t)(n-2));f[n-2]=(uint8_t)c;f[n-1]=(uint8_t)(c>>8);}
int main(void)
{
 uint8_t frame[200]={0xaa,0x55,3,0x10,1,0xf0,4,3,2,1,3,185,0,0x78,0x56,0x34,0x12};
 uint8_t expected[V3_ACK_FRAME_SIZE];uint32_t bytes;unsigned count;
 finish(frame,200);assert(V3_BuildAck(frame,expected)==V3_ACK_FRAME_SIZE);
 UART_Link_Init();assert(starts==1);
 inject(frame,73,1);UART_Link_Poll();assert(uart_link_stats.frames==0);
 inject(frame+73,127,1);UART_Link_Poll();
 assert(uart_link_stats.frames==1&&sends==1&&transmitted_length==V3_ACK_FRAME_SIZE);
 assert(memcmp(transmitted,expected,V3_ACK_FRAME_SIZE)==0&&uart_link_stats.ack_sent==1);
 bytes=uart_link_stats.rx_bytes;HAL_UARTEx_RxEventCallback(&huart1,256);assert(uart_link_stats.rx_bytes==bytes);
 /* TX busy: one ACK is queued; a further frame is counted as dropped. */
 inject(frame,200,1);UART_Link_Poll();assert(uart_link_stats.frames==2&&sends==1);
 inject(frame,200,1);UART_Link_Poll();assert(uart_link_stats.frames==3&&uart_link_stats.ack_dropped==1);
 huart1.gState=HAL_UART_STATE_READY;UART_Link_Poll();assert(sends==2&&uart_link_stats.ack_sent==2);
 inject(frame,40,1);UART_Link_Poll();tick+=251;UART_Link_Poll();assert(uart_link_stats.timeout_resets==1);
 HAL_UART_ErrorCallback(&huart1);UART_Link_Poll();assert(aborts==1&&starts==2);
 huart1.gState=HAL_UART_STATE_READY;inject(frame,200,1);UART_Link_Poll();assert(uart_link_stats.frames==4);
 for(count=0;count<15;++count)inject(frame,200,1);assert(uart_link_stats.overflows==1);UART_Link_Poll();
 fail_start=1;HAL_UART_ErrorCallback(&huart1);UART_Link_Poll();assert(uart_link_stats.start_failures==1);
 fail_start=0;UART_Link_Poll();
 puts("PASS: DMA split/wrap, binary ACK fields/CRC, TX busy queue/drop, timeout, UART error, overflow, restart");
 return 0;
}