#include "v3_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned calls;
static uint16_t last_length;
static uint8_t last[200];
static void received(const uint8_t *f, uint16_t n)
{ ++calls; last_length=n; memcpy(last,f,n); }
static void feed(V3_Parser *p, const uint8_t *f, unsigned n)
{ unsigned i; for(i=0;i<n;++i) V3_Feed(p,f[i]); }
static void finish(uint8_t *f, unsigned n)
{ uint16_t c=V3_Crc(f,(uint16_t)(n-2)); f[n-2]=(uint8_t)c; f[n-1]=(uint8_t)(c>>8); }
int main(void)
{
    V3_Parser p;
    unsigned i, split;
    uint8_t golden[191]={0xaa,0x55,3,0x10,1,0xf0,4,3,2,1,3,0xb0,0,0x78,0x56,0x34,0x12,2,0,13};
    uint8_t bad[200], boundary[200]={0xaa,0x55,3,1,1,0xf0};
    assert(V3_Crc((const uint8_t *)"123456789",9)==0x29b1);
    for(i=1;i<=13;++i) {
        unsigned o=20+(i-1)*13;
        int y=-200-(int)i;
        golden[o]=1; golden[o+1]=2; golden[o+3]=(uint8_t)i;
        golden[o+7]=(uint8_t)(100+i);
        golden[o+9]=(uint8_t)y; golden[o+10]=0xff;
        golden[o+11]=(uint8_t)(300+i); golden[o+12]=1;
    }
    /* Fixed trailer copied from the S100 golden, not calculated by the parser. */
    golden[189]=0x59; golden[190]=0x46;
    for(split=0;split<=191;++split) {
        V3_Init(&p,received); calls=0;
        feed(&p,golden,split); feed(&p,golden+split,191-split);
        assert(calls==1 && last_length==191 && memcmp(last,golden,191)==0);
    }
    V3_Init(&p,received); calls=0;
    V3_Feed(&p,0); V3_Feed(&p,0xaa); feed(&p,golden,191); feed(&p,golden,191);
    assert(calls==2);
    memcpy(bad,golden,191); bad[40]^=1; feed(&p,bad,191); feed(&p,golden,191);
    assert(calls==3 && p.crc_errors>0);
    memcpy(bad,golden,191); bad[11]=255; bad[12]=255; feed(&p,bad,191); feed(&p,golden,191);
    assert(calls==4 && p.header_errors>0);
    memcpy(bad,golden,191); bad[2]=2; feed(&p,bad,191); feed(&p,golden,191); assert(calls==5);
    boundary[11]=185; finish(boundary,200); feed(&p,boundary,200); assert(last_length==200);
    boundary[11]=0; finish(boundary,15); feed(&p,boundary,15); assert(last_length==15);
    feed(&p,golden,30); V3_Reset(&p); feed(&p,golden,191); assert(last_length==191);
    for(i=0;i<100000;++i) V3_Feed(&p,(uint8_t)(i*71));
    V3_Reset(&p); feed(&p,golden,191); assert(p.used==0);
    puts("PASS: S100 191B golden CRC=4659, all splits, concatenation, noise, CRC/version/length recovery, 15/200B, reset, long noise");
    return 0;
}
