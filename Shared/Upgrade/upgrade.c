#include "upgrade.h"
#include <string.h>
uint16_t Up_Get16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
uint32_t Up_Get32(const uint8_t *p) { return (uint32_t)Up_Get16(p) | ((uint32_t)Up_Get16(p+2)<<16); }
void Up_Put16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
void Up_Put32(uint8_t *p,uint32_t v) { Up_Put16(p,(uint16_t)v); Up_Put16(p+2,(uint16_t)(v>>16)); }
uint32_t Up_Crc32(const void *data,size_t n) {
    const uint8_t *p=data; uint32_t c=0xFFFFFFFFU;
    while(n--) { c^=*p++; for(unsigned i=0;i<8;i++) c=(c>>1)^((c&1U)?0xEDB88320U:0U); }
    return c^0xFFFFFFFFU;
}
size_t Up_Encode(uint8_t *out,uint8_t cmd,uint16_t seq,uint32_t session,const uint8_t *data,uint16_t n) {
    if(n>UP_MAX_PAYLOAD) return 0;
    memcpy(out,"TPSU",4); out[4]=1; out[5]=cmd;
    Up_Put16(out+6,seq); Up_Put32(out+8,session); Up_Put16(out+12,n);
    if(n) memcpy(out+14,data,n);
    Up_Put32(out+14+n,Up_Crc32(out+4,10U+n)); return 18U+n;
}
static void shift(UpParser *p) {
    --p->used; memmove(p->bytes,p->bytes+1,p->used);
}
int Up_Parse(UpParser *p,uint8_t byte,uint32_t now,UpFrame *f) {
    if((uint32_t)(now-p->last_tick)>200U) p->used=0;
    p->last_tick=now;
    if(p->used==sizeof(p->bytes)) p->used=0;
    p->bytes[p->used++]=byte;
    while(p->used) {
        size_t prefix=p->used<4?p->used:4;
        if(memcmp(p->bytes,"TPSU",prefix)) { shift(p); continue; }
        if(p->used<14) return 0;
        uint16_t n=Up_Get16(p->bytes+12);
        if(p->bytes[4]!=1 || n>UP_MAX_PAYLOAD) { shift(p); continue; }
        if(p->used<18U+n) return 0;
        if(Up_Get32(p->bytes+14+n)!=Up_Crc32(p->bytes+4,10U+n)) { shift(p); continue; }
        f->command=p->bytes[5]; f->seq=Up_Get16(p->bytes+6);
        f->session=Up_Get32(p->bytes+8); f->length=n; f->payload=p->bytes+14;
        p->used=0; return 1;
    }
    return 0;
}
int Up_CheckHeader(const uint8_t *h) {
    if(memcmp(h,"TPSFW001",8) || Up_Get16(h+8)!=64 || Up_Get16(h+10)!=1 ||
       Up_Get32(h+60)!=Up_Crc32(h,60) || Up_Get32(h+40)!=0) return UP_BAD_HEADER;
    for(unsigned i=44;i<60;i++) if(h[i]) return UP_BAD_HEADER;
    if(Up_Get32(h+12)!=UP_TARGET || Up_Get16(h+16)>UP_HW_REV || Up_Get16(h+18)<UP_HW_REV)
        return UP_BAD_TARGET;
    if(Up_Get32(h+36)>UP_BOOT_VERSION) return UP_BAD_VERSION;
    if(Up_Get32(h+20)!=UP_APP_BASE || Up_Get32(h+24)<8 || Up_Get32(h+24)>UP_APP_SIZE)
        return UP_OUT_OF_RANGE;
    return UP_OK;
}
int Up_CheckVectors(const uint8_t *image,uint32_t size) {
    uint32_t sp=Up_Get32(image), pc=Up_Get32(image+4);
    return size>=8 && size<=UP_APP_SIZE && sp>0x20000000UL && sp<=UP_RAM_END && !(sp&7U) &&
           (pc&1U) && (pc&~1UL)>=UP_APP_BASE && (pc&~1UL)<UP_APP_BASE+size;
}
void Up_MakeMetadata(uint8_t r[64],const uint8_t h[64]) {
    memset(r,0,64); memcpy(r,"TPSM",4); Up_Put16(r+4,1); Up_Put16(r+6,64);
    Up_Put32(r+8,1); memcpy(r+12,h+12,4); memcpy(r+16,h+20,16);
    memcpy(r+32,h+16,4); Up_Put32(r+52,Up_Crc32(r,52)); memcpy(r+56,"TPSVALID",8);
}
int Up_CheckMetadata(const uint8_t r[64]) {
    if(memcmp(r,"TPSM",4) || Up_Get16(r+4)!=1 || Up_Get16(r+6)!=64 ||
       memcmp(r+56,"TPSVALID",8) || Up_Get32(r+52)!=Up_Crc32(r,52) ||
       Up_Get32(r+12)!=UP_TARGET || Up_Get32(r+16)!=UP_APP_BASE ||
       Up_Get32(r+20)<8 || Up_Get32(r+20)>UP_APP_SIZE ||
       Up_Get16(r+32)>UP_HW_REV || Up_Get16(r+34)<UP_HW_REV) return 0;
    for(unsigned i=36;i<52;i++) if(r[i]) return 0;
    return 1;
}
