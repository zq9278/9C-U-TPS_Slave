#include "boot_engine.h"
#include <string.h>
int Boot_ValidApp(const BootFlash *f) {
    const uint8_t *a=f->read(UP_META_A), *b=f->read(UP_META_B), *r;
    int va=Up_CheckMetadata(a), vb=Up_CheckMetadata(b);
    if(!va && !vb) return 0;
    if(va && vb && memcmp(a,b,64)) return 0;
    r=va?a:b;
    const uint8_t *image=f->read(UP_APP_BASE);
    uint32_t size=Up_Get32(r+20);
    return Up_CheckVectors(image,size) && Up_Crc32(image,size)==Up_Get32(r+24) &&
           (!f->healthy || f->healthy());
}
void Boot_Init(BootEngine *b,BootFlash f) {
    memset(b,0,sizeof(*b)); b->flash=f; b->app_valid=(uint8_t)Boot_ValidApp(&f);
}
/* Flash callbacks return success only after read-back. */
static int commit(BootEngine *b) {
    uint8_t record[64]; Up_MakeMetadata(record,b->header);
    const uint32_t addresses[]={UP_META_A,UP_META_B};
    for(unsigned i=0;i<2;i++) {
        const uint8_t *old=b->flash.read(addresses[i]);
        if(!memcmp(old,record,64)) continue;
        if(!b->flash.write(addresses[i],record,56) ||
           !b->flash.write(addresses[i]+56,record+56,8)) return 0;
    }
    return Boot_ValidApp(&b->flash);
}
static unsigned execute(BootEngine *b,const UpFrame *f) {
    switch(f->command) {
    case UP_BEGIN: {
        if(f->length!=64) return UP_BAD_LENGTH;
        if(b->state!=UP_READY && b->state!=UP_VERIFIED) return UP_BAD_STATE;
        unsigned status=Up_CheckHeader(f->payload); if(status) return status;
        memcpy(b->header,f->payload,64);
        b->state=UP_ERROR; b->app_valid=0;
        if(!b->flash.erase(UP_META_A,2) ||
           Up_CheckMetadata(b->flash.read(UP_META_A)) || Up_CheckMetadata(b->flash.read(UP_META_B)) ||
           !b->flash.erase(UP_APP_BASE,UP_APP_SIZE/2048)) return UP_FLASH_ERROR;
        b->offset=0; b->state=UP_RECEIVING; return UP_OK;
    }
    case UP_DATA: {
        if(b->state!=UP_RECEIVING) return UP_BAD_STATE;
        if(f->length<=4 || f->length>260) return UP_BAD_LENGTH;
        uint32_t offset=Up_Get32(f->payload), size=Up_Get32(b->header+24), n=f->length-4U;
        if(offset!=b->offset || (offset&7U)) return UP_BAD_OFFSET;
        if(offset>size || n>size-offset) return UP_OUT_OF_RANGE;
        if(n!=256 && n!=size-offset) return UP_BAD_LENGTH;
        if(!b->flash.write(UP_APP_BASE+offset,f->payload+4,n)) {
            b->state=UP_ERROR; return UP_FLASH_ERROR;
        }
        b->offset+=n; return UP_OK;
    }
    case UP_END: {
        if(f->length) return UP_BAD_LENGTH;
        if(b->state!=UP_RECEIVING) return UP_BAD_STATE;
        uint32_t size=Up_Get32(b->header+24);
        if(b->offset!=size) return UP_BAD_OFFSET;
        const uint8_t *image=b->flash.read(UP_APP_BASE);
        if(Up_Crc32(image,size)!=Up_Get32(b->header+28)) return UP_IMAGE_CRC_ERROR;
        if(!Up_CheckVectors(image,size)) return UP_BAD_VECTOR;
        if(b->flash.healthy && !b->flash.healthy()) { b->state=UP_ERROR; return UP_FLASH_ERROR; }
        if(!commit(b)) { b->state=UP_ERROR; return UP_FLASH_ERROR; }
        b->app_valid=1; b->state=UP_VERIFIED; return UP_OK;
    }
    case UP_RUN:
        if(f->length) return UP_BAD_LENGTH;
        if(!b->app_valid || !Boot_ValidApp(&b->flash)) return UP_BAD_STATE;
        b->reset_requested=1; return UP_OK;
    case UP_ABORT:
        if(f->length) return UP_BAD_LENGTH;
        b->offset=0; b->state=UP_READY; return UP_OK;
    default: return UP_BAD_STATE;
    }
}
size_t Boot_Handle(BootEngine *b,const UpFrame *f) {
    uint8_t payload[48]={0}, request[UP_MAX_FRAME];
    size_t extra=0;
    unsigned status=UP_OK;
    uint32_t reply_session=b->session;
    if(f->command==UP_HELLO) {
        if(f->length!=4 || f->session!=0 || Up_Get32(f->payload)==0 || !f->seq) status=UP_BAD_LENGTH;
        else if(b->session && b->session==Up_Get32(f->payload)) {
            if(b->hello_seq==f->seq && !memcmp(f->payload,b->hello_request,4)) {
                memcpy(b->response,b->hello_response,b->hello_size); return b->hello_size;
            }
            status=UP_BAD_SEQUENCE;
        } else {
            b->session=Up_Get32(f->payload); reply_session=b->session; b->state=UP_READY;
            /* A new host process can recover an abandoned transfer. Only RAM
             * state is discarded; BEGIN erases Flash. Old-session DATA fails. */
            b->last_request_size=0; b->last_response_size=0; b->offset=0; b->reset_requested=0;
            b->hello_seq=f->seq; b->last_seq=f->seq; memcpy(b->hello_request,f->payload,4);
            Up_Put32(payload+8,UP_TARGET); Up_Put16(payload+12,UP_HW_REV); Up_Put16(payload+14,256);
            Up_Put32(payload+16,UP_BOOT_VERSION); Up_Put32(payload+20,UP_APP_BASE);
            Up_Put32(payload+24,UP_APP_SIZE); payload[28]=b->app_valid;
            if(b->app_valid) {
                const uint8_t *r=b->flash.read(UP_META_A);
                if(!Up_CheckMetadata(r)) r=b->flash.read(UP_META_B);
                Up_Put32(payload+32,Up_Get32(r+28));
            }
            extra=28;
        }
    } else if(!b->session || f->session!=b->session) status=UP_BAD_SESSION;
    else if(f->command==UP_STATUS) {
        if(f->length) status=UP_BAD_LENGTH;
    } else {
        size_t request_size=Up_Encode(request,f->command,f->seq,f->session,f->payload,f->length);
        if(b->last_request_size && f->seq==b->last_seq) {
            if(request_size==b->last_request_size && !memcmp(request,b->last_request,request_size)) {
                memcpy(b->response,b->last_response,b->last_response_size); return b->last_response_size;
            }
            status=UP_BAD_SEQUENCE;
        } else if(!f->seq || f->seq<=b->last_seq) status=UP_BAD_SEQUENCE;
        else {
            status=execute(b,f);
            b->last_seq=f->seq;
            memcpy(b->last_request,request,request_size); b->last_request_size=request_size;
            Up_Put16(payload,(uint16_t)status); payload[2]=b->state; Up_Put32(payload+4,b->offset);
            b->last_response_size=Up_Encode(b->last_response,f->command|0x80,f->seq,b->session,payload,8);
            memcpy(b->response,b->last_response,b->last_response_size); return b->last_response_size;
        }
    }
    Up_Put16(payload,(uint16_t)status); payload[2]=b->state; Up_Put32(payload+4,b->offset);
    size_t n=Up_Encode(b->response,f->command|0x80,f->seq,reply_session,payload,(uint16_t)(8+extra));
    if(f->command==UP_HELLO && status==UP_OK) { memcpy(b->hello_response,b->response,n); b->hello_size=n; }
    return n;
}
