/* Native test: actual firmware parser/engine with a simulated 2 KiB-page,
 * single-program-per-double-word Flash. No HAL or target board required. */
#include "Bootloader/boot_engine.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static uint8_t memory[131072], programmed[131072/8];
static int budget=-1, mutations;
static BootEngine engine;
static UpParser parser;
static uint8_t image[521],header[64];
static const uint8_t *read_flash(uint32_t a) { assert(a>=0x08000000 && a<0x08020000); return memory+a-0x08000000; }
static int step(void) { if(budget==0) return 0; if(budget>0) --budget; ++mutations; return 1; }
static int erase_flash(uint32_t a,uint32_t pages) {
    assert(a>=UP_APP_BASE && !(a&2047) && a+pages*2048<=0x08020000);
    for(uint32_t i=0;i<pages;i++) {
        if(!step()) return 0;
        memset(memory+a-0x08000000+i*2048,255,2048);
        memset(programmed+(a-0x08000000+i*2048)/8,0,256);
    }
    return 1;
}
static int write_flash(uint32_t a,const uint8_t *data,uint32_t n) {
    assert(a>=UP_APP_BASE && !(a&7) && a+((n+7)&~7U)<=0x08020000);
    for(uint32_t i=0;i<n;i+=8) {
        if(!step()) return 0;
        unsigned pos=a-0x08000000+i;
        assert(!programmed[pos/8]); programmed[pos/8]=1;
        uint8_t word[8]; memset(word,255,8); memcpy(word,data+i,n-i<8?n-i:8);
        memcpy(memory+pos,word,8);
    }
    return 1;
}
static int read_fault;
static int healthy(void) { return !read_fault; }
static BootFlash flash={read_flash,erase_flash,write_flash,healthy};
static void fresh(void) {
    memset(memory,255,sizeof(memory)); memset(programmed,0,sizeof(programmed));
    memset(memory,0xA5,16384); budget=-1; mutations=0;
    memset(&parser,0,sizeof(parser)); Boot_Init(&engine,flash);
}
static unsigned request(uint8_t command,uint16_t seq,uint32_t session,const uint8_t *data,uint16_t n) {
    uint8_t wire[UP_MAX_FRAME]; UpFrame frame={0};
    size_t length=Up_Encode(wire,command,seq,session,data,n);
    for(size_t i=0;i<length;i++) {
        int complete=Up_Parse(&parser,wire[i],100,&frame);
        assert(complete==(i==length-1));
    }
    size_t response_size=Boot_Handle(&engine,&frame);
    assert(response_size>=26 && Up_Get32(engine.response+response_size-4)==Up_Crc32(engine.response+4,response_size-8));
    return Up_Get16(engine.response+14);
}
static void hello(void) {
    uint8_t nonce[4]; Up_Put32(nonce,1234);
    assert(request(UP_HELLO,1,0,nonce,4)==UP_OK);
}
static unsigned data(unsigned offset,unsigned n,unsigned seq) {
    uint8_t payload[260]; Up_Put32(payload,offset); memcpy(payload+4,image+offset,n);
    return request(UP_DATA,(uint16_t)seq,1234,payload,(uint16_t)(n+4));
}
static void prepare_image(void) {
    for(unsigned i=0;i<sizeof(image);i++) image[i]=(uint8_t)i;
    Up_Put32(image,UP_RAM_END); Up_Put32(image+4,UP_APP_BASE+9);
    memset(header,0,64); memcpy(header,"TPSFW001",8);
    Up_Put16(header+8,64); Up_Put16(header+10,1); Up_Put32(header+12,UP_TARGET);
    Up_Put16(header+16,1); Up_Put16(header+18,1); Up_Put32(header+20,UP_APP_BASE);
    Up_Put32(header+24,sizeof(image)); Up_Put32(header+28,Up_Crc32(image,sizeof(image)));
    Up_Put32(header+32,0x01020300); Up_Put32(header+36,UP_BOOT_VERSION);
    Up_Put32(header+60,Up_Crc32(header,60));
}
static void full_upgrade(void) {
    assert(request(UP_BEGIN,2,1234,header,64)==UP_OK);
    assert(data(0,256,3)==UP_OK); assert(data(256,256,4)==UP_OK); assert(data(512,9,5)==UP_OK);
    assert(request(UP_END,6,1234,NULL,0)==UP_OK);
}
static void test_transfer(void) {
    fresh(); hello();
    assert(request(UP_BEGIN,2,999,header,64)==UP_BAD_SESSION);
    uint8_t wrong[64]; memcpy(wrong,header,64); Up_Put32(wrong+12,99); Up_Put32(wrong+60,Up_Crc32(wrong,60));
    assert(request(UP_BEGIN,2,1234,wrong,64)==UP_BAD_TARGET); assert(!mutations);
    assert(request(UP_BEGIN,3,1234,header,64)==UP_OK);
    int before=mutations;
    assert(request(UP_BEGIN,3,1234,header,64)==UP_OK && mutations==before);
    assert(request(UP_BEGIN,3,1234,wrong,64)==UP_BAD_SEQUENCE);
    assert(data(8,256,4)==UP_BAD_OFFSET);
    assert(data(0,256,5)==UP_OK); before=mutations;
    assert(request(UP_STATUS,6,1234,NULL,0)==UP_OK);
    assert(data(0,256,5)==UP_OK && mutations==before); /* STATUS preserves replay */
    assert(request(UP_END,7,1234,NULL,0)==UP_BAD_OFFSET);
    assert(data(256,256,8)==UP_OK); assert(data(512,9,9)==UP_OK);
    assert(request(UP_END,10,1234,NULL,0)==UP_OK); before=mutations;
    assert(request(UP_END,10,1234,NULL,0)==UP_OK && mutations==before);
    assert(Boot_ValidApp(&flash)); assert(!memcmp(read_flash(UP_APP_BASE),image,sizeof(image)));
    assert(read_flash(UP_APP_BASE)[521]==255 && read_flash(UP_APP_BASE)[527]==255);
    assert(request(UP_RUN,11,1234,NULL,0)==UP_OK && engine.reset_requested);
    read_fault=1; assert(!Boot_ValidApp(&flash)); read_fault=0;
    /* Redundant record can recover either copy; conflicting valid records fail closed. */
    memory[0x1F000]=0; assert(Boot_ValidApp(&flash));
    memory[0x1F000]='T'; memory[0x1F008]^=1;
    Up_Put32(memory+0x1F034,Up_Crc32(memory+0x1F000,52)); assert(!Boot_ValidApp(&flash));
    memory[0x1F800]=0; assert(Boot_ValidApp(&flash));
    memory[0x4008]^=1; assert(!Boot_ValidApp(&flash));
}
static void test_power_loss(void) {
    fresh(); hello(); full_upgrade(); int operations=mutations;
    /* Fail before each page erase or double-word programming operation, reboot,
     * and check the boot region and image validity; then recover by retransmit. */
    for(int cut=0;cut<=operations;cut++) {
        fresh(); hello(); budget=cut;
        unsigned s=request(UP_BEGIN,2,1234,header,64);
        if(s==UP_OK) s=data(0,256,3);
        if(s==UP_OK) s=data(256,256,4);
        if(s==UP_OK) s=data(512,9,5);
        if(s==UP_OK) s=request(UP_END,6,1234,NULL,0);
        Boot_Init(&engine,flash);
        for(unsigned i=0;i<16384;i++) assert(memory[i]==0xA5);
        if(engine.app_valid) assert(!memcmp(read_flash(UP_APP_BASE),image,sizeof(image)));
        budget=-1; hello(); full_upgrade(); assert(Boot_ValidApp(&flash));
    }
    /* Existing valid image + interruption invalidating metadata: only the
     * original complete image or no image may boot, never a partial overwrite. */
    uint8_t old[sizeof(memory)],old_programmed[sizeof(programmed)];
    memcpy(old,memory,sizeof(memory)); memcpy(old_programmed,programmed,sizeof(programmed));
    for(int cut=0;cut<4;cut++) {
        memcpy(memory,old,sizeof(memory)); memcpy(programmed,old_programmed,sizeof(programmed));
        Boot_Init(&engine,flash); hello(); budget=cut;
        (void)request(UP_BEGIN,2,1234,header,64); budget=-1;
        if(Boot_ValidApp(&flash)) assert(!memcmp(read_flash(UP_APP_BASE),image,sizeof(image)));
    }
    printf("power-loss injection: %d boundaries + existing image invalidation passed\n",operations+1);
}
static void test_parser(void) {
    uint8_t wire[UP_MAX_FRAME],payload[264]; memset(payload,0x55,sizeof(payload)); memcpy(payload+10,"TPSU",4);
    size_t n=Up_Encode(wire,UP_DATA,9,1234,payload,264);
    UpParser p={0}; UpFrame f;
    for(unsigned i=0;i<100;i++) assert(!Up_Parse(&p,0xFF,10,&f));
    for(unsigned pass=0;pass<2;pass++) for(size_t i=0;i<n;i++) {
        int result=Up_Parse(&p,wire[i],10,&f);
        assert(result==(i==n-1));
        if(result) assert(f.length==264 && !memcmp(f.payload,payload,264));
    }
    wire[n-1]^=1;
    for(size_t i=0;i<n;i++) assert(!Up_Parse(&p,wire[i],10,&f));
    wire[n-1]^=1;
    for(size_t i=0;i<n;i++) assert(Up_Parse(&p,wire[i],1000,&f)==(i==n-1));
    p=(UpParser){0};
    for(size_t i=0;i<12;i++) assert(!Up_Parse(&p,wire[i],10,&f));
    for(size_t i=0;i<n;i++) assert(Up_Parse(&p,wire[i],1000,&f)==(i==n-1));
    /* Oversize length, bad protocol version, then valid empty frame. */
    uint8_t bad[14]; memcpy(bad,wire,14); Up_Put16(bad+12,65535);
    for(size_t i=0;i<14;i++) assert(!Up_Parse(&p,bad[i],1000,&f));
    n=Up_Encode(wire,UP_END,10,1234,NULL,0);
    for(size_t i=0;i<n;i++) assert(Up_Parse(&p,wire[i],1000,&f)==(i==n-1));
    assert(f.length==0);
    assert(Up_Crc32("123456789",9)==0xCBF43926);
}
static void test_session_reconnect(void) {
    fresh(); hello();
    assert(request(UP_BEGIN,2,1234,header,64)==UP_OK);
    assert(data(0,256,3)==UP_OK);
    uint8_t nonce[4]; Up_Put32(nonce,5678); int count=mutations;
    assert(request(UP_HELLO,1,0,nonce,4)==UP_OK);
    assert(engine.session==5678 && engine.offset==0 && engine.state==UP_READY && mutations==count);
    assert(data(256,256,4)==UP_BAD_SESSION);
    assert(request(UP_BEGIN,2,5678,header,64)==UP_OK);
}
int main(int argc,char **argv) {
    if(argc==2 && !strcmp(argv[1],"--vectors")) {
        char line[UP_MAX_FRAME*2+3]; int count=0;
        while(fgets(line,sizeof(line),stdin)) {
            uint8_t bytes[UP_MAX_FRAME],encoded[UP_MAX_FRAME]; size_t n=strcspn(line,"\r\n")/2;
            for(size_t i=0;i<n;i++) { unsigned b; assert(sscanf(line+2*i,"%2x",&b)==1); bytes[i]=(uint8_t)b; }
            UpParser p={0}; UpFrame f;
            for(size_t i=0;i<n;i++) assert(Up_Parse(&p,bytes[i],1,&f)==(i==n-1));
            if(f.command==UP_BEGIN) assert(f.length==64 && Up_CheckHeader(f.payload)==UP_OK);
            assert(Up_Encode(encoded,f.command,f.seq,f.session,f.payload,f.length)==n);
            assert(!memcmp(encoded,bytes,n)); ++count;
        }
        printf("Python/C interop: %d wire vectors passed\n",count); return 0;
    }
    if(argc==3 && !strcmp(argv[1],"--factory")) {
        FILE *file=fopen(argv[2],"rb"); assert(file);
        assert(fread(memory,1,sizeof(memory),file)==sizeof(memory)); fclose(file);
        assert(Boot_ValidApp(&flash)); puts("factory image accepted by actual Boot validation"); return 0;
    }
    prepare_image(); test_parser(); test_transfer(); test_power_loss();
    test_session_reconnect();
    puts("boot parser, replay, offsets, metadata and recovery tests passed"); return 0;
}
