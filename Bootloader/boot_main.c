#include "boot_engine.h"
#include "main.h"
#include <string.h>
#if TPS_RTT
#include "SEGGER_RTT.h"
#define BOOT_LOG(text) SEGGER_RTT_WriteString(0,text)
#else
#define BOOT_LOG(text) ((void)0)
#endif
static UART_HandleTypeDef uart;
static BootEngine engine;
static UpParser parser;
static volatile uint8_t ecc_fault;
static int flash_healthy(void) {
    if(FLASH->ECCR & (FLASH_ECCR_ECCD|FLASH_ECCR_ECCC)) ecc_fault=1;
    return !ecc_fault;
}
void NMI_Handler(void) {
    if(FLASH->ECCR & FLASH_ECCR_ECCD) {
        /* A torn App/metadata double-word must not trap Boot indefinitely.
         * The validation that caused this read will fail closed. */
        ecc_fault=1;
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
        return;
    }
    for(;;) {}
}
static const uint8_t *flash_read(uint32_t address) { return (const uint8_t *)address; }
static int flash_erase(uint32_t address,uint32_t pages) {
    FLASH_EraseInitTypeDef erase={0}; uint32_t error;
    if(address<UP_APP_BASE || address>=0x08020000UL || (address&2047U) ||
       !pages || pages>(0x08020000UL-address)/2048U) return 0;
    erase.TypeErase=FLASH_TYPEERASE_PAGES; erase.Page=(address-FLASH_BASE)/2048U; erase.NbPages=pages;
    if(HAL_FLASH_Unlock()!=HAL_OK) return 0;
    FLASH->SR=FLASH_SR_ERRORS;
    int ok=HAL_FLASHEx_Erase(&erase,&error)==HAL_OK;
    HAL_FLASH_Lock();
    /* Clear a previous image's ECC latch only after a complete App erase. */
    if(ok && address==UP_APP_BASE && pages==UP_APP_SIZE/2048U) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCC);
        ecc_fault=0;
    }
    if(ok) for(uint32_t i=0;i<pages*2048U;i++) if(flash_read(address)[i]!=0xFF) return 0;
    if(address==UP_APP_BASE && !flash_healthy()) return 0;
    return ok;
}
static int flash_write(uint32_t address,const uint8_t *data,uint32_t n) {
    if(address<UP_APP_BASE || address>=0x08020000UL || (address&7U) || !n ||
       n>0x08020000UL-address) return 0;
    if(HAL_FLASH_Unlock()!=HAL_OK) return 0;
    FLASH->SR=FLASH_SR_ERRORS;
    int ok=1;
    for(uint32_t i=0;i<n;i+=8) {
        uint64_t word=UINT64_MAX;
        uint32_t count=(n-i<8)?n-i:8;
        memcpy(&word,data+i,count);
        /* Never program an already-programmed double-word (including ECC). */
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,address+i,word)!=HAL_OK ||
           memcmp(flash_read(address+i),&word,8)) { ok=0; break; }
    }
    HAL_FLASH_Lock(); return ok && flash_healthy();
}
/* Matches the current treatment idle route: pump/wave/left/right valves low.
 * PWM and fuse pins are held low as GPIO; no PWM timers run in Boot. */
static void safe_outputs(void) {
    GPIO_InitTypeDef pin={0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    pin.Mode=GPIO_MODE_OUTPUT_PP; pin.Pull=GPIO_NOPULL; pin.Speed=GPIO_SPEED_FREQ_LOW;
    pin.Pin=AirValve1_Pin|AirValve2_Pin|AirPump1_Pin|AirPump2_Pin|Heat1_PWM_Pin|Heat2_PWM_Pin|OTP1_RESET_Pin;
    HAL_GPIO_WritePin(GPIOA,pin.Pin,GPIO_PIN_RESET); HAL_GPIO_Init(GPIOA,&pin);
    pin.Pin=OTP2_RESET_Pin|Heat1_Fuse_Blown_Pin;
    HAL_GPIO_WritePin(GPIOB,pin.Pin,GPIO_PIN_RESET); HAL_GPIO_Init(GPIOB,&pin);
    pin.Pin=Heat2_Fuse_Blown_Pin|AirPump3_Pin|WaterValve5_Pin;
    HAL_GPIO_WritePin(GPIOD,pin.Pin,GPIO_PIN_RESET); HAL_GPIO_Init(GPIOD,&pin);
    pin.Pin=Fan1_Pin|Fan2_Pin|Fan3_Pin|WaterValve1_Pin;
    HAL_GPIO_WritePin(GPIOC,pin.Pin,GPIO_PIN_RESET); HAL_GPIO_Init(GPIOC,&pin);
}
static void uart_init(void) {
    GPIO_InitTypeDef pin={0}; __HAL_RCC_USART3_CLK_ENABLE();
    pin.Pin=GPIO_PIN_8|GPIO_PIN_9; pin.Mode=GPIO_MODE_AF_PP; pin.Pull=GPIO_PULLUP;
    pin.Speed=GPIO_SPEED_FREQ_HIGH; pin.Alternate=GPIO_AF0_USART3;
    HAL_GPIO_Init(GPIOD,&pin);
    uart.Instance=USART3; uart.Init.BaudRate=115200; uart.Init.WordLength=UART_WORDLENGTH_8B;
    uart.Init.StopBits=UART_STOPBITS_1; uart.Init.Parity=UART_PARITY_NONE;
    uart.Init.Mode=UART_MODE_TX_RX; uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    uart.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&uart)!=HAL_OK) for(;;) {}
}
__attribute__((naked,noreturn)) static void branch_app(uint32_t sp __attribute__((unused)),uint32_t pc __attribute__((unused))) {
    __asm volatile("movs r2, #0\nmsr control, r2\nisb\nmsr msp, r0\ncpsie i\nbx r1\n");
}
static void jump_app(void) {
    __disable_irq(); HAL_UART_DeInit(&uart); __HAL_RCC_USART3_FORCE_RESET(); __HAL_RCC_USART3_RELEASE_RESET();
    SysTick->CTRL=0; SysTick->LOAD=0; SysTick->VAL=0;
    NVIC->ICER[0]=0xFFFFFFFFU; NVIC->ICPR[0]=0xFFFFFFFFU;
    SCB->ICSR=SCB_ICSR_PENDSTCLR_Msk|SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR=UP_APP_BASE; __DSB(); __ISB();
    branch_app(*(const uint32_t *)UP_APP_BASE,*(const uint32_t *)(UP_APP_BASE+4));
}
void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) { for(;;) {} }
int main(void) {
    volatile uint32_t *mail=(volatile uint32_t *)UP_MAILBOX;
    uint32_t software_reset=RCC->CSR&RCC_CSR_SFTRSTF;
    int requested=software_reset && mail[0]==UP_MAILBOX_MAGIC && mail[1]==~UP_MAILBOX_MAGIC && mail[2]==1 && mail[3]==~1UL;
    mail[0]=0; __HAL_RCC_CLEAR_RESET_FLAGS();
    safe_outputs(); HAL_Init(); uart_init();
#if TPS_RTT
    SEGGER_RTT_Init();
    BOOT_LOG("[BOOT] 0.1.0 target=9C01 UART3 115200\r\n");
#endif
    BootFlash flash={flash_read,flash_erase,flash_write,flash_healthy}; Boot_Init(&engine,flash);
    uint32_t start=HAL_GetTick();
    for(;;) {
        if(!requested && !engine.session && engine.app_valid && (uint32_t)(HAL_GetTick()-start)>=500U) jump_app();
        uint8_t byte; UpFrame frame;
        if(HAL_UART_Receive(&uart,&byte,1,1)==HAL_OK && Up_Parse(&parser,byte,HAL_GetTick(),&frame)) {
            size_t n=Boot_Handle(&engine,&frame);
            if(frame.command==UP_BEGIN) BOOT_LOG("[BOOT] BEGIN processed\r\n");
            if(frame.command==UP_END) BOOT_LOG("[BOOT] END processed\r\n");
            if(HAL_UART_Transmit(&uart,engine.response,(uint16_t)n,1000)==HAL_OK &&
               frame.command==UP_RUN && engine.reset_requested && Up_Get16(engine.response+14)==UP_OK) {
                /* HAL blocking transmit waits for TC, including the final stop bit. */
                NVIC_SystemReset();
            }
        }
    }
}
