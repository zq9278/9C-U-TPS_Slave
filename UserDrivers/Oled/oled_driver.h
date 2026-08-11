#ifndef USER_DRIVERS_OLED_OLED_DRIVER_H
#define USER_DRIVERS_OLED_OLED_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_DRIVER_WIDTH  128U
#define OLED_DRIVER_HEIGHT 64U

/*
 * 0.96-inch 128x64 OLED driver (SSD1306-compatible, 4-pin software I2C).
 * Board mapping: PC8=SCL, PA15=SDA, 7-bit address 0x3C.
 */
void OledDriver_Init(void);
void OledDriver_Clear(void);
void OledDriver_Refresh(void);
void OledDriver_DrawPixel(uint8_t x, uint8_t y, uint8_t enabled);
void OledDriver_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void OledDriver_DrawChar(uint8_t x, uint8_t y, char character, uint8_t scale);
void OledDriver_DrawText(uint8_t x, uint8_t y, const char *text, uint8_t scale);
uint32_t OledDriver_GetAckCount(void);

#ifdef __cplusplus
}
#endif

#endif
