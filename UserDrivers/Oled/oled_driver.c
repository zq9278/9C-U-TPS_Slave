#include "UserDrivers/Oled/oled_driver.h"

#include <stddef.h>
#include <string.h>

#include "main.h"

#define OLED_I2C_WRITE_ADDRESS 0x78U
#define OLED_CONTROL_COMMAND   0x00U
#define OLED_CONTROL_DATA      0x40U
#define OLED_PAGE_COUNT        (OLED_DRIVER_HEIGHT / 8U)

typedef struct
{
    char character;
    uint8_t columns[5];
} OledGlyph;

/* Compact 5x7 font for the status UI. Unsupported characters render as '?'. */
static const OledGlyph s_font[] = {
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {'.', {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}},
    {'/', {0x20U, 0x10U, 0x08U, 0x04U, 0x02U}},
    {':', {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}},
    {'?', {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}},
    {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
    {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
    {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
    {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
    {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
    {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
    {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
    {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
    {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
    {'A', {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}},
    {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'C', {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}},
    {'D', {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}},
    {'E', {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}},
    {'F', {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}},
    {'G', {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}},
    {'H', {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}},
    {'I', {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}},
    {'J', {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}},
    {'K', {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}},
    {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
    {'M', {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}},
    {'N', {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}},
    {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
    {'P', {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}},
    {'Q', {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}},
    {'R', {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}},
    {'S', {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}},
    {'T', {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}},
    {'U', {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}},
    {'V', {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}},
    {'W', {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}},
    {'X', {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}},
    {'Y', {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}},
    {'Z', {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}},
};

static uint8_t s_framebuffer[OLED_DRIVER_WIDTH * OLED_PAGE_COUNT];
static volatile uint32_t s_ack_count;

static void OledDriver_Delay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < 8U; ++count)
    {
        __NOP();
    }
}

static void OledDriver_SetScl(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, state);
}

static void OledDriver_SetSda(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, state);
}

static void OledDriver_Start(void)
{
    OledDriver_SetSda(GPIO_PIN_SET);
    OledDriver_SetScl(GPIO_PIN_SET);
    OledDriver_Delay();
    OledDriver_SetSda(GPIO_PIN_RESET);
    OledDriver_Delay();
    OledDriver_SetScl(GPIO_PIN_RESET);
}

static void OledDriver_Stop(void)
{
    OledDriver_SetSda(GPIO_PIN_RESET);
    OledDriver_Delay();
    OledDriver_SetScl(GPIO_PIN_SET);
    OledDriver_Delay();
    OledDriver_SetSda(GPIO_PIN_SET);
    OledDriver_Delay();
}

static uint8_t OledDriver_WriteByte(uint8_t value)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0U; bit < 8U; ++bit)
    {
        OledDriver_SetSda(((value & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        OledDriver_Delay();
        OledDriver_SetScl(GPIO_PIN_SET);
        OledDriver_Delay();
        OledDriver_SetScl(GPIO_PIN_RESET);
        value <<= 1U;
    }

    /* Release SDA for the display ACK clock. The 4-pin module supplies pull-ups. */
    OledDriver_SetSda(GPIO_PIN_SET);
    OledDriver_Delay();
    OledDriver_SetScl(GPIO_PIN_SET);
    OledDriver_Delay();
    acknowledged = (HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    OledDriver_SetScl(GPIO_PIN_RESET);
    if (acknowledged != 0U)
    {
        ++s_ack_count;
    }
    return acknowledged;
}

static void OledDriver_BeginWrite(uint8_t control)
{
    OledDriver_Start();
    OledDriver_WriteByte(OLED_I2C_WRITE_ADDRESS);
    OledDriver_WriteByte(control);
}

static void OledDriver_WriteCommand(uint8_t command)
{
    OledDriver_BeginWrite(OLED_CONTROL_COMMAND);
    OledDriver_WriteByte(command);
    OledDriver_Stop();
}

static const uint8_t *OledDriver_FindGlyph(char character)
{
    size_t index;

    if ((character >= 'a') && (character <= 'z'))
    {
        character = (char)(character - ('a' - 'A'));
    }

    for (index = 0U; index < (sizeof(s_font) / sizeof(s_font[0])); ++index)
    {
        if (s_font[index].character == character)
        {
            return s_font[index].columns;
        }
    }

    return s_font[5].columns;
}

void OledDriver_Clear(void)
{
    (void)memset(s_framebuffer, 0, sizeof(s_framebuffer));
}

void OledDriver_DrawPixel(uint8_t x, uint8_t y, uint8_t enabled)
{
    uint16_t index;
    uint8_t mask;

    if ((x >= OLED_DRIVER_WIDTH) || (y >= OLED_DRIVER_HEIGHT))
    {
        return;
    }

    index = (uint16_t)(((uint16_t)(y / 8U) * OLED_DRIVER_WIDTH) + x);
    mask = (uint8_t)(1U << (y % 8U));
    if (enabled != 0U)
    {
        s_framebuffer[index] |= mask;
    }
    else
    {
        s_framebuffer[index] &= (uint8_t)(~mask);
    }
}

void OledDriver_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    int16_t x = x0;
    int16_t y = y0;
    int16_t end_x = x1;
    int16_t end_y = y1;
    int16_t delta_x = (end_x >= x) ? (end_x - x) : (x - end_x);
    int16_t delta_y = (end_y >= y) ? (end_y - y) : (y - end_y);
    int16_t step_x = (x < end_x) ? 1 : -1;
    int16_t step_y = (y < end_y) ? 1 : -1;
    int16_t error = delta_x - delta_y;

    for (;;)
    {
        int16_t error_twice;

        OledDriver_DrawPixel((uint8_t)x, (uint8_t)y, 1U);
        if ((x == end_x) && (y == end_y))
        {
            break;
        }
        error_twice = (int16_t)(2 * error);
        if (error_twice > -delta_y)
        {
            error -= delta_y;
            x += step_x;
        }
        if (error_twice < delta_x)
        {
            error += delta_x;
            y += step_y;
        }
    }
}

void OledDriver_DrawChar(uint8_t x, uint8_t y, char character, uint8_t scale)
{
    const uint8_t *glyph;
    uint8_t column;
    uint8_t row;
    uint8_t scale_x;
    uint8_t scale_y;

    if ((scale == 0U) || (scale > 2U))
    {
        return;
    }

    glyph = OledDriver_FindGlyph(character);
    for (column = 0U; column < 5U; ++column)
    {
        for (row = 0U; row < 7U; ++row)
        {
            if ((glyph[column] & (uint8_t)(1U << row)) == 0U)
            {
                continue;
            }
            for (scale_x = 0U; scale_x < scale; ++scale_x)
            {
                for (scale_y = 0U; scale_y < scale; ++scale_y)
                {
                    OledDriver_DrawPixel((uint8_t)(x + (column * scale) + scale_x),
                                         (uint8_t)(y + (row * scale) + scale_y),
                                         1U);
                }
            }
        }
    }
}

void OledDriver_DrawText(uint8_t x, uint8_t y, const char *text, uint8_t scale)
{
    uint8_t cursor_x = x;
    uint8_t advance;

    if ((text == NULL) || (scale == 0U) || (scale > 2U))
    {
        return;
    }

    advance = (uint8_t)(6U * scale);
    while ((*text != '\0') && ((uint16_t)cursor_x + (5U * scale) <= OLED_DRIVER_WIDTH))
    {
        OledDriver_DrawChar(cursor_x, y, *text, scale);
        cursor_x = (uint8_t)(cursor_x + advance);
        ++text;
    }
}

void OledDriver_Refresh(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < OLED_PAGE_COUNT; ++page)
    {
        OledDriver_WriteCommand((uint8_t)(0xB0U + page));
        OledDriver_WriteCommand(0x00U);
        OledDriver_WriteCommand(0x10U);
        OledDriver_BeginWrite(OLED_CONTROL_DATA);
        for (column = 0U; column < OLED_DRIVER_WIDTH; ++column)
        {
            OledDriver_WriteByte(s_framebuffer[((uint16_t)page * OLED_DRIVER_WIDTH) + column]);
        }
        OledDriver_Stop();
    }
}

void OledDriver_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Re-assert open-drain bus configuration even if CubeMX code is regenerated. */
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);
    gpio.Pin = OLED_SCL_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_SCL_GPIO_Port, &gpio);
    gpio.Pin = OLED_SDA_Pin;
    HAL_GPIO_Init(OLED_SDA_GPIO_Port, &gpio);
    s_ack_count = 0U;

    OledDriver_WriteCommand(0xAEU);
    OledDriver_WriteCommand(0x00U);
    OledDriver_WriteCommand(0x10U);
    OledDriver_WriteCommand(0x40U);
    OledDriver_WriteCommand(0x81U);
    OledDriver_WriteCommand(0xCFU);
    OledDriver_WriteCommand(0xA1U);
    OledDriver_WriteCommand(0xC8U);
    OledDriver_WriteCommand(0xA6U);
    OledDriver_WriteCommand(0xA8U);
    OledDriver_WriteCommand(0x3FU);
    OledDriver_WriteCommand(0xD3U);
    OledDriver_WriteCommand(0x00U);
    OledDriver_WriteCommand(0xD5U);
    OledDriver_WriteCommand(0x80U);
    OledDriver_WriteCommand(0xD9U);
    OledDriver_WriteCommand(0xF1U);
    OledDriver_WriteCommand(0xDAU);
    OledDriver_WriteCommand(0x12U);
    OledDriver_WriteCommand(0xDBU);
    OledDriver_WriteCommand(0x30U);
    OledDriver_WriteCommand(0x20U);
    OledDriver_WriteCommand(0x02U);
    OledDriver_WriteCommand(0x8DU);
    OledDriver_WriteCommand(0x14U);
    OledDriver_Clear();
    OledDriver_Refresh();
    OledDriver_WriteCommand(0xAFU);
}

uint32_t OledDriver_GetAckCount(void)
{
    return s_ack_count;
}
