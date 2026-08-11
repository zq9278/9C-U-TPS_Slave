#include "Modules/Display/treatment_oled_display.h"

#include <stddef.h>

#include "UserDrivers/Oled/oled_driver.h"

#define OLED_PLOT_X_MIN              1U
#define OLED_PLOT_X_MAX              126U
#define OLED_PLOT_Y_MIN              9U
#define OLED_PLOT_Y_MAX              42U
#define OLED_PLOT_HISTORY_COUNT      126U
#define OLED_PLOT_TEMP_MIN_TENTHS    200
#define OLED_PLOT_TEMP_MAX_TENTHS    450
#define OLED_PLOT_INVALID_TEMPERATURE INT16_MIN

static int16_t s_left_history[OLED_PLOT_HISTORY_COUNT];
static int16_t s_right_history[OLED_PLOT_HISTORY_COUNT];
static uint8_t s_history_next;
static uint8_t s_history_count;

static const char *TreatmentOledDisplay_StateText(TreatmentOledState state)
{
    switch (state)
    {
    case TREATMENT_OLED_STATUS_HEATING:
        return "HEATING";
    case TREATMENT_OLED_STATUS_PAUSED:
        return "PAUSED";
    case TREATMENT_OLED_STATUS_FAULT:
        return "FAULT";
    case TREATMENT_OLED_STATUS_STOPPED:
    default:
        return "STOPPED";
    }
}

static void TreatmentOledDisplay_AppendText(char **cursor, const char *text)
{
    while (*text != '\0')
    {
        **cursor = *text;
        ++(*cursor);
        ++text;
    }
}

static void TreatmentOledDisplay_AppendUnsigned(char **cursor, uint16_t value, uint8_t width)
{
    uint16_t divisor = 1U;
    uint8_t digit;

    for (digit = 1U; digit < width; ++digit)
    {
        divisor = (uint16_t)(divisor * 10U);
    }
    for (digit = 0U; digit < width; ++digit)
    {
        **cursor = (char)('0' + ((value / divisor) % 10U));
        ++(*cursor);
        divisor = (uint16_t)(divisor / 10U);
    }
}

static void TreatmentOledDisplay_AppendTemperature(char **cursor,
                                                   float temperature_c,
                                                   uint8_t valid)
{
    uint16_t tenths;

    if ((valid == 0U) || (temperature_c <= 0.0f) || (temperature_c > 99.9f))
    {
        TreatmentOledDisplay_AppendText(cursor, "--.-");
    }
    else
    {
        tenths = (uint16_t)((temperature_c * 10.0f) + 0.5f);
        TreatmentOledDisplay_AppendUnsigned(cursor, (uint16_t)(tenths / 10U), 2U);
        **cursor = '.';
        ++(*cursor);
        TreatmentOledDisplay_AppendUnsigned(cursor, (uint16_t)(tenths % 10U), 1U);
    }
}

static void TreatmentOledDisplay_FormatHeader(char *line,
                                              TreatmentOledState state,
                                              float target_temp_c)
{
    char *cursor = line;

    TreatmentOledDisplay_AppendText(&cursor, TreatmentOledDisplay_StateText(state));
    TreatmentOledDisplay_AppendText(&cursor, " T:");
    TreatmentOledDisplay_AppendTemperature(&cursor, target_temp_c, 1U);
    TreatmentOledDisplay_AppendText(&cursor, " 20-45C");
    *cursor = '\0';
}

static void TreatmentOledDisplay_FormatTemperatures(char *line,
                                                    const TreatmentOledSnapshot *snapshot)
{
    char *cursor = line;

    TreatmentOledDisplay_AppendText(&cursor, "L-");
    TreatmentOledDisplay_AppendTemperature(&cursor, snapshot->temp_left_c,
                                           snapshot->sensor_valid);
    TreatmentOledDisplay_AppendText(&cursor, "C R.");
    TreatmentOledDisplay_AppendTemperature(&cursor, snapshot->temp_right_c,
                                           snapshot->sensor_valid);
    *cursor++ = 'C';
    *cursor = '\0';
}

static void TreatmentOledDisplay_FormatPwm(char *line, uint16_t left, uint16_t right)
{
    char *cursor = line;

    TreatmentOledDisplay_AppendText(&cursor, "PWM ");
    TreatmentOledDisplay_AppendUnsigned(&cursor, left, 4U);
    *cursor++ = '/';
    TreatmentOledDisplay_AppendUnsigned(&cursor, right, 4U);
    TreatmentOledDisplay_AppendText(&cursor, " 63S");
    *cursor = '\0';
}

static int16_t TreatmentOledDisplay_TemperatureToTenths(float temperature_c, uint8_t valid)
{
    if ((valid == 0U) || (temperature_c <= 0.0f) || (temperature_c > 99.9f))
    {
        return OLED_PLOT_INVALID_TEMPERATURE;
    }
    return (int16_t)((temperature_c * 10.0f) + 0.5f);
}

static void TreatmentOledDisplay_PushHistory(const TreatmentOledSnapshot *snapshot)
{
    s_left_history[s_history_next] =
        TreatmentOledDisplay_TemperatureToTenths(snapshot->temp_left_c,
                                                 snapshot->sensor_valid);
    s_right_history[s_history_next] =
        TreatmentOledDisplay_TemperatureToTenths(snapshot->temp_right_c,
                                                 snapshot->sensor_valid);
    s_history_next = (uint8_t)((s_history_next + 1U) % OLED_PLOT_HISTORY_COUNT);
    if (s_history_count < OLED_PLOT_HISTORY_COUNT)
    {
        ++s_history_count;
    }
}

static uint8_t TreatmentOledDisplay_MapTemperature(int16_t temperature_tenths)
{
    int32_t clamped = temperature_tenths;
    int32_t plot_height = OLED_PLOT_Y_MAX - OLED_PLOT_Y_MIN;

    if (clamped < OLED_PLOT_TEMP_MIN_TENTHS)
    {
        clamped = OLED_PLOT_TEMP_MIN_TENTHS;
    }
    else if (clamped > OLED_PLOT_TEMP_MAX_TENTHS)
    {
        clamped = OLED_PLOT_TEMP_MAX_TENTHS;
    }

    return (uint8_t)(OLED_PLOT_Y_MAX -
                     (((clamped - OLED_PLOT_TEMP_MIN_TENTHS) * plot_height) /
                      (OLED_PLOT_TEMP_MAX_TENTHS - OLED_PLOT_TEMP_MIN_TENTHS)));
}

static void TreatmentOledDisplay_DrawPlotFrame(float target_temp_c)
{
    int16_t target_tenths = TreatmentOledDisplay_TemperatureToTenths(target_temp_c, 1U);

    OledDriver_DrawLine(0U, 8U, 127U, 8U);
    OledDriver_DrawLine(0U, 43U, 127U, 43U);
    OledDriver_DrawLine(0U, 8U, 0U, 43U);
    OledDriver_DrawLine(127U, 8U, 127U, 43U);

    if (target_tenths != OLED_PLOT_INVALID_TEMPERATURE)
    {
        uint8_t target_y = TreatmentOledDisplay_MapTemperature(target_tenths);
        OledDriver_DrawLine(OLED_PLOT_X_MIN, target_y, OLED_PLOT_X_MAX, target_y);
    }
}

static void TreatmentOledDisplay_DrawHistory(void)
{
    uint8_t sample;
    uint8_t oldest;
    uint8_t previous_x = 0U;
    uint8_t previous_y = 0U;
    uint8_t previous_valid = 0U;

    oldest = (uint8_t)((s_history_next + OLED_PLOT_HISTORY_COUNT - s_history_count) %
                       OLED_PLOT_HISTORY_COUNT);
    for (sample = 0U; sample < s_history_count; ++sample)
    {
        uint8_t history_index = (uint8_t)((oldest + sample) % OLED_PLOT_HISTORY_COUNT);
        uint8_t x = (uint8_t)(OLED_PLOT_X_MAX - (s_history_count - 1U - sample));
        int16_t temperature = s_left_history[history_index];

        if (temperature == OLED_PLOT_INVALID_TEMPERATURE)
        {
            previous_valid = 0U;
        }
        else
        {
            uint8_t y = TreatmentOledDisplay_MapTemperature(temperature);
            if (previous_valid != 0U)
            {
                OledDriver_DrawLine(previous_x, previous_y, x, y);
            }
            else
            {
                OledDriver_DrawPixel(x, y, 1U);
            }
            previous_x = x;
            previous_y = y;
            previous_valid = 1U;
        }

        temperature = s_right_history[history_index];
        if (temperature != OLED_PLOT_INVALID_TEMPERATURE)
        {
            OledDriver_DrawPixel(x, TreatmentOledDisplay_MapTemperature(temperature), 1U);
        }
    }
}

void TreatmentOledDisplay_Init(void)
{
    s_history_next = 0U;
    s_history_count = 0U;
    OledDriver_Init();
}

void TreatmentOledDisplay_Render(const TreatmentOledSnapshot *snapshot)
{
    char line[22];

    if (snapshot == NULL)
    {
        return;
    }

    TreatmentOledDisplay_PushHistory(snapshot);

    OledDriver_Clear();
    TreatmentOledDisplay_FormatHeader(line, snapshot->state, snapshot->target_temp_c);
    OledDriver_DrawText(0U, 0U, line, 1U);
    TreatmentOledDisplay_DrawPlotFrame(snapshot->target_temp_c);
    TreatmentOledDisplay_DrawHistory();
    TreatmentOledDisplay_FormatTemperatures(line, snapshot);
    OledDriver_DrawText(0U, 46U, line, 1U);
    TreatmentOledDisplay_FormatPwm(line, snapshot->pwm_left, snapshot->pwm_right);
    OledDriver_DrawText(0U, 56U, line, 1U);
    OledDriver_Refresh();
}
