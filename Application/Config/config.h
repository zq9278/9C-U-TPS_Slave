#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Pressure profile per mode
typedef struct __attribute__((packed)) {
    float    target_kpa;          // Target pressure (kPa)
    uint32_t t_rise_ms;           // Slow ramp duration (ms)
    uint32_t t_hold_ms;           // Constant hold duration (ms)
    uint32_t t_pulse_total_ms;    // Total pulse duration window (ms)
    uint16_t t_pulse_on_ms;       // Pulse on time (ms)
    uint16_t t_pulse_off_ms;      // Pulse off time (ms)
    uint8_t  squeeze_mode;        // 0=normal, 1=alternate, 2=sync
    uint8_t  reserved;
} PressureProfile_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;               // 0x9C71
    uint8_t  version;             // settings structure version
    uint8_t  mode_select;         // 1 or 2

    // Temperature settings (C)
    float    left_temp_c;         // left eye target
    float    right_temp_c;        // right eye target
    uint8_t  use_common_temp;     // 1=keep left/right equal
    uint8_t  pad0[3];

    // Two modes' pressure profiles
    PressureProfile_t mode[2];

    uint16_t crc16;               // CRC16(Modbus) over bytes [0..(crc16-1)]
} SystemSettings_t;

extern SystemSettings_t g_settings;

void Config_Init(void);
bool Settings_Load(SystemSettings_t *out);
bool Settings_Save(const SystemSettings_t *in);
void Settings_Defaults(SystemSettings_t *s);
// Broadcast current settings to host on boot
void Settings_Broadcast(void);


#endif // APP_CONFIG_H

