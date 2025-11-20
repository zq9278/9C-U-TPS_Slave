#include "config.h"
#include "24cxx.h"
#include "uart_driver.h"
#include "Uart_Communicate.h"
#include <string.h>
#include "config.h"
#include "24cxx.h"
#include "AppMain/system_app.h"
#include "Uart_Communicate.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>

extern UartPort_t rk3576_uart_port;
SystemSettings_t g_settings;

static uint16_t crc16_modbus_local(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else
                crc >>= 1;
        }
    }
    return crc;
}

void Settings_Defaults(SystemSettings_t *s)
{
    memset(s, 0, sizeof(*s));
    s->magic = 0x9C71;
    s->version = 1;
    s->mode_select = 1;

    s->left_temp_c  = 38.0f;
    s->right_temp_c = 38.0f;
    s->use_common_temp = 1;

    // Mode 1 default profile
    s->mode[0].target_kpa       = 25.0f;
    s->mode[0].t_rise_ms        = 1500;
    s->mode[0].t_hold_ms        = 2000;
    s->mode[0].t_pulse_total_ms = 4000;
    s->mode[0].t_pulse_on_ms    = 300;
    s->mode[0].t_pulse_off_ms   = 300;
    s->mode[0].squeeze_mode     = 0; // normal

    // Mode 2 default profile
    s->mode[1].target_kpa       = 30.0f;
    s->mode[1].t_rise_ms        = 2000;
    s->mode[1].t_hold_ms        = 3000;
    s->mode[1].t_pulse_total_ms = 5000;
    s->mode[1].t_pulse_on_ms    = 250;
    s->mode[1].t_pulse_off_ms   = 250;
    s->mode[1].squeeze_mode     = 2; // sync
}

bool Settings_Save(const SystemSettings_t *in)
{
    // Compute CRC and write to EEPROM starting at 0x20
    SystemSettings_t tmp;
    memcpy(&tmp, in, sizeof(tmp));
    tmp.crc16 = crc16_modbus_local((const uint8_t*)&tmp, (uint16_t)(sizeof(tmp) - sizeof(tmp.crc16)));

    AT24CXX_Write(0x20, (uint8_t*)&tmp, sizeof(tmp));
    return true;
}

bool Settings_Load(SystemSettings_t *out)
{
    SystemSettings_t tmp;
    AT24CXX_Read(0x20, (uint8_t*)&tmp, sizeof(tmp));

    if (tmp.magic != 0x9C71 || tmp.version != 1) {
        return false;
    }

    uint16_t crc = crc16_modbus_local((const uint8_t*)&tmp, (uint16_t)(sizeof(tmp) - sizeof(tmp.crc16)));
    if (crc != tmp.crc16) {
        return false;
    }

    memcpy(out, &tmp, sizeof(tmp));
    return true;
}

void Config_Init(void)
{
    AT24CXX_Init();
    if (!Settings_Load(&g_settings)) {
        Settings_Defaults(&g_settings);
        (void)Settings_Save(&g_settings);
    }
}

void Settings_Broadcast(void)
{
    uint8_t mode_sel = g_settings.mode_select;
    rk3576_uart_port.sender(DATA_UINT8_T, U8_MODE_SELECT, &mode_sel);

    float temp = g_settings.left_temp_c;
    rk3576_uart_port.sender(DATA_FLOAT, F32_LEFT_TEMP_SET_C, &temp);

    float press = g_settings.mode[(mode_sel <= 1) ? 0 : 1].target_kpa;
    rk3576_uart_port.sender(DATA_FLOAT, F32_PRESSURE_SET_KPA, &press);
}

// Global settings instance
SystemSettings_t g_settings;

// -----------------------------------------------------------------------------
// Local helpers
// -----------------------------------------------------------------------------


static void eeprom_read(uint16_t addr, void *dst, uint16_t len)
{
    AT24CXX_Read(addr, (uint8_t*)dst, len);
}

static void eeprom_write(uint16_t addr, const void *src, uint16_t len)
{
    AT24CXX_Write(addr, (uint8_t*)src, len);
}




// -----------------------------------------------------------------------------
// Broadcast helpers
// -----------------------------------------------------------------------------
static inline void tx_u8(uint16_t id, uint8_t v)
{
    tx_frame_t tx = {0};
    tx.type = TX_DATA_UINT8; tx.frame_id = id; tx.v.u8 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static inline void tx_u16(uint16_t id, uint16_t v)
{
    tx_frame_t tx = {0};
    tx.type = TX_DATA_U16; tx.frame_id = id; tx.v.u16 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static inline void tx_u32(uint16_t id, uint32_t v)
{
    tx_frame_t tx = {0};
    tx.type = TX_DATA_U32; tx.frame_id = id; tx.v.u32 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static inline void tx_f32(uint16_t id, float v)
{
    tx_frame_t tx = {0};
    tx.type = TX_DATA_FLOAT; tx.frame_id = id; tx.v.f32 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}
