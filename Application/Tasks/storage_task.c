#include "storage_task.h"
#include "config.h"
#include "LOG.h"
#include "AppMain/mode_curves.h"

extern SystemSettings_t g_settings;

void StorageTask(void *argument)
{
    (void)argument;
    storage_cmd_t cmd;
    for(;;)
    {
        if (xQueueReceive(gStorageQueue, &cmd, portMAX_DELAY) == pdPASS)
        {
            switch (cmd)
            {
                case STORAGE_CMD_LOAD_ALL:
                    Config_Init();
                    ModeCurves_InitFromSettings(&g_settings);
                    Settings_Broadcast();
                    break;
                case STORAGE_CMD_SAVE_PARAM:
                    Settings_Save(&g_settings);
                    break;
                default:
                    break;
            }
        }
    }
}
