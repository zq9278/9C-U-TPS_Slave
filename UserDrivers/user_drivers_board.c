#include "user_drivers_board.h"

#include "BSP/I2c/bsp_i2c_hal_adapter.h"
#include "BSP/I2c/bsp_i2c_soft_adapter.h"
#include "BSP/Spi/bsp_spi_hal_adapter.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "task.h"
#include "tim.h"

typedef struct
{
    BspGpioPin *scl;
    BspGpioPin *sda;
} BoardSoftI2cLines;

BspI2c gUserEeSoftI2c;
BspI2c gUserPressureI2c1;
BspI2c gUserPressureI2c2;
BspSpi gUserRtdSpi2;

UserEeprom24cxx gUserEeprom24c02;
UserAds1248 gUserAds1248;
UserDs18b20 gUserDs18b20;
UserPressureSensor gUserPressureSensor;

static BspGpioPin s_ee_scl;
static BspGpioPin s_ee_sda;
static BspGpioPin s_rtd_cs;
static BspGpioPin s_rtd_start;
static BspGpioPin s_rtd_drdy;
static BspGpioPin s_ds18b20_dq;

static BspI2cSoftAdapter s_ee_soft_adapter;
static BoardSoftI2cLines s_ee_lines;
static uint8_t s_board_initialized = 0U;

static void BoardDelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

static void BoardDelayUs(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    (void)HAL_TIM_Base_Start(&htim7);
    while (__HAL_TIM_GET_COUNTER(&htim7) < us)
    {
    }
}

static void BoardEnterCritical(void)
{
    taskENTER_CRITICAL();
}

static void BoardExitCritical(void)
{
    taskEXIT_CRITICAL();
}

static void BoardSoftI2cDelay(void *context)
{
    (void)context;
    BoardDelayUs(4U);
}

static void BoardSoftI2cWriteScl(void *context, uint8_t level)
{
    BoardSoftI2cLines *lines = (BoardSoftI2cLines *)context;

    if ((lines == NULL) || (lines->scl == NULL))
    {
        return;
    }

    BspGpio_ConfigOutput(lines->scl, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
    BspGpio_Write(lines->scl, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void BoardSoftI2cWriteSda(void *context, uint8_t level)
{
    BoardSoftI2cLines *lines = (BoardSoftI2cLines *)context;

    if ((lines == NULL) || (lines->sda == NULL))
    {
        return;
    }

    if (level)
    {
        BspGpio_ConfigInput(lines->sda, GPIO_PULLUP);
    }
    else
    {
        BspGpio_ConfigOutput(lines->sda, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
        BspGpio_Reset(lines->sda);
    }
}

static uint8_t BoardSoftI2cReadSda(void *context)
{
    BoardSoftI2cLines *lines = (BoardSoftI2cLines *)context;

    if ((lines == NULL) || (lines->sda == NULL))
    {
        return 1U;
    }

    BspGpio_ConfigInput(lines->sda, GPIO_PULLUP);
    return (uint8_t)(BspGpio_Read(lines->sda) == GPIO_PIN_SET);
}

static void BoardRtdChipSelect(void *context, uint8_t selected)
{
    BspGpioPin *pin = (BspGpioPin *)context;

    if (pin != NULL)
    {
        BspGpio_WriteActive(pin, selected);
    }
}

void UserDrivers_BoardInit(void)
{
    BspGpioPinConfig gpio_cfg;
    BspI2cConfig i2c_cfg;
    BspSpiConfig spi_cfg;

    if (s_board_initialized != 0U)
    {
        return;
    }

    gpio_cfg.port = EE_SCL_GPIO_Port;
    gpio_cfg.pin = EE_SCL_Pin;
    gpio_cfg.active_state = GPIO_PIN_SET;
    BspGpio_Init(&s_ee_scl, &gpio_cfg);

    gpio_cfg.port = EE_SDA_GPIO_Port;
    gpio_cfg.pin = EE_SDA_Pin;
    gpio_cfg.active_state = GPIO_PIN_SET;
    BspGpio_Init(&s_ee_sda, &gpio_cfg);

    gpio_cfg.port = RTD_CS_GPIO_Port;
    gpio_cfg.pin = RTD_CS_Pin;
    gpio_cfg.active_state = GPIO_PIN_RESET;
    BspGpio_Init(&s_rtd_cs, &gpio_cfg);

    gpio_cfg.port = RTD_START_GPIO_Port;
    gpio_cfg.pin = RTD_START_Pin;
    gpio_cfg.active_state = GPIO_PIN_SET;
    BspGpio_Init(&s_rtd_start, &gpio_cfg);

    gpio_cfg.port = RTD_RDY_GPIO_Port;
    gpio_cfg.pin = RTD_RDY_Pin;
    gpio_cfg.active_state = GPIO_PIN_RESET;
    BspGpio_Init(&s_rtd_drdy, &gpio_cfg);

    gpio_cfg.port = TS1_GPIO_Port;
    gpio_cfg.pin = TS1_Pin;
    gpio_cfg.active_state = GPIO_PIN_SET;
    BspGpio_Init(&s_ds18b20_dq, &gpio_cfg);

    s_ee_lines.scl = &s_ee_scl;
    s_ee_lines.sda = &s_ee_sda;
    s_ee_soft_adapter.write_scl = BoardSoftI2cWriteScl;
    s_ee_soft_adapter.write_sda = BoardSoftI2cWriteSda;
    s_ee_soft_adapter.read_sda = BoardSoftI2cReadSda;
    s_ee_soft_adapter.delay = BoardSoftI2cDelay;
    s_ee_soft_adapter.line_context = &s_ee_lines;
    BspI2cSoftAdapter_MakeConfig(&i2c_cfg, &s_ee_soft_adapter, 100U);
    BspI2c_Init(&gUserEeSoftI2c, &i2c_cfg);

    BspI2cHalAdapter_MakeConfig(&i2c_cfg, &hi2c1, BSP_I2C_HAL_MODE_BLOCKING, 20U);
    BspI2c_Init(&gUserPressureI2c1, &i2c_cfg);
    BspI2cHalAdapter_MakeConfig(&i2c_cfg, &hi2c2, BSP_I2C_HAL_MODE_BLOCKING, 20U);
    BspI2c_Init(&gUserPressureI2c2, &i2c_cfg);

    BspSpiHalAdapter_MakeConfig(&spi_cfg,
                                &hspi2,
                                BSP_SPI_HAL_MODE_BLOCKING,
                                BoardRtdChipSelect,
                                &s_rtd_cs,
                                1U,
                                100U);
    BspSpi_Init(&gUserRtdSpi2, &spi_cfg);
    BspSpi_Deselect(&gUserRtdSpi2);

    UserEeprom24cxx_Init(&gUserEeprom24c02,
                         &gUserEeSoftI2c,
                         NULL,
                         USER_EEPROM24C02,
                         8U,
                         BoardDelayMs);
    UserAds1248_InitDevice(&gUserAds1248,
                           &gUserRtdSpi2,
                           &s_rtd_start,
                           &s_rtd_drdy,
                           NULL,
                           BoardDelayMs);
    UserDs18b20_InitDevice(&gUserDs18b20,
                           &s_ds18b20_dq,
                           BoardDelayUs,
                           BoardEnterCritical,
                           BoardExitCritical);
    UserPressureSensor_Init(&gUserPressureSensor,
                            &gUserPressureI2c1,
                            NULL,
                            BoardDelayMs);
    s_board_initialized = 1U;
}

void UserDrivers_BoardSetAds1248DrdySemaphore(SemaphoreHandle_t sem)
{
    UserAds1248_SetDrdySemaphore(&gUserAds1248, sem);
}
