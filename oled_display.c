#include "oled_display.h"

#include "car_chassis.h"
#include "line_sensor.h"
#include "track_control.h"
#include "ti_msp_dl_config.h"

#define OLED_I2C                  (I2C0)
#define OLED_TIMEOUT              (100000U)
#define OLED_I2C_ERRATA_DELAY     (1200U)
#define OLED_LINE_PIXELS          (64U)
#define OLED_REFRESH_TICKS        (10U)

volatile uint8_t g_oledReady = 0U;
volatile uint8_t g_oledAddress = 0x3CU;
volatile uint8_t g_oledErrorStage = 0U;

typedef struct {
    char character;
    uint8_t column[5];
} OLED_Glyph_t;

static const OLED_Glyph_t g_glyphs[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'N', {0x7F, 0x02, 0x0C, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
};

static void OLED_I2CInit(void)
{
    static const DL_I2C_ClockConfig clock_config = {
        .clockSel = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    /*
     * OLED与MPU6050共用PA28/PA31上的I2C0总线。
     * 两个器件地址不同，OLED为0x3C，MPU6050为0x68。
     */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM3, IOMUX_PINCM3_PF_I2C0_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM6, IOMUX_PINCM6_PF_I2C0_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IOMUX_PINCM3);
    DL_GPIO_enableHiZ(IOMUX_PINCM6);

    DL_I2C_reset(OLED_I2C);
    DL_I2C_enablePower(OLED_I2C);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_I2C_setClockConfig(OLED_I2C,
        (DL_I2C_ClockConfig *) &clock_config);
    DL_I2C_disableAnalogGlitchFilter(OLED_I2C);
    DL_I2C_resetControllerTransfer(OLED_I2C);
    DL_I2C_setTimerPeriod(OLED_I2C, 31U);
    DL_I2C_setControllerTXFIFOThreshold(
        OLED_I2C, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(
        OLED_I2C, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(OLED_I2C);
    DL_I2C_enableController(OLED_I2C);
}

static uint8_t OLED_WaitIdle(void)
{
    uint32_t timeout = OLED_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_I2C) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t OLED_Write(const uint8_t *data, uint16_t length)
{
    uint16_t sent;
    uint32_t timeout = OLED_TIMEOUT;

    if (OLED_WaitIdle() == 0U || length == 0U) {
        return 0U;
    }
    DL_I2C_resetControllerTransfer(OLED_I2C);
    DL_I2C_flushControllerTXFIFO(OLED_I2C);
    DL_I2C_flushControllerRXFIFO(OLED_I2C);
    sent = DL_I2C_fillControllerTXFIFO(OLED_I2C, data, length);
    DL_I2C_startControllerTransfer(OLED_I2C, g_oledAddress,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(OLED_I2C_ERRATA_DELAY);

    while (sent < length) {
        if ((DL_I2C_getControllerStatus(OLED_I2C) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return 0U;
        }
        if (!DL_I2C_isControllerTXFIFOFull(OLED_I2C)) {
            DL_I2C_transmitControllerData(OLED_I2C, data[sent]);
            sent++;
            timeout = OLED_TIMEOUT;
        } else if (timeout-- == 0U) {
            return 0U;
        }
    }

    if (OLED_WaitIdle() == 0U) {
        return 0U;
    }
    return ((DL_I2C_getControllerStatus(OLED_I2C) &
        DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) ? 1U : 0U;
}

static uint8_t OLED_Command(const uint8_t *commands, uint8_t count)
{
    uint8_t packet[32];
    uint8_t i;

    if (count > 31U) {
        return 0U;
    }
    packet[0] = 0x00U;
    for (i = 0U; i < count; i++) {
        packet[i + 1U] = commands[i];
    }
    return OLED_Write(packet, (uint16_t) count + 1U);
}

static const uint8_t *OLED_FindGlyph(char character)
{
    uint8_t i;
    for (i = 0U; i < (uint8_t) (sizeof(g_glyphs) / sizeof(g_glyphs[0])); i++) {
        if (g_glyphs[i].character == character) {
            return g_glyphs[i].column;
        }
    }
    return g_glyphs[0].column;
}

static void OLED_PutText(uint8_t *pixels, const char *text)
{
    uint8_t x = 0U;
    uint8_t column;
    const uint8_t *glyph;

    while (*text != '\0' && (uint8_t) (x + 6U) <= OLED_LINE_PIXELS) {
        glyph = OLED_FindGlyph(*text);
        for (column = 0U; column < 5U; column++) {
            pixels[x++] = glyph[column];
        }
        pixels[x++] = 0U;
        text++;
    }
    while (x < OLED_LINE_PIXELS) {
        pixels[x++] = 0U;
    }
}

static void OLED_FormatValue(char label, int32_t value, char *text)
{
    uint32_t magnitude;
    int8_t position = 7;

    text[0] = label;
    text[1] = ':';
    text[2] = ' ';
    text[3] = ' ';
    text[4] = ' ';
    text[5] = ' ';
    text[6] = ' ';
    text[7] = '0';
    text[8] = '\0';

    if (value < 0) {
        magnitude = (uint32_t) (-value);
    } else {
        magnitude = (uint32_t) value;
    }
    do {
        text[position--] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U && position >= 2);
    if (value < 0 && position >= 2) {
        text[position] = '-';
    }
}

static void OLED_FormatHex(uint8_t value, char *text)
{
    uint8_t high = (uint8_t) ((value >> 4) & 0x0FU);
    uint8_t low = (uint8_t) (value & 0x0FU);
    text[0] = 'I';
    text[1] = ':';
    text[2] = (char) ((high < 10U) ? ('0' + high) : ('A' + high - 10U));
    text[3] = (char) ((low < 10U) ? ('0' + low) : ('A' + low - 10U));
    text[4] = '\0';
}

static uint8_t OLED_DrawPage(uint8_t page, const char *text)
{
    uint8_t command[3];
    uint8_t packet[OLED_LINE_PIXELS + 1U];

    command[0] = (uint8_t) (0xB0U | (page & 0x07U));
    command[1] = 0x00U;
    command[2] = 0x10U;
    if (OLED_Command(command, 3U) == 0U) {
        return 0U;
    }
    packet[0] = 0x40U;
    OLED_PutText(&packet[1], text);
    return OLED_Write(packet, OLED_LINE_PIXELS + 1U);
}

void OLED_Display_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };
    uint8_t page;

    OLED_I2CInit();
    g_oledReady = 0U;
    g_oledErrorStage = 1U;
    g_oledAddress = 0x3CU;
    if (OLED_Command(init_commands, (uint8_t) sizeof(init_commands)) != 0U) {
        g_oledReady = 1U;
    } else {
        /* 常见模块也可能把地址电阻配置成0x3D。 */
        DL_I2C_resetControllerTransfer(OLED_I2C);
        DL_I2C_flushControllerTXFIFO(OLED_I2C);
        delay_cycles(32000U);
        g_oledErrorStage = 2U;
        g_oledAddress = 0x3DU;
        if (OLED_Command(init_commands,
            (uint8_t) sizeof(init_commands)) != 0U) {
            g_oledReady = 1U;
        }
    }
    if (g_oledReady == 0U) {
        return;
    }
    g_oledErrorStage = 3U;
    for (page = 0U; page < 8U; page++) {
        if (OLED_DrawPage(page, "") == 0U) {
            g_oledReady = 0U;
            return;
        }
    }
    g_oledErrorStage = 0U;
}

void OLED_Display_Task10ms(void)
{
    static uint8_t tick = 0U;
    static uint8_t slot = 0U;
    char text[9];
    uint8_t page;

    if (g_oledReady == 0U || ++tick < OLED_REFRESH_TICKS) {
        return;
    }
    tick = 0U;

    switch (slot) {
        case 0U:
        case 2U:
        case 5U:
            page = 0U;
            OLED_FormatValue('L', motor_left_pwm, text);
            break;
        case 1U:
        case 3U:
        case 6U:
            page = 1U;
            OLED_FormatValue('R', motor_right_pwm, text);
            break;
        case 4U:
            page = 2U;
            OLED_FormatValue('B', (int32_t) g_openLoopBasePwm, text);
            break;
        case 7U:
            page = 3U;
            OLED_FormatValue('C', (int32_t) g_turnCorrection, text);
            break;
        case 8U:
            page = 4U;
            if (g_squareControlState == 1U) {
                text[0] = 'S'; text[1] = ':'; text[2] = 'T'; text[3] = 'U';
                text[4] = 'R'; text[5] = 'N'; text[6] = '\0';
            } else if (g_squareControlState == 2U) {
                text[0] = 'S'; text[1] = ':'; text[2] = 'D'; text[3] = 'O';
                text[4] = 'N'; text[5] = 'E'; text[6] = '\0';
            } else {
                text[0] = 'S'; text[1] = ':'; text[2] = 'E'; text[3] = 'D';
                text[4] = 'G'; text[5] = 'E'; text[6] = '\0';
            }
            break;
        default:
            page = 5U;
            OLED_FormatHex(g_lineMask, text);
            break;
    }

    if (OLED_DrawPage(page, text) == 0U) {
        g_oledReady = 0U;
        g_oledErrorStage = 4U;
    }
    slot++;
    if (slot >= 10U) {
        slot = 0U;
    }
}
