#include "line_sensor.h"

#include "ti_msp_dl_config.h"

/*
 * 7 路灰度红外传感器引脚分配，按车头从左到右。
 * 这些脚都在你图里右侧两排排针上，并避开已有电机/PWM/编码器接线：
 *   S1 -> PB12  S2 -> PB17  S3 -> PB8   S4 -> PB7
 *   S5 -> PB13  S6 -> PB13  S7 -> PB20
 *
 * 这里按“数字量输出”模块处理，bit0=S1最左，bit6=S7最右。
 * 如果你的模块黑线输出低电平，把 LINE_SENSOR_BLACK_IS_HIGH 改成 0。
 */
#define LINE_SENSOR_ENABLE          (1U)
#define LINE_SENSOR_BLACK_IS_HIGH   (1U)
#define LINE_SENSOR_RIGHT12_TIED    (1U)

#define LINE_S1_IOMUX               (IOMUX_PINCM29)
#define LINE_S2_IOMUX               (IOMUX_PINCM43)
#define LINE_S3_IOMUX               (IOMUX_PINCM25)
#define LINE_S4_IOMUX               (IOMUX_PINCM24)
#define LINE_S5_IOMUX               (IOMUX_PINCM30)
#define LINE_S6_IOMUX               (IOMUX_PINCM30)
#define LINE_S7_IOMUX               (IOMUX_PINCM48)
#define LINE_SENSOR_GPIO_PORT       (GPIOB)
#define LINE_SENSOR_GPIO_MASK       (DL_GPIO_PIN_12 | DL_GPIO_PIN_17 | \
                                     DL_GPIO_PIN_8 | DL_GPIO_PIN_7 | \
                                     DL_GPIO_PIN_13 | \
                                     DL_GPIO_PIN_20)

volatile uint8_t g_lineMask = 0;
volatile int16_t g_lineError = 0;
volatile uint8_t g_lineSeen = 0;

static uint8_t g_rawHistory[3] = {0U, 0U, 0U};
static uint8_t g_rawHistoryIndex = 0U;
static uint8_t g_rawHistoryReady = 0U;

static uint8_t LineSensor_ReadDigitalRaw(void)
{
#if LINE_SENSOR_ENABLE
    uint8_t mask = 0U;
    uint32_t pins;

    pins = DL_GPIO_readPins(LINE_SENSOR_GPIO_PORT, LINE_SENSOR_GPIO_MASK);
    if ((pins & DL_GPIO_PIN_12) != 0U) {
        mask |= (1U << 0);
    }
    if ((pins & DL_GPIO_PIN_17) != 0U) {
        mask |= (1U << 1);
    }
    if ((pins & DL_GPIO_PIN_8) != 0U) {
        mask |= (1U << 2);
    }
    if ((pins & DL_GPIO_PIN_7) != 0U) {
        mask |= (1U << 3);
    }
    if ((pins & DL_GPIO_PIN_13) != 0U) {
        /* PCB上第5、第6路共用PB13，因此两个逻辑位同步。 */
        mask |= (1U << 4) | (1U << 5);
    }
    if ((pins & DL_GPIO_PIN_20) != 0U) {
        mask |= (1U << 6);
    }
    return mask;
#else
    return 0U;
#endif
}

static uint8_t LineSensor_FilterRaw(uint8_t raw)
{
    uint8_t filtered = 0U;
    uint8_t bit;

    /* 每一路用最近3次采样多数表决，滤掉比较器临界点的单拍跳变。 */
    if (g_rawHistoryReady == 0U) {
        g_rawHistory[0] = raw;
        g_rawHistory[1] = raw;
        g_rawHistory[2] = raw;
        g_rawHistoryReady = 1U;
    } else {
        g_rawHistory[g_rawHistoryIndex] = raw;
        g_rawHistoryIndex++;
        if (g_rawHistoryIndex >= 3U) {
            g_rawHistoryIndex = 0U;
        }
    }

    for (bit = 0U; bit < LINE_SENSOR_COUNT; bit++) {
        uint8_t bit_mask = (uint8_t) (1U << bit);
        uint8_t count = 0U;
        if ((g_rawHistory[0] & bit_mask) != 0U) {
            count++;
        }
        if ((g_rawHistory[1] & bit_mask) != 0U) {
            count++;
        }
        if ((g_rawHistory[2] & bit_mask) != 0U) {
            count++;
        }
        if (count >= 2U) {
            filtered |= bit_mask;
        }
    }
    return filtered;
}

void LineSensor_Init(void)
{
#if LINE_SENSOR_ENABLE
    /*
     * 灰度模块通常自己有比较器输出，MCU 端使用普通数字输入。
     * 这里开内部上拉，避免悬空时乱跳；若你的模块推挽输出，上拉也不影响。
     */
    DL_GPIO_initDigitalInputFeatures(
        LINE_S1_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LINE_S2_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LINE_S3_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LINE_S4_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LINE_S5_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LINE_S6_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LINE_S7_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
#endif
    g_lineMask = 0U;
    g_lineError = 0;
    g_lineSeen = 0U;
    g_rawHistory[0] = 0U;
    g_rawHistory[1] = 0U;
    g_rawHistory[2] = 0U;
    g_rawHistoryIndex = 0U;
    g_rawHistoryReady = 0U;
}

LineSensor_State_t LineSensor_Update(void)
{
    static const int16_t weight[LINE_SENSOR_COUNT] = {
        -3000, -2000, -1000, 0, 1000, 2000, 3000
    };
    LineSensor_State_t state;
    int32_t weighted_sum = 0;
    int32_t active_count = 0;
    uint8_t raw;
    uint8_t mask;
    uint8_t i;

    raw = LineSensor_FilterRaw(LineSensor_ReadDigitalRaw());
#if LINE_SENSOR_BLACK_IS_HIGH
    mask = raw & 0x7FU;
#else
    mask = ((uint8_t) ~raw) & 0x7FU;
#endif

#if LINE_SENSOR_RIGHT12_TIED
    /*
     * PCB 上 S5/S6 两路共用PB13，实际只能作为一个右侧虚拟探头使用。
     * g_lineMask 仍保留原始状态，便于观察和直角判断；误差计算时只算一次，
     * 避免 S6/S7 同时为 1 时右侧权重被重复放大。
     */
    for (i = 0U; i < 4U; i++) {
        if ((mask & (uint8_t) (1U << i)) != 0U) {
            weighted_sum += weight[i];
            active_count++;
        }
    }
    if ((mask & 0x30U) != 0U) {
        /* S5/S6共用PB13，使用二者中间的位置权重。 */
        weighted_sum += 1500;
        active_count++;
    }
    if ((mask & 0x40U) != 0U) {
        weighted_sum += 3000;
        active_count++;
    }
#else
    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((mask & (uint8_t) (1U << i)) != 0U) {
            weighted_sum += weight[i];
            active_count++;
        }
    }
#endif

    state.mask = mask;
    state.active_num = (uint8_t) active_count;
    if (active_count > 0) {
        state.seen = 1U;
        state.error = (int16_t) (weighted_sum / active_count);
    } else {
        state.seen = 0U;
        state.error = g_lineError;
    }

    g_lineMask = state.mask;
    g_lineError = state.error;
    g_lineSeen = state.seen;

    return state;
}
