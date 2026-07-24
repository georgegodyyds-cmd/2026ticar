#include "jy61p.h"

#include "ti_msp_dl_config.h"

/*
 * JY61P 使用 UART0：
 *   JY61P TX -> PA1 (UART0_RX)
 *   JY61P RX -> PA0 (UART0_TX，可用于以后配置模块)
 *   JY61P GND 与单片机 GND 共地
 * 模块串口保持默认 9600、8N1，并开启角度和角速度数据输出。
 */
#define JY61P_UART                 (UART0)
#define JY61P_UART_INT_IRQn        (UART0_INT_IRQn)
#define JY61P_FRAME_LENGTH         (11U)
#define JY61P_FRAME_HEADER         (0x55U)
#define JY61P_FRAME_GYRO           (0x52U)
#define JY61P_FRAME_ANGLE          (0x53U)
#define JY61P_DATA_TIMEOUT_TICKS   (50U)

volatile uint8_t g_jy61pReady = 0U;
volatile float g_jy61pYawAngle = 0.0f;
volatile float g_jy61pGyroZ = 0.0f;
volatile uint32_t g_jy61pAngleFrameCount = 0U;

static volatile uint8_t g_frame[JY61P_FRAME_LENGTH];
static volatile uint8_t g_frameIndex = 0U;
static volatile uint8_t g_noAngleTicks = JY61P_DATA_TIMEOUT_TICKS;
static float g_previousRawYaw = 0.0f;
static float g_unwrappedYaw = 0.0f;
static float g_yawZero = 0.0f;
static float g_resetYawRequest = 0.0f;
static uint8_t g_haveAngle = 0U;

static float normalize_delta(float delta)
{
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

static int16_t frame_int16(uint8_t low_index)
{
    return (int16_t) ((uint16_t) g_frame[low_index] |
        ((uint16_t) g_frame[low_index + 1U] << 8));
}

static void process_frame(void)
{
    uint8_t checksum = 0U;
    uint8_t i;
    float raw_yaw;

    for (i = 0U; i < 10U; i++) {
        checksum = (uint8_t) (checksum + g_frame[i]);
    }
    if (checksum != g_frame[10]) {
        return;
    }

    if (g_frame[1] == JY61P_FRAME_GYRO) {
        g_jy61pGyroZ = (float) frame_int16(6U) * 2000.0f / 32768.0f;
        return;
    }
    if (g_frame[1] != JY61P_FRAME_ANGLE) {
        return;
    }

    raw_yaw = (float) frame_int16(6U) * 180.0f / 32768.0f;
    if (g_haveAngle == 0U) {
        g_previousRawYaw = raw_yaw;
        g_unwrappedYaw = raw_yaw;
        g_yawZero = g_unwrappedYaw - g_resetYawRequest;
        g_haveAngle = 1U;
    } else {
        g_unwrappedYaw += normalize_delta(raw_yaw - g_previousRawYaw);
        g_previousRawYaw = raw_yaw;
    }

    g_jy61pYawAngle = g_unwrappedYaw - g_yawZero;
    g_noAngleTicks = 0U;
    g_jy61pReady = 1U;
    g_jy61pAngleFrameCount++;
}

void JY61P_Init(void)
{
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel = DL_UART_MAIN_CLOCK_LFCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    static const DL_UART_Main_Config uart_config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE
    };

    g_jy61pReady = 0U;
    g_jy61pYawAngle = 0.0f;
    g_jy61pGyroZ = 0.0f;
    g_jy61pAngleFrameCount = 0U;
    g_frameIndex = 0U;
    g_noAngleTicks = JY61P_DATA_TIMEOUT_TICKS;
    g_haveAngle = 0U;
    g_resetYawRequest = 0.0f;

    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM1, IOMUX_PINCM1_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(
        IOMUX_PINCM2, IOMUX_PINCM2_PF_UART0_RX);

    DL_UART_Main_reset(JY61P_UART);
    DL_UART_Main_enablePower(JY61P_UART);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_Main_setClockConfig(JY61P_UART,
        (DL_UART_Main_ClockConfig *) &clock_config);
    DL_UART_Main_init(JY61P_UART,
        (DL_UART_Main_Config *) &uart_config);
    DL_UART_Main_setOversampling(
        JY61P_UART, DL_UART_OVERSAMPLING_RATE_3X);
    /* 32768Hz LFCLK，3倍过采样，分频值 1 + 9/64，对应约 9600bps。 */
    DL_UART_Main_setBaudRateDivisor(JY61P_UART, 1U, 9U);
    DL_UART_Main_enableInterrupt(JY61P_UART, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(JY61P_UART);

    NVIC_ClearPendingIRQ(JY61P_UART_INT_IRQn);
    NVIC_EnableIRQ(JY61P_UART_INT_IRQn);
}

void JY61P_Task10ms(void)
{
    if (g_noAngleTicks < 255U) {
        g_noAngleTicks++;
    }
    if (g_noAngleTicks >= JY61P_DATA_TIMEOUT_TICKS) {
        g_jy61pReady = 0U;
        g_jy61pGyroZ = 0.0f;
    }
}

void JY61P_ResetYaw(float yaw_deg)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_resetYawRequest = yaw_deg;
    if (g_haveAngle != 0U) {
        g_yawZero = g_unwrappedYaw - yaw_deg;
    }
    g_jy61pYawAngle = yaw_deg;
    if (primask == 0U) {
        __enable_irq();
    }
}

void UART0_IRQHandler(void)
{
    uint8_t data;

    if (DL_UART_Main_getPendingInterrupt(JY61P_UART) !=
        DL_UART_MAIN_IIDX_RX) {
        return;
    }

    data = DL_UART_Main_receiveData(JY61P_UART);
    if (g_frameIndex == 0U && data != JY61P_FRAME_HEADER) {
        return;
    }

    g_frame[g_frameIndex++] = data;
    if (g_frameIndex >= JY61P_FRAME_LENGTH) {
        process_frame();
        g_frameIndex = 0U;
    }
}
