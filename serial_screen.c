#include "serial_screen.h"

#include "ti_msp_dl_config.h"

#define SERIAL_UART_INST           (UART1)
#define SERIAL_UART_INT_IRQn       (UART1_INT_IRQn)

volatile uint8_t g_serialLastByte = 0U;
volatile uint8_t g_serialLapCommand = 0U;
volatile uint32_t g_serialRxCount = 0U;

static volatile uint8_t g_commandReady = 0U;

void SerialScreen_Init(void)
{
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel = DL_UART_MAIN_CLOCK_LFCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    static const DL_UART_Main_Config uart_config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE
    };

    g_serialLastByte = 0U;
    g_serialLapCommand = 0U;
    g_serialRxCount = 0U;
    g_commandReady = 0U;

    /* 串口屏只发送：PA9作为UART1_RX，PA10保持空闲。 */
    DL_GPIO_initPeripheralInputFunction(
        IOMUX_PINCM20, IOMUX_PINCM20_PF_UART1_RX);
    DL_UART_Main_reset(SERIAL_UART_INST);
    DL_UART_Main_enablePower(SERIAL_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_Main_setClockConfig(SERIAL_UART_INST,
        (DL_UART_Main_ClockConfig *) &clock_config);
    DL_UART_Main_init(SERIAL_UART_INST,
        (DL_UART_Main_Config *) &uart_config);
    DL_UART_Main_setOversampling(
        SERIAL_UART_INST, DL_UART_OVERSAMPLING_RATE_3X);
    DL_UART_Main_setBaudRateDivisor(SERIAL_UART_INST, 1U, 9U);
    DL_UART_Main_enableInterrupt(
        SERIAL_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(SERIAL_UART_INST);

    NVIC_ClearPendingIRQ(SERIAL_UART_INT_IRQn);
    NVIC_EnableIRQ(SERIAL_UART_INT_IRQn);
}

uint8_t SerialScreen_ConsumeLapCommand(uint8_t *laps)
{
    uint32_t primask;

    if (laps == 0) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_commandReady == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    *laps = g_serialLapCommand;
    g_commandReady = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void UART1_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_Main_getPendingInterrupt(SERIAL_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            data = DL_UART_Main_receiveData(SERIAL_UART_INST);
            g_serialLastByte = data;
            g_serialRxCount++;

            /* 同时兼容单字节0x01~0x05和ASCII字符'1'~'5'。 */
            if (data >= 0x01U && data <= 0x05U) {
                g_serialLapCommand = data;
                g_commandReady = 1U;
            } else if (data >= (uint8_t) '1' && data <= (uint8_t) '5') {
                g_serialLapCommand = (uint8_t) (data - (uint8_t) '0');
                g_commandReady = 1U;
            }
            break;

        default:
            break;
    }
}
