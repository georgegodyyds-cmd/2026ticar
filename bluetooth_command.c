#include "bluetooth_command.h"

#include "ti_msp_dl_config.h"

/*
 * HC-05/HC-06透明串口接线：
 *   蓝牙 TXD -> PB16 (UART2_RX)
 *   蓝牙 RXD -> PB15 (UART2_TX)
 * 串口参数固定为9600、8N1。
 */
#define BLUETOOTH_UART          (UART2)
#define BLUETOOTH_UART_INT_IRQn (UART2_INT_IRQn)

volatile uint8_t g_bluetoothLastByte = 0U;
volatile uint8_t g_bluetoothLapCommand = 0U;
volatile uint32_t g_bluetoothRxCount = 0U;

static volatile uint8_t g_commandReady = 0U;

void BluetoothCommand_Init(void)
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

    g_bluetoothLastByte = 0U;
    g_bluetoothLapCommand = 0U;
    g_bluetoothRxCount = 0U;
    g_commandReady = 0U;

    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM32, IOMUX_PINCM32_PF_UART2_TX);
    DL_GPIO_initPeripheralInputFunction(
        IOMUX_PINCM33, IOMUX_PINCM33_PF_UART2_RX);

    DL_UART_Main_reset(BLUETOOTH_UART);
    DL_UART_Main_enablePower(BLUETOOTH_UART);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_Main_setClockConfig(BLUETOOTH_UART,
        (DL_UART_Main_ClockConfig *) &clock_config);
    DL_UART_Main_init(BLUETOOTH_UART,
        (DL_UART_Main_Config *) &uart_config);
    DL_UART_Main_setOversampling(
        BLUETOOTH_UART, DL_UART_OVERSAMPLING_RATE_3X);
    /* 32768Hz LFCLK下，分频1+9/64约等于9600bps。 */
    DL_UART_Main_setBaudRateDivisor(BLUETOOTH_UART, 1U, 9U);
    DL_UART_Main_enableInterrupt(
        BLUETOOTH_UART, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(BLUETOOTH_UART);

    NVIC_ClearPendingIRQ(BLUETOOTH_UART_INT_IRQn);
    NVIC_EnableIRQ(BLUETOOTH_UART_INT_IRQn);
}

uint8_t BluetoothCommand_ConsumeLapCommand(uint8_t *laps)
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

    *laps = g_bluetoothLapCommand;
    g_commandReady = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void BluetoothCommand_SendLapCommand(uint8_t laps)
{
    if (laps < 1U || laps > 5U) {
        return;
    }

    /* 主车把单字节圈数命令转发给从车。 */
    DL_UART_Main_transmitDataBlocking(BLUETOOTH_UART, laps);
}

void BluetoothCommand_SendAccepted(uint8_t laps)
{
    /* 0x81~0x85表示对应圈数命令已经被本车接受。 */
    DL_UART_Main_transmitDataBlocking(
        BLUETOOTH_UART, (uint8_t) (0x80U | laps));
}

void BluetoothCommand_SendFinished(uint8_t laps)
{
    /* 0x41~0x45表示本车已经跑完对应圈数。 */
    DL_UART_Main_transmitDataBlocking(
        BLUETOOTH_UART, (uint8_t) (0x40U | laps));
}

void UART2_IRQHandler(void)
{
    uint8_t data;
    uint8_t laps = 0U;

    if (DL_UART_Main_getPendingInterrupt(BLUETOOTH_UART) !=
        DL_UART_MAIN_IIDX_RX) {
        return;
    }

    data = DL_UART_Main_receiveData(BLUETOOTH_UART);
    g_bluetoothLastByte = data;
    g_bluetoothRxCount++;

    /* 同时兼容十六进制01~05和ASCII字符'1'~'5'。 */
    if (data >= 0x01U && data <= 0x05U) {
        laps = data;
    } else if (data >= (uint8_t) '1' && data <= (uint8_t) '5') {
        laps = (uint8_t) (data - (uint8_t) '0');
    }

    if (laps != 0U) {
        g_bluetoothLapCommand = laps;
        g_commandReady = 1U;
    }
}
