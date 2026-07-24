#include "bluetooth_command.h"
#include "car_role.h"
#include "car_chassis.h"
#include "lap_control.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "oled_display.h"
#include "serial_screen.h"
#include "track_control.h"
#include "ti_msp_dl_config.h"

volatile uint8_t g_bootStage = 0U;
volatile uint32_t g_mainLoopTicks = 0U;

int main(void)
{
    uint8_t lap_command;

    /* 外部电源冷启动时，先等待电压和NRST稳定。 */
    delay_cycles(6400000U);

    g_bootStage = 1U;
    SYSCFG_DL_init();

    g_bootStage = 2U;
    BluetoothCommand_Init();
#if CAR_ROLE == CAR_ROLE_MASTER
    /* 串口屏只安装在主车：屏幕TX -> PA9(UART1_RX)。 */
    SerialScreen_Init();
#endif

    /* 初始化期间保持TB6612待机，避免错误PWM驱动电机。 */
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
    delay_cycles(48000000U);

    g_bootStage = 3U;
    /*
     * MPU6050上电后会进行陀螺仪零偏校准。
     * 校准期间必须让小车保持静止，否则后续积分角度会持续漂移。
     */
    MPU6050_Init();
    OLED_Display_Init();

    g_bootStage = 4U;
    LineSensor_Init();

    g_bootStage = 5U;
    CarChassis_Init();

    g_bootStage = 6U;
    TrackControl_Init();
    LapControl_Init();
    MPU6050_ResetYaw(0.0f);
    TrackControl_SetMode(TRACK_MODE_AUTO);
    CarChassis_SetOpenLoopPWM(0.0f, 0.0f);

    /* 主车等待串口屏命令，从车等待主车的蓝牙命令。 */
    g_bootStage = 7U;
    while (1) {
#if CAR_ROLE == CAR_ROLE_MASTER
        /* 主车收到屏幕圈数后，先通知从车，再启动自身计圈。 */
        if (SerialScreen_ConsumeLapCommand(&lap_command) != 0U &&
            LapControl_IsRunning() == 0U) {
            CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
            MPU6050_ResetYaw(0.0f);
            TrackControl_SetMode(TRACK_MODE_AUTO);
            BluetoothCommand_SendLapCommand(lap_command);
            LapControl_Start(lap_command);
        }

        /* 主车不执行蓝牙收到的圈数命令，但清除可能残留的数据。 */
        (void) BluetoothCommand_ConsumeLapCommand(&lap_command);
#else
        /* 从车只在停车状态接受主车发来的圈数。 */
        if (BluetoothCommand_ConsumeLapCommand(&lap_command) != 0U &&
            LapControl_IsRunning() == 0U) {
            CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
            MPU6050_ResetYaw(0.0f);
            TrackControl_SetMode(TRACK_MODE_AUTO);
            LapControl_Start(lap_command);
            BluetoothCommand_SendAccepted(lap_command);
        }
#endif

        if (CarChassis_Consume10msFlag() != 0U) {
            g_mainLoopTicks++;
            if (LapControl_IsRunning() != 0U) {
                TrackControl_Task10ms();
                if (LapControl_Update(g_squareControlState) != 0U) {
                    CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
#if CAR_ROLE == CAR_ROLE_SLAVE
                    BluetoothCommand_SendFinished(g_completedLaps);
#endif
                }
            } else {
                CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
            }
            OLED_Display_Task10ms();
        }
    }
}
