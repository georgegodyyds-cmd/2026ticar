#include "car_chassis.h"
#include "lap_control.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "serial_screen.h"
#include "track_control.h"
#include "ti_msp_dl_config.h"

volatile uint8_t g_bootStage = 0U;
volatile uint32_t g_mainLoopTicks = 0U;

static void Boot_Blink(uint8_t times)
{
    uint8_t i;

    for (i = 0U; i < times; i++) {
        DL_GPIO_setPins(TEST_PORT, TEST_PIN_0_PIN);
        delay_cycles(3200000);
        DL_GPIO_clearPins(TEST_PORT, TEST_PIN_0_PIN);
        delay_cycles(3200000);
    }
}

int main(void)
{
    uint8_t lap_command;

    /* 外部降压电源冷启动时先等待电压和NRST稳定。 */
    delay_cycles(6400000U);

    g_bootStage = 1U;
    SYSCFG_DL_init();

    g_bootStage = 2U;
    SerialScreen_Init();
    Boot_Blink(3U);

    /*
     * TB6612 STBY 先保持低电平，等 MPU 和传感器初始化完成后再由底盘模块使能。
     * 这样外部电源冷启动时，不会在初始化阶段误输出 PWM。
     */
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
    delay_cycles(48000000);

    g_bootStage = 3U;
    MPU6050_Init();

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

    g_bootStage = 7U;
    while (1) {
        if (SerialScreen_ConsumeLapCommand(&lap_command) != 0U) {
            CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
            MPU6050_ResetYaw(0.0f);
            TrackControl_SetMode(TRACK_MODE_AUTO);
            LapControl_Start(lap_command);
        }

        if (CarChassis_Consume10msFlag() != 0U) {
            g_mainLoopTicks++;
            if ((g_mainLoopTicks % 50U) == 0U) {
                DL_GPIO_togglePins(TEST_PORT, TEST_PIN_0_PIN);
            }

            if (LapControl_IsRunning() != 0U) {
                TrackControl_Task10ms();
                if (LapControl_Update(g_squareControlState) != 0U) {
                    CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
                }
            } else {
                CarChassis_SetOpenLoopPWM(0.0f, 0.0f);
            }
        }
    }
}
