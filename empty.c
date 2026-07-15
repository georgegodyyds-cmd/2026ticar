#include "car_chassis.h"
#include "line_sensor.h"
#include "mpu6050.h"
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
    g_bootStage = 1U;
    SYSCFG_DL_init();

    g_bootStage = 2U;
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
    MPU6050_ResetYaw(0.0f);
    TrackControl_SetMode(TRACK_MODE_AUTO);

    g_bootStage = 7U;
    while (1) {
        if (CarChassis_Consume10msFlag() != 0U) {
            g_mainLoopTicks++;
            if ((g_mainLoopTicks % 50U) == 0U) {
                DL_GPIO_togglePins(TEST_PORT, TEST_PIN_0_PIN);
            }

            TrackControl_Task10ms();
            CarChassis_Task10ms();
        }
    }
}
