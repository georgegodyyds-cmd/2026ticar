#include "mpu6050.h"

#include "ti_msp_dl_config.h"

/*
 * MPU6050 引脚分配，两个脚都在你图里的右侧排针上：
 *   SDA -> PA28 / I2C0_SDA
 *   SCL -> PA31 / I2C0_SCL
 *   AD0 -> GND 时地址 0x68；AD0 -> VCC 时把 MPU6050_ADDR 改成 0x69。
 */
#define MPU6050_ENABLE              (1U)
#define MPU6050_ADDR                (0x68U)
#define MPU6050_DT_SEC              (0.01f)
#define MPU6050_I2C                 (I2C0)
#define MPU6050_I2C_TIMEOUT         (50000U)
#define MPU6050_CALIB_SAMPLES       (1000U)
#define MPU6050_BOOT_RETRY_COUNT    (5U)
#define MPU6050_BOOT_RETRY_DELAY    (6400000U)
#define MPU6050_GYRO_Z_DEADBAND_DPS (0.05f)
#define MPU6050_I2C_ERRATA_DELAY    (1200U)

#define MPU6050_REG_PWR_MGMT_1      (0x6BU)
#define MPU6050_REG_SMPLRT_DIV      (0x19U)
#define MPU6050_REG_CONFIG          (0x1AU)
#define MPU6050_REG_GYRO_CONFIG     (0x1BU)
#define MPU6050_REG_WHO_AM_I        (0x75U)
#define MPU6050_REG_GYRO_XOUT_H     (0x43U)

#define MPU6050_WHO_AM_I_VALUE      (0x68U)
#define MPU6050_GYRO_SCALE_250DPS   (131.0f)

volatile uint8_t g_mpuReady = 0U;
volatile float g_yawAngle = 0.0f;
volatile float g_gyroZ = 0.0f;
volatile float g_gyroZOffset = 0.0f;

volatile uint8_t g_mpuWhoAmI = 0U;
volatile uint8_t g_mpuI2cError = 0U;
volatile float g_gyroXRaw = 0.0f;
volatile float g_gyroYRaw = 0.0f;
volatile float g_gyroZRaw = 0.0f;
volatile float g_gyroXOffset = 0.0f;
volatile float g_gyroYOffset = 0.0f;
volatile uint32_t g_mpuReadCount = 0U;
volatile uint32_t g_mpuTaskCount = 0U;
volatile uint32_t g_mpuNotReadyCount = 0U;
volatile uint32_t g_mpuReadFailCount = 0U;
volatile uint8_t g_mpuLastReadOk = 0U;
volatile int16_t g_gyroXRawInt = 0;
volatile int16_t g_gyroYRawInt = 0;
volatile int16_t g_gyroZRawInt = 0;
volatile uint8_t g_gyroRawByte0 = 0U;
volatile uint8_t g_gyroRawByte1 = 0U;
volatile uint8_t g_gyroRawByte2 = 0U;
volatile uint8_t g_gyroRawByte3 = 0U;
volatile uint8_t g_gyroRawByte4 = 0U;
volatile uint8_t g_gyroRawByte5 = 0U;
volatile uint8_t g_mpuRuntimeWhoAmI = 0U;
volatile uint8_t g_mpuRuntimePwrMgmt1 = 0U;
volatile uint8_t g_mpuRuntimeGyroConfig = 0U;

static uint8_t I2C_WaitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(MPU6050_I2C) &
        DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout == 0U) {
            return 0U;
        }
        timeout--;
    }
    return 1U;
}

static uint8_t I2C_WaitNotBusy(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(MPU6050_I2C) &
        DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout == 0U) {
            return 0U;
        }
        timeout--;
    }
    return 1U;
}

static void I2C_PrepareTransfer(void)
{
    DL_I2C_resetControllerTransfer(MPU6050_I2C);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C);
}

static uint8_t I2C_CheckNoError(void)
{
    if ((DL_I2C_getControllerStatus(MPU6050_I2C) &
        DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return 0U;
    }
    return 1U;
}

static void MPU6050_I2CInit(void)
{
    static const DL_I2C_ClockConfig i2cClockConfig = {
        .clockSel = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    /* PA28/PA31 复用为 I2C0，I2C 总线需要模块或外部有上拉电阻。 */
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

    DL_I2C_reset(MPU6050_I2C);
    DL_I2C_enablePower(MPU6050_I2C);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_I2C_setClockConfig(MPU6050_I2C,
        (DL_I2C_ClockConfig *) &i2cClockConfig);
    DL_I2C_disableAnalogGlitchFilter(MPU6050_I2C);
    DL_I2C_resetControllerTransfer(MPU6050_I2C);

    /* 32MHz BUSCLK 下 timerPeriod=31 约为 100kHz，比 400kHz 更稳。 */
    DL_I2C_setTimerPeriod(MPU6050_I2C, 31U);
    DL_I2C_setControllerTXFIFOThreshold(
        MPU6050_I2C, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(
        MPU6050_I2C, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MPU6050_I2C);
    DL_I2C_enableController(MPU6050_I2C);
}

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t packet[2];

    if (I2C_WaitIdle() == 0U) {
        return 0U;
    }

    I2C_PrepareTransfer();
    packet[0] = reg;
    packet[1] = value;
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C, packet, 2U);
    DL_I2C_startControllerTransfer(MPU6050_I2C, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cycles(MPU6050_I2C_ERRATA_DELAY);

    if (I2C_WaitNotBusy() == 0U) {
        return 0U;
    }
    if (I2C_CheckNoError() == 0U) {
        return 0U;
    }
    return 1U;
}

static uint8_t MPU6050_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t i;

    if (I2C_WaitIdle() == 0U) {
        return 0U;
    }

    I2C_PrepareTransfer();
    DL_I2C_fillControllerTXFIFO(MPU6050_I2C, &reg, 1U);
    DL_I2C_startControllerTransfer(MPU6050_I2C, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(MPU6050_I2C_ERRATA_DELAY);
    if (I2C_WaitNotBusy() == 0U) {
        return 0U;
    }
    if (I2C_CheckNoError() == 0U) {
        return 0U;
    }

    if (I2C_WaitIdle() == 0U) {
        return 0U;
    }
    DL_I2C_resetControllerTransfer(MPU6050_I2C);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C);
    DL_I2C_startControllerTransfer(MPU6050_I2C, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    delay_cycles(MPU6050_I2C_ERRATA_DELAY);

    for (i = 0U; i < len; i++) {
        uint32_t timeout = MPU6050_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C)) {
            if (timeout == 0U) {
                return 0U;
            }
            timeout--;
        }
        buffer[i] = DL_I2C_receiveControllerData(MPU6050_I2C);
    }

    if (I2C_WaitNotBusy() == 0U) {
        return 0U;
    }
    if (I2C_CheckNoError() == 0U) {
        return 0U;
    }
    return 1U;
}

static uint8_t MPU6050_ReadGyroXYZ(float *gyro_x_dps, float *gyro_y_dps,
    float *gyro_z_dps)
{
    uint8_t data[6];
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    if (MPU6050_ReadRegs(MPU6050_REG_GYRO_XOUT_H, data, 6U) == 0U) {
        return 0U;
    }

    raw_x = (int16_t) (((uint16_t) data[0] << 8) | data[1]);
    raw_y = (int16_t) (((uint16_t) data[2] << 8) | data[3]);
    raw_z = (int16_t) (((uint16_t) data[4] << 8) | data[5]);
    g_gyroRawByte0 = data[0];
    g_gyroRawByte1 = data[1];
    g_gyroRawByte2 = data[2];
    g_gyroRawByte3 = data[3];
    g_gyroRawByte4 = data[4];
    g_gyroRawByte5 = data[5];
    g_gyroXRawInt = raw_x;
    g_gyroYRawInt = raw_y;
    g_gyroZRawInt = raw_z;
    *gyro_x_dps = (float) raw_x / MPU6050_GYRO_SCALE_250DPS;
    *gyro_y_dps = (float) raw_y / MPU6050_GYRO_SCALE_250DPS;
    *gyro_z_dps = (float) raw_z / MPU6050_GYRO_SCALE_250DPS;
    return 1U;
}

void MPU6050_Init(void)
{
#if MPU6050_ENABLE
    uint8_t who = 0U;
    uint8_t retry;
    uint16_t i;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_x_sum = 0.0f;
    float gyro_y_sum = 0.0f;
    float gyro_sum = 0.0f;

    MPU6050_I2CInit();
    delay_cycles(3200000U);

    g_mpuReady = 0U;
    g_mpuI2cError = 0U;

    for (retry = 0U; retry < MPU6050_BOOT_RETRY_COUNT; retry++) {
        if (MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &who, 1U) != 0U) {
            g_mpuWhoAmI = who;
            if (who == MPU6050_WHO_AM_I_VALUE) {
                g_mpuI2cError = 0U;
                break;
            }
            g_mpuI2cError = 2U;
        } else {
            g_mpuI2cError = 1U;
        }
        delay_cycles(MPU6050_BOOT_RETRY_DELAY);
    }

    if (retry >= MPU6050_BOOT_RETRY_COUNT) {
        return;
    }

    /* 唤醒 MPU6050，并把陀螺仪量程设为 +/-250dps，便于转 90 度积分。 */
    if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00U) == 0U ||
        MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07U) == 0U ||
        MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03U) == 0U ||
        MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00U) == 0U) {
        g_mpuI2cError = 3U;
        return;
    }
    delay_cycles(6400000U);

    /*
     * 上电时保持车不动，采样 Z 轴零偏。
     * 如果这段时间车在动，90 度角度会有固定误差。
    */
    for (i = 0U; i < MPU6050_CALIB_SAMPLES; i++) {
        if (MPU6050_ReadGyroXYZ(&gyro_x, &gyro_y, &gyro_z) == 0U) {
            g_mpuI2cError = 4U;
            return;
        }
        gyro_x_sum += gyro_x;
        gyro_y_sum += gyro_y;
        gyro_sum += gyro_z;
        delay_cycles(32000U);
    }

    g_gyroXOffset = gyro_x_sum / (float) MPU6050_CALIB_SAMPLES;
    g_gyroYOffset = gyro_y_sum / (float) MPU6050_CALIB_SAMPLES;
    g_gyroZOffset = gyro_sum / (float) MPU6050_CALIB_SAMPLES;
    g_gyroZ = 0.0f;
    g_yawAngle = 0.0f;
    g_mpuReady = 1U;
#else
    g_mpuReady = 0U;
    g_yawAngle = 0.0f;
    g_gyroZ = 0.0f;
    g_gyroZOffset = 0.0f;
#endif
}

void MPU6050_Task10ms(void)
{
#if MPU6050_ENABLE
    float gyro_x;
    float gyro_y;
    float gyro_z;

    g_mpuTaskCount++;
    if (g_mpuReady == 0U) {
        g_mpuNotReadyCount++;
        return;
    }
    if (MPU6050_ReadGyroXYZ(&gyro_x, &gyro_y, &gyro_z) == 0U) {
        g_mpuReadFailCount++;
        g_mpuLastReadOk = 0U;
        g_mpuI2cError = 5U;
        return;
    }
    g_mpuLastReadOk = 1U;
    (void) MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I,
        (uint8_t *) &g_mpuRuntimeWhoAmI, 1U);
    (void) MPU6050_ReadRegs(MPU6050_REG_PWR_MGMT_1,
        (uint8_t *) &g_mpuRuntimePwrMgmt1, 1U);
    (void) MPU6050_ReadRegs(MPU6050_REG_GYRO_CONFIG,
        (uint8_t *) &g_mpuRuntimeGyroConfig, 1U);
    g_mpuReadCount++;
    g_gyroXRaw = gyro_x;
    g_gyroYRaw = gyro_y;
    g_gyroZRaw = gyro_z;
    g_gyroZ = gyro_z - g_gyroZOffset;
    if (g_gyroZ < MPU6050_GYRO_Z_DEADBAND_DPS &&
        g_gyroZ > -MPU6050_GYRO_Z_DEADBAND_DPS) {
        g_gyroZ = 0.0f;
    }
    g_yawAngle += g_gyroZ * MPU6050_DT_SEC;
#endif
}

void MPU6050_ResetYaw(float yaw_deg)
{
    g_yawAngle = yaw_deg;
}

MPU6050_State_t MPU6050_GetState(void)
{
    MPU6050_State_t state;

    state.ready = g_mpuReady;
    state.gyro_z_dps = g_gyroZ;
    state.yaw_deg = g_yawAngle;
    state.gyro_z_offset = g_gyroZOffset;

    return state;
}
