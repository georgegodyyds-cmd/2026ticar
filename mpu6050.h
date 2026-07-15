#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct {
    uint8_t ready;
    float gyro_z_dps;
    float yaw_deg;
    float gyro_z_offset;
} MPU6050_State_t;

void MPU6050_Init(void);
void MPU6050_Task10ms(void);
void MPU6050_ResetYaw(float yaw_deg);
MPU6050_State_t MPU6050_GetState(void);

extern volatile uint8_t g_mpuReady;
extern volatile float g_yawAngle;
extern volatile float g_gyroZ;
extern volatile float g_gyroZOffset;
extern volatile uint8_t g_mpuWhoAmI;
extern volatile uint8_t g_mpuI2cError;
extern volatile float g_gyroXRaw;
extern volatile float g_gyroYRaw;
extern volatile float g_gyroZRaw;
extern volatile float g_gyroXOffset;
extern volatile float g_gyroYOffset;
extern volatile uint32_t g_mpuReadCount;
extern volatile uint32_t g_mpuTaskCount;
extern volatile uint32_t g_mpuNotReadyCount;
extern volatile uint32_t g_mpuReadFailCount;
extern volatile uint8_t g_mpuLastReadOk;
extern volatile int16_t g_gyroXRawInt;
extern volatile int16_t g_gyroYRawInt;
extern volatile int16_t g_gyroZRawInt;
extern volatile uint8_t g_gyroRawByte0;
extern volatile uint8_t g_gyroRawByte1;
extern volatile uint8_t g_gyroRawByte2;
extern volatile uint8_t g_gyroRawByte3;
extern volatile uint8_t g_gyroRawByte4;
extern volatile uint8_t g_gyroRawByte5;
extern volatile uint8_t g_mpuRuntimeWhoAmI;
extern volatile uint8_t g_mpuRuntimePwrMgmt1;
extern volatile uint8_t g_mpuRuntimeGyroConfig;

#endif
