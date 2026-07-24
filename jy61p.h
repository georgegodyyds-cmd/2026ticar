#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>

void JY61P_Init(void);
void JY61P_Task10ms(void);
void JY61P_ResetYaw(float yaw_deg);

/* 调试时重点观察这四个变量。 */
extern volatile uint8_t g_jy61pReady;
extern volatile float g_jy61pYawAngle;
extern volatile float g_jy61pGyroZ;
extern volatile uint32_t g_jy61pAngleFrameCount;

#endif
