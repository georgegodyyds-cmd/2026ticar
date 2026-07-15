#ifndef TRACK_CONTROL_H
#define TRACK_CONTROL_H

#include <stdint.h>

typedef enum {
    TRACK_MODE_STRAIGHT = 0,
    TRACK_MODE_LINE_CURVE,
    TRACK_MODE_SQUARE,
    TRACK_MODE_AUTO
} TrackMode_t;

void TrackControl_Init(void);
void TrackControl_Task10ms(void);
void TrackControl_SetMode(TrackMode_t mode);
TrackMode_t TrackControl_GetMode(void);

extern volatile int16_t g_trackMode;
extern volatile int16_t g_activeTrackMode;
extern volatile float g_trackTurnPwm;
extern volatile float g_yawTarget;
extern volatile float g_yawError;
extern volatile int8_t g_squareTurnDir;
extern volatile uint8_t g_squareCornerMask;
extern volatile uint8_t g_squareControlState;
extern volatile float g_lineCorrectionPwm;
extern volatile float g_yawCorrectionPwm;
extern volatile uint8_t g_cornerStableCount;
extern volatile uint8_t g_turnDoneStableCount;
extern volatile uint8_t g_lineRecoveryCount;
extern volatile uint8_t g_cornerLockoutCount;
extern volatile uint8_t g_allWhiteHoldActive;
extern volatile float g_turnStartYaw;
extern volatile float g_turnRequestedAngle;
extern volatile uint8_t g_lineCenterStableCount;
extern volatile uint8_t g_lineSevereRecovery;
extern volatile float g_lineYawWeight;
extern volatile int8_t g_cornerDetectedDir;
extern volatile int8_t g_lastCornerDirection;
extern volatile uint8_t g_cornerGlobalTrigger;

#endif
