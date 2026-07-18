#ifndef LAP_CONTROL_H
#define LAP_CONTROL_H

#include <stdint.h>

void LapControl_Init(void);
void LapControl_Start(uint8_t target_laps);
uint8_t LapControl_Update(uint8_t track_state);
uint8_t LapControl_IsRunning(void);

extern volatile uint8_t g_targetLaps;
extern volatile uint8_t g_completedLaps;
extern volatile uint8_t g_completedTurns;
extern volatile uint8_t g_lapRunning;

#endif
