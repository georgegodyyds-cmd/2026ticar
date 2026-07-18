#include "lap_control.h"

#define TURNS_PER_LAP               (4U)
#define TRACK_STATE_TURN            (1U)
#define TRACK_STATE_REACQUIRE       (2U)

volatile uint8_t g_targetLaps = 0U;
volatile uint8_t g_completedLaps = 0U;
volatile uint8_t g_completedTurns = 0U;
volatile uint8_t g_lapRunning = 0U;

static uint8_t g_previousTrackState = 0U;

void LapControl_Init(void)
{
    g_targetLaps = 0U;
    g_completedLaps = 0U;
    g_completedTurns = 0U;
    g_lapRunning = 0U;
    g_previousTrackState = 0U;
}

void LapControl_Start(uint8_t target_laps)
{
    if (target_laps < 1U || target_laps > 5U) {
        return;
    }

    g_targetLaps = target_laps;
    g_completedLaps = 0U;
    g_completedTurns = 0U;
    g_lapRunning = 1U;
    g_previousTrackState = 0U;
}

uint8_t LapControl_Update(uint8_t track_state)
{
    uint8_t finished = 0U;

    if (g_lapRunning != 0U &&
        g_previousTrackState == TRACK_STATE_TURN &&
        track_state == TRACK_STATE_REACQUIRE) {
        g_completedTurns++;
        if (g_completedTurns >= TURNS_PER_LAP) {
            g_completedTurns = 0U;
            g_completedLaps++;
            if (g_completedLaps >= g_targetLaps) {
                g_lapRunning = 0U;
                finished = 1U;
            }
        }
    }

    g_previousTrackState = track_state;
    return finished;
}

uint8_t LapControl_IsRunning(void)
{
    return g_lapRunning;
}
