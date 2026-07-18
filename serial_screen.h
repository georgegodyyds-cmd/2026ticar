#ifndef SERIAL_SCREEN_H
#define SERIAL_SCREEN_H

#include <stdint.h>

void SerialScreen_Init(void);
uint8_t SerialScreen_ConsumeLapCommand(uint8_t *laps);

extern volatile uint8_t g_serialLastByte;
extern volatile uint8_t g_serialLapCommand;
extern volatile uint32_t g_serialRxCount;

#endif
