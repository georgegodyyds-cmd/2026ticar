#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdint.h>

void OLED_Display_Init(void);
void OLED_Display_Task10ms(void);

extern volatile uint8_t g_oledReady;
extern volatile uint8_t g_oledAddress;
extern volatile uint8_t g_oledErrorStage;

#endif
