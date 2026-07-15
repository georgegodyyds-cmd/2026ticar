#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

#define LINE_SENSOR_COUNT  (7U)

typedef struct {
    uint8_t seen;
    uint8_t mask;
    int16_t error;      /* -3000 到 +3000，负数表示线偏左，正数表示线偏右 */
    uint8_t active_num;
} LineSensor_State_t;

void LineSensor_Init(void);
LineSensor_State_t LineSensor_Update(void);

extern volatile uint8_t g_lineMask;
extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineSeen;

#endif
