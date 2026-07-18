#ifndef CAR_CHASSIS_H
#define CAR_CHASSIS_H

#include <stdint.h>

#define CAR_CHASSIS_CTRL_TICK_MS  (10U)

void CarChassis_Init(void);
uint8_t CarChassis_Consume10msFlag(void);
void CarChassis_SetOpenLoopPWM(float base_pwm, float correction_pwm);

/* turn_pwm>0原地左转，turn_pwm<0原地右转。 */
void CarChassis_SetPivotPWM(float turn_pwm);

extern volatile int16_t motor_left_pwm;
extern volatile int16_t motor_right_pwm;
extern volatile float g_turnCorrection;
extern volatile float g_openLoopBasePwm;
extern volatile uint8_t g_motorPivotActive;
extern volatile int8_t g_motorLeftDirection;
extern volatile int8_t g_motorRightDirection;

#endif
