#ifndef CAR_CHASSIS_H
#define CAR_CHASSIS_H

#include <stdint.h>

#define CAR_CHASSIS_CTRL_TICK_MS       (10U)
#define CAR_CHASSIS_SPEED_SAMPLE_MS    (50U)

typedef struct {
    int32_t left_speed;
    int32_t right_speed;
    int32_t left_distance;
    int32_t right_distance;
    float speed_diff;
    float pwm_balance;
    float distance_balance;
} CarChassis_State_t;

void CarChassis_Init(void);
uint8_t CarChassis_Consume10msFlag(void);
void CarChassis_Task10ms(void);
void CarChassis_SetSpeedTarget(float target_pulse_50ms);
void CarChassis_SetTurnCorrection(float correction_pwm);
void CarChassis_SetOpenLoopPWM(float base_pwm, float correction_pwm);
/* turn_pwm>0原地左转，turn_pwm<0原地右转。 */
void CarChassis_SetPivotPWM(float turn_pwm);
void CarChassis_DirectForwardPWM(int16_t left_pwm, int16_t right_pwm);
void CarChassis_DirectStop(void);
void CarChassis_ResetDistance(void);
CarChassis_State_t CarChassis_GetState(void);

extern volatile uint8_t g_motorPivotActive;
extern volatile int8_t g_motorLeftDirection;
extern volatile int8_t g_motorRightDirection;

#endif
