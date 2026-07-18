#include "car_chassis.h"

#include "ti_msp_dl_config.h"

/* PWM周期由SysConfig配置为3200。 */
#define PWM_MAX                  (3200)
#define PWM_MIN_RUN              (260)
#define TURN_CORRECTION_LIMIT    (700.0f)

volatile int16_t motor_left_pwm = 0;
volatile int16_t motor_right_pwm = 0;
volatile float g_turnCorrection = 0.0f;
volatile float g_openLoopBasePwm = 0.0f;
volatile uint8_t g_motorPivotActive = 0U;
volatile int8_t g_motorLeftDirection = 0;
volatile int8_t g_motorRightDirection = 0;

static volatile uint8_t g_controlTickFlag = 0U;

static int16_t pwm_from_command(float value)
{
    if (value >= (float) PWM_MAX) {
        return PWM_MAX;
    }
    if (value <= 0.0f) {
        return 0;
    }
    if (value < (float) PWM_MIN_RUN) {
        return PWM_MIN_RUN;
    }
    return (int16_t) value;
}

static float clamp_value(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void Motor_Stop(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0U, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0U, GPIO_PWM_0_C1_IDX);
    DL_GPIO_clearPins(GPIO_GRP_4_PORT, GPIO_GRP_4_STAYBY_PIN);

    motor_left_pwm = 0;
    motor_right_pwm = 0;
    g_motorPivotActive = 0U;
    g_motorLeftDirection = 0;
    g_motorRightDirection = 0;
}

static void Motor_Forward(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = pwm_from_command((float) left_pwm);
    right_pwm = pwm_from_command((float) right_pwm);

    /* 正转方向保持原接线：右电机PB1/PB2，左电机PB3/PB4。 */
    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_AN1_PIN);
    DL_GPIO_setPins(GPIO_GRP_1_PORT, GPIO_GRP_1_AN2_PIN);
    DL_GPIO_setPins(GPIO_GRP_2_PORT, GPIO_GRP_2_BN1_PIN);
    DL_GPIO_clearPins(GPIO_GRP_3_PORT, GPIO_GRP_3_BIN2_PIN);
    DL_GPIO_setPins(GPIO_GRP_4_PORT, GPIO_GRP_4_STAYBY_PIN);

    motor_left_pwm = left_pwm;
    motor_right_pwm = right_pwm;
    g_motorPivotActive = 0U;
    g_motorLeftDirection = 1;
    g_motorRightDirection = 1;

    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, (uint32_t) right_pwm, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, (uint32_t) left_pwm, GPIO_PWM_0_C1_IDX);
}

static void Motor_Pivot(float turn_pwm)
{
    float magnitude;
    int16_t pwm;

    if (turn_pwm == 0.0f) {
        Motor_Stop();
        return;
    }

    magnitude = (turn_pwm > 0.0f) ? turn_pwm : -turn_pwm;
    magnitude = clamp_value(magnitude, 0.0f, (float) PWM_MAX);
    pwm = (int16_t) magnitude;
    if (pwm <= 0) {
        Motor_Stop();
        return;
    }

    DL_GPIO_setPins(GPIO_GRP_4_PORT, GPIO_GRP_4_STAYBY_PIN);
    if (turn_pwm > 0.0f) {
        /* 原地左转：左轮反转，右轮正转。 */
        DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_AN1_PIN);
        DL_GPIO_setPins(GPIO_GRP_1_PORT, GPIO_GRP_1_AN2_PIN);
        DL_GPIO_clearPins(GPIO_GRP_2_PORT, GPIO_GRP_2_BN1_PIN);
        DL_GPIO_setPins(GPIO_GRP_3_PORT, GPIO_GRP_3_BIN2_PIN);
        motor_left_pwm = -pwm;
        motor_right_pwm = pwm;
        g_motorLeftDirection = -1;
        g_motorRightDirection = 1;
    } else {
        /* 原地右转：左轮正转，右轮反转。 */
        DL_GPIO_setPins(GPIO_GRP_0_PORT, GPIO_GRP_0_AN1_PIN);
        DL_GPIO_clearPins(GPIO_GRP_1_PORT, GPIO_GRP_1_AN2_PIN);
        DL_GPIO_setPins(GPIO_GRP_2_PORT, GPIO_GRP_2_BN1_PIN);
        DL_GPIO_clearPins(GPIO_GRP_3_PORT, GPIO_GRP_3_BIN2_PIN);
        motor_left_pwm = pwm;
        motor_right_pwm = -pwm;
        g_motorLeftDirection = 1;
        g_motorRightDirection = -1;
    }
    g_motorPivotActive = 1U;

    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, (uint32_t) pwm, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, (uint32_t) pwm, GPIO_PWM_0_C1_IDX);
}

void CarChassis_Init(void)
{
    g_controlTickFlag = 0U;
    g_turnCorrection = 0.0f;
    g_openLoopBasePwm = 0.0f;

    DL_TimerG_clearInterruptStatus(
        TIMER_1_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(TIMER_1_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);

    DL_TimerG_startCounter(PWM_0_INST);
    DL_TimerG_startCounter(TIMER_1_INST);
    Motor_Stop();
}

uint8_t CarChassis_Consume10msFlag(void)
{
    uint8_t flag = g_controlTickFlag;
    g_controlTickFlag = 0U;
    return flag;
}

void CarChassis_SetOpenLoopPWM(float base_pwm, float correction_pwm)
{
    base_pwm = clamp_value(base_pwm, 0.0f, (float) PWM_MAX);
    correction_pwm = clamp_value(correction_pwm,
        -TURN_CORRECTION_LIMIT, TURN_CORRECTION_LIMIT);

    g_openLoopBasePwm = base_pwm;
    g_turnCorrection = correction_pwm;
    if (base_pwm <= 0.0f) {
        Motor_Stop();
        return;
    }

    Motor_Forward(
        pwm_from_command(base_pwm - correction_pwm),
        pwm_from_command(base_pwm + correction_pwm));
}

void CarChassis_SetPivotPWM(float turn_pwm)
{
    g_openLoopBasePwm = 0.0f;
    g_turnCorrection = clamp_value(turn_pwm,
        -TURN_CORRECTION_LIMIT, TURN_CORRECTION_LIMIT);
    Motor_Pivot(g_turnCorrection);
}

void TIMG6_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_1_INST) == DL_TIMER_IIDX_LOAD) {
        DL_TimerG_clearInterruptStatus(
            TIMER_1_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
        g_controlTickFlag = 1U;
    }
}
