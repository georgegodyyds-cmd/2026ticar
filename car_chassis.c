#include "car_chassis.h"

#include "pid_controller.h"
#include "ti_msp_dl_config.h"

/* PWM 周期为 SysConfig 里的 3200，比较值范围 0~3200。 */
#define PWM_MAX                 (3200)
#define OPEN_LOOP_DIAG          (0U)
#define STARTUP_MOTOR_TEST      (0U)
#define STARTUP_TEST_PWM        (850U)
#define STARTUP_TEST_DELAY      (16000000U)

/* 电机静摩擦补偿，PWM 太小时电机可能只响不转。 */
#define PWM_MIN_RUN             (260)

/* 左右电机基础 PWM，不改你的 PA12/PA13 和 TB6612 接线。 */
#define PWM_BASE_LEFT           (750.0f)
#define PWM_BASE_RIGHT          (750.0f)
#define PWM_ADJUST_LIMIT_LEFT   (500.0f)
#define PWM_ADJUST_LIMIT_RIGHT  (500.0f)

#define CTRL_TICKS_PER_UPDATE   (5U)
#define SPEED_FILTER_ALPHA      (0.35f)
#define DEFAULT_SPEED_TARGET    (18.0f)
#define SPEED_TARGET_STEP       (0.75f)

#define KP_SPEED                (30.0f)
#define KI_SPEED                (1.9f)
#define KD_SPEED                (0.5f)

#define KP_BALANCE              (58.0f)
#define KI_BALANCE              (2.2f)
#define KD_BALANCE              (6.5f)
#define BALANCE_OUTPUT_LIMIT    (2000.0f)

#define KP_DISTANCE_BALANCE     (28.0f)
#define DISTANCE_BALANCE_LIMIT  (2600.0f)

#define PID_INTEGRAL_LIMIT      (140.0f)

#define ENCODER_LOST_COUNT      (3U)
#define ENCODER_ALIVE_PULSE     (2)
#define SWAP_ENCODER_LEFT_RIGHT (0U)

/*
 * 编码器测量：
 *   右轮 PA17(A) -> TIMG7_CCP0 捕获，PA18(B) 读方向。
 *   左轮 PB10(A) -> GPIOB 下降沿中断，PB11(B) 读方向。
 */
#define ENCODER_CAPTURE_ENABLE  (1U)
#define LEFT_ENCODER_A_IOMUX    (IOMUX_PINCM27)
#define LEFT_ENCODER_A_PIN      (DL_GPIO_PIN_10)
#define LEFT_ENCODER_A_IIDX     (DL_GPIO_IIDX_DIO10)
#define LEFT_ENCODER_B_IOMUX    (IOMUX_PINCM28)
#define LEFT_ENCODER_B_PIN      (DL_GPIO_PIN_11)
#define LEFT_ENCODER_GPIO_IRQ_ENABLE (1U)

volatile int32_t g_leftPulse  = 0;
volatile int32_t g_rightPulse = 0;
volatile uint8_t g_speedCtrlFlag = 0;

/* 这些变量保留为全局，方便在 CCS Watch 里观察调参。 */
volatile int32_t g_leftSpeed  = 0;
volatile int32_t g_rightSpeed = 0;
volatile int32_t g_leftDistance = 0;
volatile int32_t g_rightDistance = 0;

volatile int16_t motor_left_pwm  = 0;
volatile int16_t motor_right_pwm = 0;
volatile float g_straightAdjust = 0.0f;
volatile float g_pwmBalance = 0.0f;
volatile float g_distanceBalance = 0.0f;
volatile float g_leftTarget = 0.0f;
volatile float g_rightTarget = 0.0f;
volatile float g_leftSpeedFilt = 0.0f;
volatile float g_rightSpeedFilt = 0.0f;
volatile float g_speedDiff = 0.0f;
volatile float g_turnCorrection = 0.0f;
volatile uint8_t g_openLoopEnable = 0U;
volatile float g_openLoopBasePwm = 0.0f;
volatile uint8_t g_leftEncoderLost = 0;
volatile uint8_t g_rightEncoderLost = 0;
volatile uint32_t g_leftCaptureIrq = 0;
volatile uint32_t g_rightCaptureIrq = 0;
volatile uint8_t g_motorPivotActive = 0U;
volatile int8_t g_motorLeftDirection = 0;
volatile int8_t g_motorRightDirection = 0;

static PID_Controller_t g_leftSpeedPid;
static PID_Controller_t g_rightSpeedPid;
static PID_Controller_t g_balancePid;

static uint8_t g_ctrlTickCount = 0U;
static uint8_t g_leftEncoderLostCount = 0U;
static uint8_t g_rightEncoderLostCount = 0U;
static float g_speedTarget = 0.0f;
static float g_speedTargetCmd = DEFAULT_SPEED_TARGET;

static void Motor_Stop(void);

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

static float ramp_target(float current, float target, float step)
{
    if (current < target) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static float low_pass_speed(float last, float sample)
{
    return last + SPEED_FILTER_ALPHA * (sample - last);
}

static void Encoder_CaptureInit(void)
{
#if ENCODER_CAPTURE_ENABLE
    static const DL_TimerG_ClockConfig encoderTimerClockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    static const DL_TimerG_CaptureConfig encoderCaptureConfig = {
        .captureMode = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
        .period = 65535U,
        .startTimer = DL_TIMER_STOP,
        .edgeCaptMode = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
        .inputChan = DL_TIMER_INPUT_CHAN_0,
        .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT,
    };

    /* 右轮 PA17(A) 交给 TIMG7_CCP0 捕获，PA18(B) 保持 GPIO 输入。 */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_GRP_6_AA_IOMUX, IOMUX_PINCM39_PF_TIMG7_CCP0,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 左轮 PB10/PB11 使用 GPIO，避开 PA26/PA27 的板载模拟电路。 */
    DL_GPIO_initDigitalInputFeatures(
        LEFT_ENCODER_A_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        LEFT_ENCODER_B_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_disableInterrupt(GPIOA, GPIO_GRP_6_AA_PIN);
    DL_GPIO_disableInterrupt(GPIOA, GPIO_GRP_8_AA2_PIN | GPIO_GRP_9_AB2_PIN);
    DL_GPIO_clearInterruptStatus(GPIOA,
        GPIO_GRP_6_AA_PIN | GPIO_GRP_8_AA2_PIN | GPIO_GRP_9_AB2_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, LEFT_ENCODER_A_PIN);

#if LEFT_ENCODER_GPIO_IRQ_ENABLE
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_10_EDGE_FALL);
    DL_GPIO_enableInterrupt(GPIOB, LEFT_ENCODER_A_PIN);
    NVIC_SetPriority(GPIOB_INT_IRQn, 0);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
#else
    DL_GPIO_disableInterrupt(GPIOB, LEFT_ENCODER_A_PIN);
    NVIC_DisableIRQ(GPIOB_INT_IRQn);
#endif

    /* SysConfig 仍有 QEI_0，这里把 PB10/PB11 重新切回普通 GPIO。 */
    DL_GPIO_initDigitalInput(GPIO_QEI_0_PHA_IOMUX);
    DL_GPIO_initDigitalInput(GPIO_QEI_0_PHB_IOMUX);
    DL_GPIO_initDigitalInput(GPIO_QEI_0_IDX_IOMUX);

    DL_TimerG_reset(TIMG7);
    DL_TimerG_reset(TIMG8);
    DL_TimerG_enablePower(TIMG7);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_TimerG_setClockConfig(
        TIMG7, (DL_TimerG_ClockConfig *) &encoderTimerClockConfig);
    DL_TimerG_initCaptureMode(
        TIMG7, (DL_TimerG_CaptureConfig *) &encoderCaptureConfig);
    DL_TimerG_setCCPDirection(TIMG7, DL_TIMER_CC0_INPUT);
    DL_TimerG_enableInterrupt(TIMG7, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
    NVIC_SetPriority(TIMG7_INT_IRQn, 0);
    NVIC_EnableIRQ(TIMG7_INT_IRQn);
    DL_TimerG_enableClock(TIMG7);
    DL_TimerG_startCounter(TIMG7);
#endif
}

static void Motor_Forward(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = pwm_from_command((float) left_pwm);
    right_pwm = pwm_from_command((float) right_pwm);

    /* TB6612 正转方向：右电机 PB1/PB2，左电机 PB3/PB4。 */
    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_AN1_PIN);
    DL_GPIO_setPins(GPIO_GRP_1_PORT, GPIO_GRP_1_AN2_PIN);
    DL_GPIO_setPins(GPIO_GRP_2_PORT, GPIO_GRP_2_BN1_PIN);
    DL_GPIO_clearPins(GPIO_GRP_3_PORT, GPIO_GRP_3_BIN2_PIN);
    DL_GPIO_setPins(GPIO_GRP_4_PORT, GPIO_GRP_4_STAYBY_PIN);
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
    int16_t pwm;
    float magnitude;

    if (turn_pwm == 0.0f) {
        Motor_Stop();
        return;
    }

    /* 原地转弯允许使用低于直行静摩擦补偿值的PWM，便于接近目标时慢转。 */
    magnitude = (turn_pwm > 0.0f) ? turn_pwm : -turn_pwm;
    if (magnitude > (float) PWM_MAX) {
        magnitude = (float) PWM_MAX;
    }
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

static void Motor_Stop(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    DL_GPIO_clearPins(GPIO_GRP_4_PORT, GPIO_GRP_4_STAYBY_PIN);
    g_motorPivotActive = 0U;
    g_motorLeftDirection = 0;
    g_motorRightDirection = 0;
}

static void Motor_ChannelTest(void)
{
#if STARTUP_MOTOR_TEST
    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_AN1_PIN);
    DL_GPIO_setPins(GPIO_GRP_1_PORT, GPIO_GRP_1_AN2_PIN);
    DL_GPIO_setPins(GPIO_GRP_2_PORT, GPIO_GRP_2_BN1_PIN);
    DL_GPIO_clearPins(GPIO_GRP_3_PORT, GPIO_GRP_3_BIN2_PIN);
    DL_GPIO_setPins(GPIO_GRP_4_PORT, GPIO_GRP_4_STAYBY_PIN);

    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, STARTUP_TEST_PWM, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    delay_cycles(STARTUP_TEST_DELAY);

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        PWM_0_INST, STARTUP_TEST_PWM, GPIO_PWM_0_C1_IDX);
    delay_cycles(STARTUP_TEST_DELAY);

    Motor_Stop();
    delay_cycles(STARTUP_TEST_DELAY / 4U);
#endif
}

void CarChassis_Init(void)
{
    PID_Init(&g_leftSpeedPid, KP_SPEED, KI_SPEED, KD_SPEED,
        PID_INTEGRAL_LIMIT, -PWM_ADJUST_LIMIT_LEFT, PWM_ADJUST_LIMIT_LEFT);
    PID_Init(&g_rightSpeedPid, KP_SPEED, KI_SPEED, KD_SPEED,
        PID_INTEGRAL_LIMIT, -PWM_ADJUST_LIMIT_RIGHT, PWM_ADJUST_LIMIT_RIGHT);
    PID_Init(&g_balancePid, KP_BALANCE, KI_BALANCE, KD_BALANCE,
        PID_INTEGRAL_LIMIT, -BALANCE_OUTPUT_LIMIT, BALANCE_OUTPUT_LIMIT);

    g_leftDistance = 0;
    g_rightDistance = 0;
    g_speedTarget = 0.0f;
    g_speedTargetCmd = DEFAULT_SPEED_TARGET;
    g_turnCorrection = 0.0f;
    g_openLoopEnable = 0U;
    g_openLoopBasePwm = 0.0f;

    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
    DL_TimerG_startCounter(PWM_0_INST);
    DL_TimerG_startCounter(TIMER_1_INST);

    Motor_Stop();
    Motor_ChannelTest();
    Encoder_CaptureInit();
}

uint8_t CarChassis_Consume10msFlag(void)
{
    uint8_t flag = g_speedCtrlFlag;
    g_speedCtrlFlag = 0U;
    return flag;
}

void CarChassis_SetSpeedTarget(float target_pulse_50ms)
{
    if (target_pulse_50ms < 0.0f) {
        target_pulse_50ms = 0.0f;
    }
    g_speedTargetCmd = target_pulse_50ms;
}

void CarChassis_SetTurnCorrection(float correction_pwm)
{
    g_turnCorrection = PID_Clamp(correction_pwm, -700.0f, 700.0f);
}

void CarChassis_SetOpenLoopPWM(float base_pwm, float correction_pwm)
{
    if (base_pwm < 0.0f) {
        base_pwm = 0.0f;
    }
    g_openLoopEnable = 1U;
    g_openLoopBasePwm = PID_Clamp(base_pwm, 0.0f, (float) PWM_MAX);
    g_turnCorrection = PID_Clamp(correction_pwm, -700.0f, 700.0f);

    motor_left_pwm = pwm_from_command(g_openLoopBasePwm - g_turnCorrection);
    motor_right_pwm = pwm_from_command(g_openLoopBasePwm + g_turnCorrection);
    if (g_openLoopBasePwm <= 0.0f) {
        Motor_Stop();
    } else {
        Motor_Forward(motor_left_pwm, motor_right_pwm);
    }
}

void CarChassis_SetPivotPWM(float turn_pwm)
{
    g_openLoopEnable = 2U;
    g_openLoopBasePwm = 0.0f;
    g_turnCorrection = PID_Clamp(turn_pwm, -700.0f, 700.0f);
    Motor_Pivot(g_turnCorrection);
}

void CarChassis_DirectForwardPWM(int16_t left_pwm, int16_t right_pwm)
{
    motor_left_pwm = pwm_from_command((float) left_pwm);
    motor_right_pwm = pwm_from_command((float) right_pwm);
    Motor_Forward(motor_left_pwm, motor_right_pwm);
}

void CarChassis_DirectStop(void)
{
    motor_left_pwm = 0;
    motor_right_pwm = 0;
    Motor_Stop();
}

void CarChassis_ResetDistance(void)
{
    __disable_irq();
    g_leftDistance = 0;
    g_rightDistance = 0;
    __enable_irq();
}

CarChassis_State_t CarChassis_GetState(void)
{
    CarChassis_State_t state;
    state.left_speed = g_leftSpeed;
    state.right_speed = g_rightSpeed;
    state.left_distance = g_leftDistance;
    state.right_distance = g_rightDistance;
    state.speed_diff = g_speedDiff;
    state.pwm_balance = g_pwmBalance;
    state.distance_balance = g_distanceBalance;
    return state;
}

void CarChassis_Task10ms(void)
{
    int32_t left_speed;
    int32_t right_speed;
    int32_t distance_error;
    float straight_adjust;
    float pwm_balance;
    float distance_balance;
    float left_speed_f;
    float right_speed_f;
    float avg_speed_f;
    float speed_output;

    g_ctrlTickCount++;
    if (g_ctrlTickCount < CTRL_TICKS_PER_UPDATE) {
        return;
    }
    g_ctrlTickCount = 0U;

    __disable_irq();
    left_speed = g_leftPulse;
    right_speed = g_rightPulse;
    g_leftPulse = 0;
    g_rightPulse = 0;
    __enable_irq();

    if (left_speed < 0) {
        left_speed = -left_speed;
    }
    if (right_speed < 0) {
        right_speed = -right_speed;
    }

#if SWAP_ENCODER_LEFT_RIGHT
    {
        int32_t temp_speed = left_speed;
        left_speed = right_speed;
        right_speed = temp_speed;
    }
#endif

    g_leftSpeed = left_speed;
    g_rightSpeed = right_speed;
    g_leftDistance += left_speed;
    g_rightDistance += right_speed;

    left_speed_f = low_pass_speed(g_leftSpeedFilt, (float) left_speed);
    right_speed_f = low_pass_speed(g_rightSpeedFilt, (float) right_speed);
    avg_speed_f = (left_speed_f + right_speed_f) * 0.5f;
    g_leftSpeedFilt = left_speed_f;
    g_rightSpeedFilt = right_speed_f;

    if (g_openLoopEnable != 0U) {
        PID_Reset(&g_leftSpeedPid);
        PID_Reset(&g_rightSpeedPid);
        PID_Reset(&g_balancePid);

        g_speedTarget = 0.0f;
        g_speedTargetCmd = 0.0f;
        g_straightAdjust = left_speed_f - right_speed_f;
        g_pwmBalance = 0.0f;
        g_distanceBalance = 0.0f;
        g_speedDiff = g_straightAdjust;
        g_leftTarget = g_openLoopBasePwm;
        g_rightTarget = g_openLoopBasePwm;

        /* 原地转弯输出由10ms角度环直接更新，这里不能覆盖成正转。 */
        if (g_openLoopEnable == 2U) {
            return;
        }

        motor_left_pwm = pwm_from_command(
            g_openLoopBasePwm - g_turnCorrection);
        motor_right_pwm = pwm_from_command(
            g_openLoopBasePwm + g_turnCorrection);

        if (g_openLoopBasePwm <= 0.0f) {
            Motor_Stop();
        } else {
            Motor_Forward(motor_left_pwm, motor_right_pwm);
        }
        return;
    }

    g_speedTarget = ramp_target(
        g_speedTarget, g_speedTargetCmd, SPEED_TARGET_STEP);

    if (g_speedTarget > 1.0f &&
        left_speed >= ENCODER_ALIVE_PULSE && right_speed == 0) {
        if (g_rightEncoderLostCount < ENCODER_LOST_COUNT) {
            g_rightEncoderLostCount++;
        }
    } else {
        g_rightEncoderLostCount = 0U;
    }

    if (g_speedTarget > 1.0f &&
        right_speed >= ENCODER_ALIVE_PULSE && left_speed == 0) {
        if (g_leftEncoderLostCount < ENCODER_LOST_COUNT) {
            g_leftEncoderLostCount++;
        }
    } else {
        g_leftEncoderLostCount = 0U;
    }

    g_rightEncoderLost =
        (g_rightEncoderLostCount >= ENCODER_LOST_COUNT) ? 1U : 0U;
    g_leftEncoderLost =
        (g_leftEncoderLostCount >= ENCODER_LOST_COUNT) ? 1U : 0U;

    straight_adjust = left_speed_f - right_speed_f;
    pwm_balance = PID_Calculate(
        &g_balancePid, 0.0f, right_speed_f - left_speed_f);

    distance_error = g_leftDistance - g_rightDistance;
    distance_balance = KP_DISTANCE_BALANCE * (float) distance_error;
    distance_balance = PID_Clamp(distance_balance,
        -DISTANCE_BALANCE_LIMIT, DISTANCE_BALANCE_LIMIT);

    pwm_balance += distance_balance;
    pwm_balance = PID_Clamp(pwm_balance,
        -BALANCE_OUTPUT_LIMIT, BALANCE_OUTPUT_LIMIT);

#if OPEN_LOOP_DIAG
    motor_left_pwm = pwm_from_command(PWM_BASE_LEFT);
    motor_right_pwm = pwm_from_command(PWM_BASE_RIGHT);
#else
    speed_output =
        PID_Calculate(&g_leftSpeedPid, g_speedTarget, avg_speed_f);

    /*
     * g_turnCorrection 给循迹/角度环使用：
     *   正值：左轮减、右轮加；
     *   负值：左轮加、右轮减。
     */
    motor_left_pwm = pwm_from_command(
        PWM_BASE_LEFT + speed_output - pwm_balance - g_turnCorrection);
    motor_right_pwm = pwm_from_command(
        PWM_BASE_RIGHT + speed_output + pwm_balance + g_turnCorrection);
#endif

    if (g_leftEncoderLost != 0U) {
        PID_Reset(&g_leftSpeedPid);
        PID_Reset(&g_rightSpeedPid);
        PID_Reset(&g_balancePid);
        motor_left_pwm = pwm_from_command(PWM_BASE_LEFT);
    }
    if (g_rightEncoderLost != 0U) {
        PID_Reset(&g_leftSpeedPid);
        PID_Reset(&g_rightSpeedPid);
        PID_Reset(&g_balancePid);
        motor_right_pwm = pwm_from_command(PWM_BASE_RIGHT);
    }

    g_straightAdjust = straight_adjust;
    g_pwmBalance = pwm_balance;
    g_distanceBalance = distance_balance;
    g_speedDiff = straight_adjust;
    g_leftTarget = g_speedTarget;
    g_rightTarget = g_speedTarget;

    if (g_speedTarget <= 0.0f) {
        PID_Reset(&g_leftSpeedPid);
        PID_Reset(&g_rightSpeedPid);
        PID_Reset(&g_balancePid);
        Motor_Stop();
    } else {
        Motor_Forward(motor_left_pwm, motor_right_pwm);
    }
}

void GROUP0_IRQHandler(void)
{
#if LEFT_ENCODER_GPIO_IRQ_ENABLE
    DL_GPIO_IIDX pending;
    while ((pending = DL_GPIO_getPendingInterrupt(GPIOB)) !=
        DL_GPIO_IIDX_NO_INTR) {
        if (pending == LEFT_ENCODER_A_IIDX) {
            g_leftCaptureIrq++;
            if (DL_GPIO_readPins(GPIOB, LEFT_ENCODER_B_PIN) != 0U) {
                g_leftPulse++;
            } else {
                g_leftPulse--;
            }
        }
        DL_GPIO_clearInterruptStatus(GPIOB, LEFT_ENCODER_A_PIN);
    }
#else
    DL_GPIO_clearInterruptStatus(GPIOB, LEFT_ENCODER_A_PIN);
#endif
}

void GROUP1_IRQHandler(void)
{
    DL_GPIO_IIDX pending;

    while ((pending = DL_GPIO_getPendingInterrupt(GPIOB)) !=
        DL_GPIO_IIDX_NO_INTR) {
        if (pending == LEFT_ENCODER_A_IIDX) {
            g_leftCaptureIrq++;
            if (DL_GPIO_readPins(GPIOB, LEFT_ENCODER_B_PIN) != 0U) {
                g_leftPulse++;
            } else {
                g_leftPulse--;
            }
        }
        DL_GPIO_clearInterruptStatus(GPIOB, LEFT_ENCODER_A_PIN);
    }

    while ((pending = DL_GPIO_getPendingInterrupt(GPIOA)) !=
        DL_GPIO_IIDX_NO_INTR) {
        switch (pending) {
            case GPIO_GRP_6_AA_IIDX:
                DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_6_AA_PIN);
                break;

            case GPIO_GRP_8_AA2_IIDX:
                g_leftCaptureIrq++;
                if (DL_GPIO_readPins(GPIO_GRP_9_PORT,
                    GPIO_GRP_9_AB2_PIN) != 0U) {
                    g_leftPulse++;
                } else {
                    g_leftPulse--;
                }
                DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_8_AA2_PIN);
                break;

            case DL_GPIO_IIDX_DIO27:
                g_leftCaptureIrq++;
                if (DL_GPIO_readPins(GPIO_GRP_8_PORT,
                    GPIO_GRP_8_AA2_PIN) != 0U) {
                    g_leftPulse++;
                } else {
                    g_leftPulse--;
                }
                DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_9_AB2_PIN);
                break;

            default:
                DL_GPIO_clearInterruptStatus(GPIOA,
                    GPIO_GRP_6_AA_PIN | GPIO_GRP_8_AA2_PIN |
                    GPIO_GRP_9_AB2_PIN);
                break;
        }
    }
}

void TIMG7_IRQHandler(void)
{
#if ENCODER_CAPTURE_ENABLE
    switch (DL_TimerG_getPendingInterrupt(TIMG7)) {
        case DL_TIMERG_IIDX_CC0_DN:
            g_rightCaptureIrq++;
            if (DL_GPIO_readPins(GPIO_GRP_7_PORT, GPIO_GRP_7_AB_PIN) != 0U) {
                g_rightPulse++;
            } else {
                g_rightPulse--;
            }
            DL_TimerG_clearInterruptStatus(
                TIMG7, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
            break;

        case DL_TIMERG_IIDX_CC1_DN:
            g_leftCaptureIrq++;
            if (DL_GPIO_readPins(GPIO_GRP_8_PORT,
                GPIO_GRP_8_AA2_PIN) != 0U) {
                g_leftPulse++;
            } else {
                g_leftPulse--;
            }
            DL_TimerG_clearInterruptStatus(
                TIMG7, DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
            break;

        default:
            break;
    }
#endif
}

void TIMG8_IRQHandler(void)
{
#if ENCODER_CAPTURE_ENABLE
    (void) DL_TimerG_getPendingInterrupt(TIMG8);
#endif
}

void TIMG6_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_1_INST) == DL_TIMER_IIDX_LOAD) {
        DL_TimerG_clearInterruptStatus(
            TIMER_1_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
        g_speedCtrlFlag = 1U;
    }
}
