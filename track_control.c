#include "track_control.h"

#include "car_chassis.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "pid_controller.h"

/* 直边只使用七路红外巡线。 */
#define EDGE_BASE_PWM                 (700.0f)
#define LINE_KP_NEAR                  (0.700f)
#define LINE_KP_FAR                   (0.900f)
#define LINE_KI                       (0.0015f)
#define LINE_I_ERROR_LIMIT            (1500.0f)
#define LINE_I_OUTPUT_LIMIT           (480.0f)
#define LINE_KD                       (0.000f)
#define LINE_D_LIMIT                  (0.0f)
#define LINE_NEAR_ERROR               (1200.0f)
#define LINE_ERROR_FILTER_ALPHA_NEAR  (1.00f)
#define LINE_ERROR_FILTER_ALPHA_FAR   (1.00f)
#define LINE_DEADBAND                 (0.0f)
#define LINE_CORRECTION_LIMIT         (700.0f)
#define LINE_SEVERE_ERROR             (800.0f)
#define LINE_SEVERE_EXIT_ERROR        (550.0f)
#define LINE_SEVERE_EXIT_COUNT        (4U)
#define LINE_SEVERE_BASE_PWM          (600.0f)
#define LINE_SEVERE_CENTER_PWM        (300.0f)
#define LINE_OUTPUT_RISE_STEP         (500.0f)
#define LINE_OUTPUT_FAR_STEP          (500.0f)
#define LINE_OUTPUT_FAR_THRESHOLD     (150.0f)
#define LINE_OUTPUT_RELEASE_STEP      (500.0f)
#define LINE_GAP_HOLD_COUNT           (0U)
#define LINE_LOST_SEARCH_PWM          (700.0f)
/* 当前底盘的红外纠偏方向：保持原方向，不能反相。 */
#define LINE_STEERING_SIGN             (1.0f)

/* 直角连续确认，避免单帧跳变误触发。 */
#define CORNER_DETECTION_ENABLE       (1U)
#define CORNER_CONFIRM_CENTER_COUNT   (2U)
#define CORNER_CONFIRM_SIDE_COUNT     (2U)
#define CORNER_LOCKOUT_COUNT          (30U)
#define EDGE_ARM_COUNT                (5U)
#define CORNER_TURN_DELAY_COUNT       (6U)

/* 直角转弯只使用MPU，相对触发瞬间原地转90度。 */
#define TURN_ANGLE_DEG                (90.0f)
#define TURN_KP                       (10.0f)
#define TURN_OUTPUT_LIMIT             (600.0f)
#define TURN_MIN_PWM                  (490.0f)
#define TURN_FINE_PWM                 (300.0f)
#define TURN_FINE_ZONE_DEG            (20.0f)
#define TURN_DONE_DEG                 (0.8f)
#define TURN_DONE_COUNT               (3U)
#define TURN_MIN_COUNT                (20U)
#define TURN_REACHED_MARGIN_DEG       (0.1f)
#define TURN_TIMEOUT_COUNT            (350U)
#define POST_TURN_HOLD_COUNT          (8U)

#define LINE_MASK_ALL_WHITE           (0x00U)
#define LINE_MASK_ALL_BLACK           (0x7FU)
#define LINE_MASK_CENTER              (0x08U)
#define TURN_DIR_LEFT                 (1)
#define TURN_DIR_RIGHT                (-1)

#define SQUARE_STATE_EDGE             (0U)
#define SQUARE_STATE_TURN             (1U)
#define SQUARE_STATE_TURN_FINISHED    (2U)

volatile int16_t g_trackMode = TRACK_MODE_AUTO;
volatile int16_t g_activeTrackMode = TRACK_MODE_STRAIGHT;
volatile float g_trackTurnPwm = 0.0f;
volatile float g_yawTarget = 0.0f;
volatile float g_yawError = 0.0f;
volatile int8_t g_squareTurnDir = TURN_DIR_LEFT;
volatile uint8_t g_squareCornerMask = 0U;
volatile uint8_t g_squareControlState = SQUARE_STATE_EDGE;
volatile float g_lineCorrectionPwm = 0.0f;
volatile float g_lineIntegralPwm = 0.0f;
volatile float g_yawCorrectionPwm = 0.0f;
volatile uint8_t g_cornerStableCount = 0U;
volatile uint8_t g_turnDoneStableCount = 0U;
volatile uint8_t g_lineRecoveryCount = 0U;
volatile uint8_t g_cornerLockoutCount = 0U;
volatile uint8_t g_allWhiteHoldActive = 0U;
volatile float g_turnStartYaw = 0.0f;
volatile float g_turnRequestedAngle = TURN_ANGLE_DEG;
volatile uint8_t g_lineCenterStableCount = 0U;
volatile uint8_t g_lineSevereRecovery = 0U;
volatile float g_lineYawWeight = 0.0f;
volatile int8_t g_cornerDetectedDir = 0;
volatile int8_t g_lastCornerDirection = TURN_DIR_LEFT;
volatile uint8_t g_cornerGlobalTrigger = 0U;

static PID_Controller_t g_turnPid;
static float g_filteredLineError = 0.0f;
static float g_lastFilteredLineError = 0.0f;
static float g_lineOutputPwm = 0.0f;
static int16_t g_lastLineError = 0;
static uint16_t g_turnTimeCount = 0U;
static uint8_t g_lineLostCount = 0U;
static uint8_t g_edgeArmCount = 0U;
static uint8_t g_edgeReadyForCorner = 0U;
static uint8_t g_cornerConfirmRequired = CORNER_CONFIRM_SIDE_COUNT;
static int8_t g_cornerCandidateDir = 0;
static uint8_t g_cornerTurnDelayCount = 0U;
static int8_t g_pendingTurnDir = 0;
static uint8_t g_pendingCornerMask = 0U;
static uint8_t g_postTurnHoldCount = 0U;
static int8_t g_severeRecoveryDirection = 0;
static uint8_t g_severeRecoveryExitCount = 0U;
static uint8_t g_internalState = SQUARE_STATE_EDGE;

static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float angle_error_deg(float target, float actual)
{
    return normalize_angle_deg(target - actual);
}

static float abs_value(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float approach_value(float current, float target, float step)
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

static float line_output_approach(float target)
{
    float step = LINE_OUTPUT_RISE_STEP;

    /* 误差换边时先撤销旧方向差速，避免继续把车推过中线。 */
    if ((g_lineOutputPwm > 0.0f && target < 0.0f) ||
        (g_lineOutputPwm < 0.0f && target > 0.0f)) {
        g_lineOutputPwm = 0.0f;
    }

    if (abs_value(target) < abs_value(g_lineOutputPwm) ||
        (g_lineOutputPwm > 0.0f && target <= 0.0f) ||
        (g_lineOutputPwm < 0.0f && target >= 0.0f)) {
        step = LINE_OUTPUT_RELEASE_STEP;
    } else if (abs_value(target) >= LINE_OUTPUT_FAR_THRESHOLD) {
        step = LINE_OUTPUT_FAR_STEP;
    }
    g_lineOutputPwm = approach_value(g_lineOutputPwm, target, step);
    return g_lineOutputPwm;
}

static uint8_t line_is_normal_edge(LineSensor_State_t line)
{
    return (line.mask != LINE_MASK_ALL_WHITE &&
            line.mask != LINE_MASK_ALL_BLACK &&
            line.active_num > 0U && line.active_num <= 2U) ? 1U : 0U;
}

static int8_t detect_corner_direction(uint8_t mask)
{
    uint8_t left_corner;
    uint8_t right_corner;

    /*
     * 同侧三路同时为黑即可判定直角，不要求中心S4为黑：
     * 左侧S1/S2/S3对应bit0~2，右侧S5/S6/S7对应bit4~6。
     */
    left_corner = ((mask & 0x07U) == 0x07U) ? 1U : 0U;
    right_corner = ((mask & 0x70U) == 0x70U) ? 1U : 0U;

    if (left_corner != 0U && right_corner == 0U) {
        return TURN_DIR_LEFT;
    }
    if (right_corner != 0U && left_corner == 0U) {
        return TURN_DIR_RIGHT;
    }
    if (left_corner != 0U && right_corner != 0U) {
        return g_lastCornerDirection;
    }
    return 0;
}

static float line_control(LineSensor_State_t line)
{
    float target;
    float line_kp;
    float derivative;
    float error_delta;
    float abs_error;
    float filter_alpha;
    float integral_error;

    if (line.mask == LINE_MASK_ALL_WHITE) {
        if (g_lineLostCount < 255U) {
            g_lineLostCount++;
        }
        g_allWhiteHoldActive = 1U;
        if (g_lineLostCount <= LINE_GAP_HOLD_COUNT) {
            target = 0.0f;
        } else if (g_lastLineError > (int16_t) LINE_DEADBAND) {
            target = -LINE_LOST_SEARCH_PWM;
        } else if (g_lastLineError < -(int16_t) LINE_DEADBAND) {
            target = LINE_LOST_SEARCH_PWM;
        } else {
            target = 0.0f;
        }
        return line_output_approach(target);
    }

    g_allWhiteHoldActive = 0U;
    if (line.mask == LINE_MASK_ALL_BLACK) {
        return line_output_approach(0.0f);
    }

    g_lineLostCount = 0U;
    /*
     * 只记住最后一次有方向的偏差。探头短暂全白时，
     * 继续按照丢线前的方向找回黑线。
     */
    if (line.error > (int16_t) LINE_DEADBAND ||
        line.error < -(int16_t) LINE_DEADBAND) {
        g_lastLineError = line.error;
    }
    filter_alpha = (abs_value((float) line.error) > LINE_NEAR_ERROR) ?
        LINE_ERROR_FILTER_ALPHA_FAR : LINE_ERROR_FILTER_ALPHA_NEAR;
    g_filteredLineError += filter_alpha *
        ((float) line.error - g_filteredLineError);
    error_delta = g_filteredLineError - g_lastFilteredLineError;
    g_lastFilteredLineError = g_filteredLineError;
    derivative = PID_Clamp(-LINE_KD * error_delta,
        -LINE_D_LIMIT, LINE_D_LIMIT);
    abs_error = abs_value(g_filteredLineError);
    integral_error = PID_Clamp(g_filteredLineError,
        -LINE_I_ERROR_LIMIT, LINE_I_ERROR_LIMIT);
    if (abs_error > LINE_DEADBAND) {
        /*
         * 积分项学习左右电机的固定速度差。回到中心后不清零，
         * 继续保留所需差速，避免只能偏在某个探头上才能走直。
         */
        g_lineIntegralPwm += -LINE_KI * integral_error;
        g_lineIntegralPwm = PID_Clamp(g_lineIntegralPwm,
            -LINE_I_OUTPUT_LIMIT, LINE_I_OUTPUT_LIMIT);
    }

    if (abs_error <= LINE_DEADBAND) {
        target = g_lineIntegralPwm;
    } else {
        line_kp = LINE_KP_NEAR;
        if (abs_error > LINE_NEAR_ERROR) {
            line_kp += (LINE_KP_FAR - LINE_KP_NEAR) *
                (abs_error - LINE_NEAR_ERROR) /
                (3000.0f - LINE_NEAR_ERROR);
            line_kp = PID_Clamp(line_kp, LINE_KP_NEAR, LINE_KP_FAR);
        }
        target = -line_kp * g_filteredLineError +
            g_lineIntegralPwm + derivative;
    }
    target = PID_Clamp(target,
        -LINE_CORRECTION_LIMIT, LINE_CORRECTION_LIMIT);
    return line_output_approach(target);
}

static void reset_line_control(void)
{
    g_filteredLineError = 0.0f;
    g_lastFilteredLineError = 0.0f;
    g_lineOutputPwm = 0.0f;
    g_lastLineError = 0;
    g_lineLostCount = 0U;
    g_lineCorrectionPwm = 0.0f;
    g_lineCenterStableCount = 0U;
    g_lineSevereRecovery = 0U;
    g_severeRecoveryDirection = 0;
    g_severeRecoveryExitCount = 0U;
    g_lineYawWeight = 0.0f;
    g_yawCorrectionPwm = 0.0f;
}

static void start_turn(int8_t direction, uint8_t mask)
{
    g_internalState = SQUARE_STATE_TURN;
    g_squareControlState = SQUARE_STATE_TURN;
    g_activeTrackMode = TRACK_MODE_SQUARE;
    g_squareTurnDir = direction;
    g_lastCornerDirection = direction;
    g_squareCornerMask = mask;
    g_turnStartYaw = g_yawAngle;
    g_turnRequestedAngle = TURN_ANGLE_DEG * (float) direction;
    g_yawTarget = normalize_angle_deg(
        g_turnStartYaw + g_turnRequestedAngle);
    g_yawError = angle_error_deg(g_yawTarget, g_yawAngle);
    g_turnTimeCount = 0U;
    g_turnDoneStableCount = 0U;
    g_cornerStableCount = 0U;
    g_cornerCandidateDir = 0;
    g_cornerGlobalTrigger = 1U;
    PID_Reset(&g_turnPid);
    reset_line_control();
}

static uint8_t turn_control(float *turn_pwm)
{
    float output;
    float magnitude;
    float abs_error;
    float directed_progress;

    g_yawError = angle_error_deg(g_yawTarget, g_yawAngle);
    abs_error = abs_value(g_yawError);
    directed_progress = (float) g_squareTurnDir *
        normalize_angle_deg(g_yawAngle - g_turnStartYaw);
    if (g_turnTimeCount < 65535U) {
        g_turnTimeCount++;
    }

    if (g_turnTimeCount >= TURN_MIN_COUNT && abs_error <= TURN_DONE_DEG) {
        if (g_turnDoneStableCount < TURN_DONE_COUNT) {
            g_turnDoneStableCount++;
        }
    } else {
        g_turnDoneStableCount = 0U;
    }
    /*
     * 电机供电充足后可能一次跨过完成窗口，因此沿目标方向达到角度
     * 就立即结束，不再要求必须在窄角度范围内连续停留。
     */
    if ((g_turnTimeCount >= TURN_MIN_COUNT &&
            directed_progress >=
                (TURN_ANGLE_DEG - TURN_REACHED_MARGIN_DEG)) ||
        g_turnDoneStableCount >= TURN_DONE_COUNT ||
        g_turnTimeCount >= TURN_TIMEOUT_COUNT) {
        *turn_pwm = 0.0f;
        return 1U;
    }

    output = PID_Calculate(&g_turnPid, 0.0f, -g_yawError);
    magnitude = PID_Clamp(abs_value(output), 0.0f, TURN_OUTPUT_LIMIT);
    if (abs_error <= TURN_FINE_ZONE_DEG) {
        magnitude = TURN_FINE_PWM;
    } else if (magnitude < TURN_MIN_PWM) {
        magnitude = TURN_MIN_PWM;
    }
    *turn_pwm = (g_yawError >= 0.0f) ? magnitude : -magnitude;
    return 0U;
}

static void reset_all_control(void)
{
    g_internalState = SQUARE_STATE_EDGE;
    g_squareControlState = SQUARE_STATE_EDGE;
    g_activeTrackMode = TRACK_MODE_STRAIGHT;
    g_turnTimeCount = 0U;
    g_postTurnHoldCount = 0U;
    g_cornerStableCount = 0U;
    g_cornerCandidateDir = 0;
    g_cornerTurnDelayCount = 0U;
    g_pendingTurnDir = 0;
    g_pendingCornerMask = 0U;
    g_cornerLockoutCount = 0U;
    g_edgeArmCount = 0U;
    g_edgeReadyForCorner = 0U;
    g_trackTurnPwm = 0.0f;
    g_yawCorrectionPwm = 0.0f;
    reset_line_control();
    PID_Reset(&g_turnPid);
}

void TrackControl_Init(void)
{
    PID_Init(&g_turnPid, TURN_KP, 0.0f, 0.0f,
        0.0f, -TURN_OUTPUT_LIMIT, TURN_OUTPUT_LIMIT);
    g_lineIntegralPwm = 0.0f;
    reset_all_control();
}

void TrackControl_SetMode(TrackMode_t mode)
{
    g_trackMode = (int16_t) mode;
    reset_all_control();
}

TrackMode_t TrackControl_GetMode(void)
{
    return (TrackMode_t) g_trackMode;
}

void TrackControl_Task10ms(void)
{
    LineSensor_State_t line;
    float correction_pwm;
    float line_pwm;
    float drive_base_pwm;
    float severe_pwm;
    int8_t requested_recovery_direction;
    int8_t detected_dir;

    MPU6050_Task10ms();
    g_cornerGlobalTrigger = 0U;

    if (g_internalState == SQUARE_STATE_TURN) {
        g_squareControlState = SQUARE_STATE_TURN;
        g_activeTrackMode = TRACK_MODE_SQUARE;
        g_lineCorrectionPwm = 0.0f;
        if (g_mpuReady == 0U) {
            g_yawCorrectionPwm = 0.0f;
            CarChassis_SetPivotPWM(0.0f);
            return;
        }
        if (turn_control(&correction_pwm) != 0U) {
            g_internalState = SQUARE_STATE_TURN_FINISHED;
            g_squareControlState = SQUARE_STATE_TURN_FINISHED;
            g_postTurnHoldCount = POST_TURN_HOLD_COUNT;
            g_cornerLockoutCount = CORNER_LOCKOUT_COUNT;
            g_trackTurnPwm = 0.0f;
            g_yawCorrectionPwm = 0.0f;
            CarChassis_SetPivotPWM(0.0f);
        } else {
            g_trackTurnPwm = correction_pwm;
            g_yawCorrectionPwm = correction_pwm;
            CarChassis_SetPivotPWM(correction_pwm);
        }
        return;
    }

    if (g_internalState == SQUARE_STATE_TURN_FINISHED) {
        g_squareControlState = SQUARE_STATE_TURN_FINISHED;
        g_lineCorrectionPwm = 0.0f;
        g_yawCorrectionPwm = 0.0f;
        g_trackTurnPwm = 0.0f;
        if (g_postTurnHoldCount > 0U) {
            g_postTurnHoldCount--;
            CarChassis_SetPivotPWM(0.0f);
            return;
        }
        g_internalState = SQUARE_STATE_EDGE;
        g_squareControlState = SQUARE_STATE_EDGE;
        g_activeTrackMode = TRACK_MODE_STRAIGHT;
        g_edgeArmCount = 0U;
        g_edgeReadyForCorner = 0U;
        reset_line_control();
        CarChassis_SetOpenLoopPWM(EDGE_BASE_PWM, 0.0f);
        return;
    }

    /* EDGE状态：只读取红外并输出红外差速，MPU修正固定为0。 */
    g_squareControlState = SQUARE_STATE_EDGE;
    g_activeTrackMode = TRACK_MODE_STRAIGHT;
    g_yawCorrectionPwm = 0.0f;
    g_yawError = 0.0f;
    if (g_cornerLockoutCount > 0U) {
        g_cornerLockoutCount--;
    }

    if (g_cornerTurnDelayCount > 0U) {
        g_cornerTurnDelayCount--;
        g_lineCorrectionPwm = 0.0f;
        g_trackTurnPwm = 0.0f;
        if (g_cornerTurnDelayCount == 0U) {
            start_turn(g_pendingTurnDir, g_pendingCornerMask);
            if (g_mpuReady != 0U &&
                turn_control(&correction_pwm) == 0U) {
                g_trackTurnPwm = correction_pwm;
                g_yawCorrectionPwm = correction_pwm;
                CarChassis_SetPivotPWM(correction_pwm);
            } else {
                CarChassis_SetPivotPWM(0.0f);
            }
        } else {
            CarChassis_SetOpenLoopPWM(EDGE_BASE_PWM, 0.0f);
        }
        return;
    }

    line = LineSensor_Update();
    if (line_is_normal_edge(line) != 0U) {
        if (g_edgeArmCount < EDGE_ARM_COUNT) {
            g_edgeArmCount++;
        }
        if (g_edgeArmCount >= EDGE_ARM_COUNT) {
            g_edgeReadyForCorner = 1U;
        }
    } else if (g_edgeReadyForCorner == 0U) {
        g_edgeArmCount = 0U;
    }
    g_lineRecoveryCount = g_edgeArmCount;

    detected_dir = detect_corner_direction(line.mask);
    g_cornerDetectedDir = detected_dir;
    g_cornerConfirmRequired =
        ((line.mask & LINE_MASK_CENTER) != 0U) ?
        CORNER_CONFIRM_CENTER_COUNT : CORNER_CONFIRM_SIDE_COUNT;

    if (CORNER_DETECTION_ENABLE != 0U &&
        g_edgeReadyForCorner != 0U &&
        g_cornerLockoutCount == 0U && detected_dir != 0) {
        if (detected_dir == g_cornerCandidateDir) {
            if (g_cornerStableCount < CORNER_CONFIRM_SIDE_COUNT) {
                g_cornerStableCount++;
            }
        } else {
            g_cornerCandidateDir = detected_dir;
            g_cornerStableCount = 1U;
        }
    } else {
        g_cornerCandidateDir = 0;
        g_cornerStableCount = 0U;
    }

    if (g_cornerStableCount >= g_cornerConfirmRequired) {
        g_pendingTurnDir = g_cornerCandidateDir;
        g_pendingCornerMask = line.mask;
        g_cornerTurnDelayCount = CORNER_TURN_DELAY_COUNT;
        g_cornerStableCount = 0U;
        g_cornerCandidateDir = 0;
        g_lineCorrectionPwm = 0.0f;
        g_trackTurnPwm = 0.0f;
        CarChassis_SetOpenLoopPWM(EDGE_BASE_PWM, 0.0f);
        return;
    }

    /* 直边完全由七路红外纠偏，MPU只保留给直角转弯使用。 */
    line_pwm = line_control(line);
    correction_pwm = LINE_STEERING_SIGN * line_pwm;
    correction_pwm = PID_Clamp(correction_pwm,
        -LINE_CORRECTION_LIMIT, LINE_CORRECTION_LIMIT);
    drive_base_pwm = EDGE_BASE_PWM;

    requested_recovery_direction = 0;
    if (line.seen != 0U &&
        abs_value((float) line.error) >= LINE_SEVERE_ERROR) {
        requested_recovery_direction =
            (line.error > 0) ? -1 : 1;
    } else if (line.mask == LINE_MASK_ALL_WHITE &&
        g_lastLineError != 0) {
        requested_recovery_direction =
            (g_lastLineError > 0) ? -1 : 1;
    }

    if (requested_recovery_direction != 0 &&
        (g_lineSevereRecovery == 0U ||
         requested_recovery_direction != g_severeRecoveryDirection)) {
        g_lineSevereRecovery = 1U;
        g_severeRecoveryDirection = requested_recovery_direction;
        g_severeRecoveryExitCount = 0U;
    }

    if (g_lineSevereRecovery != 0U) {
        if (line.seen != 0U &&
            abs_value((float) line.error) <= LINE_SEVERE_EXIT_ERROR) {
            if (g_severeRecoveryExitCount < LINE_SEVERE_EXIT_COUNT) {
                g_severeRecoveryExitCount++;
            }
            severe_pwm = LINE_SEVERE_CENTER_PWM;
        } else {
            g_severeRecoveryExitCount = 0U;
            severe_pwm = LINE_CORRECTION_LIMIT;
        }

        if (g_severeRecoveryExitCount >= LINE_SEVERE_EXIT_COUNT) {
            g_lineSevereRecovery = 0U;
            g_severeRecoveryDirection = 0;
            g_severeRecoveryExitCount = 0U;
        } else {
            /*
             * 严重恢复一旦触发就持续输出，不受单次探头跳变影响。
             * 若冲到黑线另一侧，请求方向会在上方立即反转。
             */
            correction_pwm = LINE_STEERING_SIGN *
                (float) g_severeRecoveryDirection * severe_pwm;
            drive_base_pwm = LINE_SEVERE_BASE_PWM;
        }
    } else if (line.mask == LINE_MASK_ALL_WHITE ||
        abs_value((float) line.error) >= LINE_SEVERE_ERROR) {
        /*
         * 严重偏线或丢线时降低前进基准并使用大差速，
         * 让内侧轮停下、外侧轮继续转，优先把传感器拉回黑线。
         */
        drive_base_pwm = LINE_SEVERE_BASE_PWM;
    }
    g_lineCorrectionPwm = correction_pwm;
    g_yawCorrectionPwm = 0.0f;
    g_lineYawWeight = 0.0f;
    g_trackTurnPwm = correction_pwm;
    CarChassis_SetOpenLoopPWM(drive_base_pwm, correction_pwm);
}
