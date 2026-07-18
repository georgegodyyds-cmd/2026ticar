#include "track_control.h"

#include "car_chassis.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "pid_controller.h"

/* 直边只使用七路红外，PWM修正先滤波再限速，避免左右摆动。 */
#define EDGE_BASE_PWM                 (620.0f)
#define LINE_LOST_BASE_PWM            (520.0f)
#define REACQUIRE_BASE_PWM            (570.0f)
#define LINE_KP_NEAR                  (0.10f)
#define LINE_KP_FAR                   (0.20f)
#define LINE_KD                       (0.06f)
#define LINE_D_LIMIT                  (45.0f)
#define LINE_NEAR_ERROR               (1200.0f)
#define LINE_ERROR_FILTER_ALPHA       (0.35f)
#define LINE_DEADBAND                 (200.0f)
#define LINE_MEDIUM_ERROR             (400.0f)
#define LINE_LARGE_ERROR              (1500.0f)
#define LINE_SEVERE_ERROR             (2000.0f)
#define LINE_MEDIUM_MIN_PWM           (60.0f)
#define LINE_LARGE_MIN_PWM            (170.0f)
#define LINE_SEVERE_MIN_PWM           (210.0f)
#define EDGE_CORRECTION_LIMIT         (250.0f)
#define REACQUIRE_CORRECTION_LIMIT    (230.0f)
#define LINE_OUTPUT_RISE_STEP         (20.0f)
#define LINE_OUTPUT_RELEASE_STEP      (60.0f)
#define LINE_LOST_SEARCH_PWM          (180.0f)
#define LINE_GAP_HOLD_COUNT           (18U)

/* 只有S4稳定压线后才启用直边角度环，防止MPU锁住偏线位置。 */
#define EDGE_YAW_KP                    (15.0f)
#define EDGE_YAW_LIMIT                 (145.0f)
#define EDGE_YAW_DEADBAND_DEG          (0.5f)
#define EDGE_YAW_ENABLE_COUNT          (8U)

/* 检测到角点后先让轮轴中心到达拐角，再由MPU原地转90度。 */
#define TURN_APPROACH_PWM             (470.0f)
#define TURN_APPROACH_COUNT           (18U)
#define TURN_ANGLE_DEG                (104.0f)
#define TURN_KP                       (12.0f)
#define TURN_OUTPUT_LIMIT             (550.0f)
#define TURN_MIN_PWM                  (380.0f)
#define TURN_FINE_PWM                 (380.0f)
#define TURN_FINE_ZONE_DEG            (8.0f)
#define TURN_DONE_DEG                 (2.0f)
#define TURN_DONE_COUNT               (2U)
#define TURN_MIN_CONTROL_COUNT        (25U)
#define TURN_TIMEOUT_COUNT            (250U)

/* 连续确认用于过滤数字探头的瞬时跳变。 */
#define EDGE_ARM_COUNT                (3U)
#define CORNER_CONFIRM_CENTER_COUNT   (4U)
#define CORNER_CONFIRM_SIDE_COUNT     (5U)
#define CORNER_LOCKOUT_COUNT          (15U)
#define REACQUIRE_CENTER_COUNT        (10U)
#define CENTER_ERROR_LIMIT            (600)

#define LINE_MASK_ALL_WHITE           (0x00U)
#define LINE_MASK_ALL_BLACK           (0x7FU)
#define LINE_MASK_CENTER              (0x08U)
#define TURN_DIR_LEFT                 (1)
#define TURN_DIR_RIGHT                (-1)

typedef enum {
    SQUARE_STATE_EDGE = 0,
    SQUARE_STATE_TURN = 1,
    SQUARE_STATE_REACQUIRE = 2
} SquareState_t;

volatile int16_t g_trackMode = TRACK_MODE_AUTO;
volatile int16_t g_activeTrackMode = TRACK_MODE_STRAIGHT;
volatile float g_trackTurnPwm = 0.0f;
volatile float g_yawTarget = 0.0f;
volatile float g_yawError = 0.0f;
volatile int8_t g_squareTurnDir = 0;
volatile uint8_t g_squareCornerMask = 0U;
volatile uint8_t g_squareControlState = 0U;
volatile float g_lineCorrectionPwm = 0.0f;
volatile float g_yawCorrectionPwm = 0.0f;
volatile uint8_t g_cornerStableCount = 0U;
volatile uint8_t g_turnDoneStableCount = 0U;
volatile uint8_t g_lineRecoveryCount = 0U;
volatile uint8_t g_cornerLockoutCount = 0U;
volatile uint8_t g_allWhiteHoldActive = 0U;
volatile float g_turnStartYaw = 0.0f;
volatile float g_turnRequestedAngle = 0.0f;
volatile uint8_t g_lineCenterStableCount = 0U;
volatile uint8_t g_lineSevereRecovery = 0U;
volatile float g_lineYawWeight = 0.0f;
volatile int8_t g_cornerDetectedDir = 0;
volatile int8_t g_lastCornerDirection = TURN_DIR_LEFT;
volatile uint8_t g_cornerGlobalTrigger = 0U;

static PID_Controller_t g_turnPid;
static PID_Controller_t g_edgeYawPid;
static SquareState_t g_squareState = SQUARE_STATE_EDGE;
static float g_filteredLineError = 0.0f;
static float g_lastFilteredLineError = 0.0f;
static float g_lineOutputPwm = 0.0f;
static int16_t g_lastLineError = 0;
static uint8_t g_lineLostCount = 0U;
static uint8_t g_edgeArmCount = 0U;
static uint8_t g_edgeReadyForCorner = 0U;
static int8_t g_cornerCandidateDir = 0;
static uint8_t g_turnApproachCount = 0U;
static uint16_t g_turnTimeCount = 0U;
static uint8_t g_edgeYawEnabled = 0U;
static uint8_t g_edgeWhiteLatched = 0U;

static float angle_error_deg(float target, float actual)
{
    float error = target - actual;

    while (error > 180.0f) {
        error -= 360.0f;
    }
    while (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

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
    float abs_target = (target >= 0.0f) ? target : -target;
    float abs_output = (g_lineOutputPwm >= 0.0f) ?
        g_lineOutputPwm : -g_lineOutputPwm;

    /* 压回中心或需要反向时快速卸掉旧差速，防止残余PWM把车推过线。 */
    if (abs_target < abs_output ||
        (g_lineOutputPwm > 0.0f && target <= 0.0f) ||
        (g_lineOutputPwm < 0.0f && target >= 0.0f)) {
        step = LINE_OUTPUT_RELEASE_STEP;
    }
    g_lineOutputPwm = approach_value(g_lineOutputPwm, target, step);
    return g_lineOutputPwm;
}

static float edge_yaw_control(void)
{
    float output;

    g_yawError = angle_error_deg(g_yawTarget, g_yawAngle);
    if (g_yawError < EDGE_YAW_DEADBAND_DEG &&
        g_yawError > -EDGE_YAW_DEADBAND_DEG) {
        PID_Reset(&g_edgeYawPid);
        return 0.0f;
    }
    output = PID_Calculate(&g_edgeYawPid, 0.0f, -g_yawError);
    return PID_Clamp(output, -EDGE_YAW_LIMIT, EDGE_YAW_LIMIT);
}

static uint8_t line_is_normal(uint8_t mask)
{
    return (mask != LINE_MASK_ALL_WHITE &&
            mask != LINE_MASK_ALL_BLACK) ? 1U : 0U;
}

static uint8_t line_is_straight_sample(LineSensor_State_t line)
{
    uint8_t virtual_mask;
    uint8_t i;

    if (line_is_normal(line.mask) == 0U) {
        return 0U;
    }
    if (line.active_num == 1U) {
        return 1U;
    }
    if (line.active_num != 2U) {
        return 0U;
    }

    /* S1~S4独立，S5/S6共用PB13，S7保持最右侧独立。 */
    virtual_mask = line.mask & 0x0FU;
    if ((line.mask & 0x30U) != 0U) {
        virtual_mask |= 0x10U;
    }
    if ((line.mask & 0x40U) != 0U) {
        virtual_mask |= 0x20U;
    }
    for (i = 0U; i < 5U; i++) {
        uint8_t adjacent = (uint8_t) (0x03U << i);
        if ((virtual_mask & adjacent) == adjacent) {
            return 1U;
        }
    }
    return 0U;
}

static int8_t detect_corner_direction(uint8_t mask, int8_t previous_dir)
{
    uint8_t left_corner;
    uint8_t right_corner;

    if (mask == LINE_MASK_ALL_WHITE) {
        return 0;
    }

    /* 左三路或最左两路全黑，直接判定为左直角。 */
    left_corner = (((mask & 0x07U) == 0x07U) ||
        (((mask & 0x03U) == 0x03U) &&
         ((mask & LINE_MASK_CENTER) != 0U))) ? 1U : 0U;

    /* 右三路或最右两路全黑，直接判定为右直角。 */
    right_corner = (((mask & 0x70U) == 0x70U) ||
        (((mask & 0x60U) == 0x60U) &&
         ((mask & LINE_MASK_CENTER) != 0U))) ? 1U : 0U;

    if (left_corner != 0U && right_corner == 0U) {
        return TURN_DIR_LEFT;
    }
    if (right_corner != 0U && left_corner == 0U) {
        return TURN_DIR_RIGHT;
    }
    if (left_corner != 0U && right_corner != 0U) {
        return (previous_dir != 0) ?
            previous_dir : g_lastCornerDirection;
    }
    return 0;
}

static float line_control(LineSensor_State_t line, float output_limit)
{
    float target;
    float abs_error;
    float abs_target;
    float error_delta;
    float derivative;

    if (line.mask == LINE_MASK_ALL_WHITE) {
        if (g_lineLostCount < 255U) {
            g_lineLostCount++;
        }
        /*
         * 黑线落在两个探头间隙时会短暂全白。前120ms不做红外搜索，
         * 快速卸掉旧差速并交给MPU保持原直线角度；持续更久才按丢线找回。
         */
        if (g_lineLostCount <= LINE_GAP_HOLD_COUNT) {
            target = 0.0f;
        } else if (g_lastLineError > (int16_t) LINE_DEADBAND) {
            target = -LINE_LOST_SEARCH_PWM;
        } else if (g_lastLineError < -(int16_t) LINE_DEADBAND) {
            target = LINE_LOST_SEARCH_PWM;
        } else {
            target = 0.0f;
        }
        target = PID_Clamp(target, -output_limit, output_limit);
        g_lineOutputPwm = line_output_approach(target);
        g_lastFilteredLineError = g_filteredLineError;
        g_lineCorrectionPwm = g_lineOutputPwm;
        g_lineSevereRecovery = 0U;
        return g_lineOutputPwm;
    }
    if (line.mask == LINE_MASK_ALL_BLACK) {
        g_lineOutputPwm = line_output_approach(0.0f);
        g_lastFilteredLineError = g_filteredLineError;
        g_lineCorrectionPwm = g_lineOutputPwm;
        return g_lineOutputPwm;
    }

    g_lineLostCount = 0U;
    g_lastLineError = line.error;

    g_filteredLineError += LINE_ERROR_FILTER_ALPHA *
        ((float) line.error - g_filteredLineError);
    error_delta = g_filteredLineError - g_lastFilteredLineError;
    g_lastFilteredLineError = g_filteredLineError;
    derivative = PID_Clamp(-LINE_KD * error_delta,
        -LINE_D_LIMIT, LINE_D_LIMIT);
    abs_error = (g_filteredLineError >= 0.0f) ?
        g_filteredLineError : -g_filteredLineError;

    if (abs_error <= LINE_DEADBAND ||
        (line.mask & LINE_MASK_CENTER) != 0U) {
        target = 0.0f;
    } else if (abs_error < LINE_NEAR_ERROR) {
        /* 中线附近降低增益，避免从S4跳到相邻探头时一把修过头。 */
        target = -LINE_KP_NEAR * g_filteredLineError + derivative;
    } else {
        /* 偏差较大时保留原力度，保证车仍能重新追回黑线。 */
        target = -LINE_KP_FAR * g_filteredLineError + derivative;
    }
    /* D项只负责减速阻尼，不允许把车推向误差更大的方向。 */
    if ((g_filteredLineError > 0.0f && target > 0.0f) ||
        (g_filteredLineError < 0.0f && target < 0.0f)) {
        target = 0.0f;
    }
    abs_target = (target >= 0.0f) ? target : -target;

    if (abs_error >= LINE_SEVERE_ERROR) {
        g_lineSevereRecovery = 2U;
        if (abs_target < LINE_SEVERE_MIN_PWM) {
            target = (target >= 0.0f) ?
                LINE_SEVERE_MIN_PWM : -LINE_SEVERE_MIN_PWM;
        }
    } else if (abs_error >= LINE_LARGE_ERROR) {
        g_lineSevereRecovery = 1U;
        if (abs_target < LINE_LARGE_MIN_PWM) {
            target = (target >= 0.0f) ?
                LINE_LARGE_MIN_PWM : -LINE_LARGE_MIN_PWM;
        }
    } else if (abs_error >= LINE_MEDIUM_ERROR &&
        (line.mask & LINE_MASK_CENTER) == 0U) {
        g_lineSevereRecovery = 0U;
        if (abs_target < LINE_MEDIUM_MIN_PWM) {
            target = (target >= 0.0f) ?
                LINE_MEDIUM_MIN_PWM : -LINE_MEDIUM_MIN_PWM;
        }
    } else {
        g_lineSevereRecovery = 0U;
    }

    target = PID_Clamp(target, -output_limit, output_limit);
    g_lineOutputPwm = line_output_approach(target);
    g_lineCorrectionPwm = g_lineOutputPwm;
    return g_lineOutputPwm;
}

static void start_turn(int8_t direction, uint8_t mask)
{
    g_squareState = SQUARE_STATE_TURN;
    g_squareControlState = (uint8_t) SQUARE_STATE_TURN;
    g_activeTrackMode = TRACK_MODE_SQUARE;
    g_squareTurnDir = direction;
    g_lastCornerDirection = direction;
    g_squareCornerMask = mask;
    g_turnStartYaw = g_yawAngle;
    g_turnRequestedAngle = TURN_ANGLE_DEG * (float) direction;
    g_yawTarget = normalize_angle_deg(
        g_turnStartYaw + g_turnRequestedAngle);
    g_yawError = angle_error_deg(g_yawTarget, g_yawAngle);

    g_turnApproachCount = TURN_APPROACH_COUNT;
    g_turnTimeCount = 0U;
    g_turnDoneStableCount = 0U;
    g_cornerStableCount = 0U;
    g_lineCenterStableCount = 0U;
    g_cornerCandidateDir = 0;
    g_edgeReadyForCorner = 0U;
    g_edgeArmCount = 0U;
    g_lastFilteredLineError = g_filteredLineError;
    g_lineOutputPwm = 0.0f;
    g_lineCorrectionPwm = 0.0f;
    g_edgeYawEnabled = 0U;
    g_edgeWhiteLatched = 0U;
    PID_Reset(&g_edgeYawPid);
    PID_Reset(&g_turnPid);
}

static void finish_turn(void)
{
    g_squareState = SQUARE_STATE_REACQUIRE;
    g_squareControlState = (uint8_t) SQUARE_STATE_REACQUIRE;
    g_activeTrackMode = TRACK_MODE_LINE_CURVE;
    g_squareTurnDir = 0;
    g_squareCornerMask = 0U;
    g_turnDoneStableCount = 0U;
    g_lineCenterStableCount = 0U;
    g_cornerLockoutCount = CORNER_LOCKOUT_COUNT;
    g_filteredLineError = 0.0f;
    g_lastFilteredLineError = 0.0f;
    g_lineOutputPwm = 0.0f;
    g_lastLineError = 0;
    g_lineLostCount = 0U;
    g_edgeYawEnabled = 0U;
    g_edgeWhiteLatched = 0U;
    PID_Reset(&g_edgeYawPid);
    PID_Reset(&g_turnPid);
}

static float turn_control(void)
{
    float output;
    float turn_progress;
    float remaining_angle;
    float magnitude;

    /*
     * 红外决定转向，MPU只测量相对起点已经转过多少度。
     * 这样不依赖陀螺仪Z轴正负方向与左右转定义是否一致。
     */
    turn_progress = angle_error_deg(g_yawAngle, g_turnStartYaw);
    if (turn_progress < 0.0f) {
        turn_progress = -turn_progress;
    }
    remaining_angle = TURN_ANGLE_DEG - turn_progress;
    g_yawError = remaining_angle * (float) g_squareTurnDir;
    if (g_turnTimeCount < 65535U) {
        g_turnTimeCount++;
    }

    /* 达到或跨过目标角后立即恢复前进，不再原地等待超时。 */
    if (g_turnTimeCount >= TURN_MIN_CONTROL_COUNT &&
        turn_progress >= (TURN_ANGLE_DEG - TURN_DONE_DEG)) {
        finish_turn();
        return 0.0f;
    }

    magnitude = PID_Calculate(&g_turnPid,
        TURN_ANGLE_DEG, turn_progress);
    magnitude = PID_Clamp(magnitude, 0.0f, TURN_OUTPUT_LIMIT);
    if (remaining_angle <= TURN_FINE_ZONE_DEG) {
        magnitude = TURN_FINE_PWM;
    } else if (magnitude < TURN_MIN_PWM) {
        magnitude = TURN_MIN_PWM;
    }
    output = (g_squareTurnDir >= 0) ? magnitude : -magnitude;

    if (g_turnTimeCount >= TURN_TIMEOUT_COUNT) {
        finish_turn();
        return 0.0f;
    }
    return output;
}

static void reset_control(void)
{
    g_activeTrackMode = TRACK_MODE_STRAIGHT;
    g_trackTurnPwm = 0.0f;
    g_yawTarget = g_yawAngle;
    g_yawError = 0.0f;
    g_squareTurnDir = 0;
    g_squareCornerMask = 0U;
    g_squareControlState = (uint8_t) SQUARE_STATE_EDGE;
    g_lineCorrectionPwm = 0.0f;
    g_yawCorrectionPwm = 0.0f;
    g_cornerStableCount = 0U;
    g_turnDoneStableCount = 0U;
    g_lineRecoveryCount = 0U;
    g_cornerLockoutCount = 0U;
    g_allWhiteHoldActive = 0U;
    g_turnStartYaw = 0.0f;
    g_turnRequestedAngle = 0.0f;
    g_lineCenterStableCount = 0U;
    g_lineSevereRecovery = 0U;
    g_lineYawWeight = 0.0f;
    g_cornerDetectedDir = 0;
    g_cornerGlobalTrigger = 0U;

    g_squareState = SQUARE_STATE_EDGE;
    g_filteredLineError = 0.0f;
    g_lastFilteredLineError = 0.0f;
    g_lineOutputPwm = 0.0f;
    g_lastLineError = 0;
    g_lineLostCount = 0U;
    g_edgeArmCount = 0U;
    g_edgeReadyForCorner = 0U;
    g_cornerCandidateDir = 0;
    g_turnApproachCount = 0U;
    g_turnTimeCount = 0U;
    g_edgeYawEnabled = 0U;
    g_edgeWhiteLatched = 0U;
    PID_Reset(&g_edgeYawPid);
    PID_Reset(&g_turnPid);
}

void TrackControl_Init(void)
{
    PID_Init(&g_edgeYawPid, EDGE_YAW_KP, 0.0f, 0.0f, 0.0f,
        -EDGE_YAW_LIMIT, EDGE_YAW_LIMIT);
    PID_Init(&g_turnPid, TURN_KP, 0.0f, 0.0f, 0.0f,
        -TURN_OUTPUT_LIMIT, TURN_OUTPUT_LIMIT);
    g_trackMode = TRACK_MODE_AUTO;
    g_lastCornerDirection = TURN_DIR_LEFT;
    reset_control();
}

void TrackControl_SetMode(TrackMode_t mode)
{
    g_trackMode = (int16_t) mode;
    reset_control();
}

TrackMode_t TrackControl_GetMode(void)
{
    return (TrackMode_t) g_trackMode;
}

void TrackControl_Task10ms(void)
{
    LineSensor_State_t line;
    int8_t detected_dir;
    float base_pwm = EDGE_BASE_PWM;
    float correction_pwm = 0.0f;
    uint8_t pivot_active = 0U;
    uint8_t square_enabled;
    uint8_t corner_confirm_required = CORNER_CONFIRM_SIDE_COUNT;

    MPU6050_Task10ms();
    line = LineSensor_Update();
    square_enabled = (g_trackMode == TRACK_MODE_AUTO ||
        g_trackMode == TRACK_MODE_SQUARE) ? 1U : 0U;
    g_cornerGlobalTrigger = 0U;
    g_allWhiteHoldActive =
        (line.mask == LINE_MASK_ALL_WHITE) ? 1U : 0U;
    g_lineYawWeight = 0.0f;

    if (g_cornerLockoutCount > 0U) {
        g_cornerLockoutCount--;
    }

    if (g_squareState == SQUARE_STATE_EDGE) {
        g_activeTrackMode = TRACK_MODE_STRAIGHT;

        if (line_is_straight_sample(line) != 0U) {
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

        detected_dir = detect_corner_direction(
            line.mask, g_cornerCandidateDir);
        g_cornerDetectedDir = detected_dir;
        /*
         * 中心探头仍压线时更像真实直角，可较快确认；只有侧边黑线时
         * 也可能只是车身摆偏，必须持续60ms才允许进入原地转弯。
         */
        if ((line.mask & LINE_MASK_CENTER) != 0U) {
            corner_confirm_required = CORNER_CONFIRM_CENTER_COUNT;
        }
        if (square_enabled != 0U &&
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

        if (g_cornerStableCount >= corner_confirm_required) {
            start_turn(g_cornerCandidateDir, line.mask);
            g_cornerGlobalTrigger = 1U;
            base_pwm = TURN_APPROACH_PWM;
            correction_pwm = 0.0f;
        } else {
            if (line.mask == LINE_MASK_ALL_WHITE) {
                base_pwm = LINE_LOST_BASE_PWM;
            }
            correction_pwm = line_control(
                line, EDGE_CORRECTION_LIMIT);

            g_yawCorrectionPwm = 0.0f;
            if (line_is_normal(line.mask) != 0U) {
                g_edgeWhiteLatched = 0U;
                if ((line.mask & LINE_MASK_CENTER) != 0U) {
                    if (g_lineCenterStableCount < EDGE_YAW_ENABLE_COUNT) {
                        g_lineCenterStableCount++;
                    }
                    if (g_lineCenterStableCount >= EDGE_YAW_ENABLE_COUNT &&
                        g_edgeYawEnabled == 0U) {
                        g_yawTarget = g_yawAngle;
                        g_yawError = 0.0f;
                        g_edgeYawEnabled = 1U;
                        PID_Reset(&g_edgeYawPid);
                    }
                } else {
                    /*
                     * 偏线时先由红外拉回中央，但保留已经建立的直线角度。
                     * 回到S4后MPU继续追原目标，避免把偏航后的角度重新记为直线。
                     */
                    g_lineCenterStableCount = 0U;
                    PID_Reset(&g_edgeYawPid);
                }
            } else if (line.mask == LINE_MASK_ALL_WHITE) {
                if (g_edgeWhiteLatched == 0U) {
                    /*
                     * 已经在S4建立过直线目标时继续沿用，跨过探头间隙；
                     * 上电后尚未建立目标才以当前角度作为临时直线方向。
                     */
                    if (g_edgeYawEnabled == 0U) {
                        g_yawTarget = g_yawAngle;
                        g_yawError = 0.0f;
                    }
                    g_edgeYawEnabled = 1U;
                    g_edgeWhiteLatched = 1U;
                    PID_Reset(&g_edgeYawPid);
                }
            } else {
                g_lineCenterStableCount = 0U;
                g_edgeYawEnabled = 0U;
                g_edgeWhiteLatched = 0U;
                PID_Reset(&g_edgeYawPid);
            }

            if (g_edgeYawEnabled != 0U &&
                (((line.mask & LINE_MASK_CENTER) != 0U) ||
                 line.mask == LINE_MASK_ALL_WHITE)) {
                g_yawCorrectionPwm = edge_yaw_control();
                correction_pwm = PID_Clamp(
                    correction_pwm + g_yawCorrectionPwm,
                    -EDGE_CORRECTION_LIMIT, EDGE_CORRECTION_LIMIT);
                g_lineYawWeight = 1.0f;
            }
        }
    } else if (g_squareState == SQUARE_STATE_TURN) {
        g_activeTrackMode = TRACK_MODE_SQUARE;
        g_lineCorrectionPwm = 0.0f;
        if (g_turnApproachCount > 0U) {
            g_turnApproachCount--;
            base_pwm = TURN_APPROACH_PWM;
            correction_pwm = 0.0f;
            g_yawCorrectionPwm = 0.0f;
            if (g_turnApproachCount == 0U) {
                /* 靠近拐角后再记录起始角，避免触发瞬间摆动影响转角基准。 */
                g_turnStartYaw = g_yawAngle;
                g_yawTarget = normalize_angle_deg(
                    g_turnStartYaw + g_turnRequestedAngle);
                g_yawError = angle_error_deg(g_yawTarget, g_yawAngle);
                g_turnTimeCount = 0U;
                g_turnDoneStableCount = 0U;
                PID_Reset(&g_turnPid);
            }
        } else {
            correction_pwm = turn_control();
            g_yawCorrectionPwm = correction_pwm;
            pivot_active =
                (g_squareState == SQUARE_STATE_TURN) ? 1U : 0U;
            if (g_squareState == SQUARE_STATE_REACQUIRE) {
                base_pwm = REACQUIRE_BASE_PWM;
                correction_pwm = line_control(
                    line, REACQUIRE_CORRECTION_LIMIT);
                g_yawCorrectionPwm = 0.0f;
            }
        }
    } else {
        int16_t abs_error = (line.error >= 0) ?
            line.error : -line.error;

        g_activeTrackMode = TRACK_MODE_LINE_CURVE;
        base_pwm = REACQUIRE_BASE_PWM;
        correction_pwm = line_control(
            line, REACQUIRE_CORRECTION_LIMIT);
        g_yawCorrectionPwm = 0.0f;
        g_cornerDetectedDir = 0;

        if (line_is_normal(line.mask) != 0U &&
            (line.mask & LINE_MASK_CENTER) != 0U &&
            abs_error <= CENTER_ERROR_LIMIT) {
            if (g_lineCenterStableCount < REACQUIRE_CENTER_COUNT) {
                g_lineCenterStableCount++;
            }
        } else {
            g_lineCenterStableCount = 0U;
        }

        if (g_lineCenterStableCount >= REACQUIRE_CENTER_COUNT) {
            g_squareState = SQUARE_STATE_EDGE;
            g_squareControlState = (uint8_t) SQUARE_STATE_EDGE;
            g_activeTrackMode = TRACK_MODE_STRAIGHT;
            g_lineCenterStableCount = 0U;
            g_edgeArmCount = 0U;
            g_edgeReadyForCorner = 0U;
            g_cornerCandidateDir = 0;
            g_cornerStableCount = 0U;
            g_cornerLockoutCount = CORNER_LOCKOUT_COUNT;
            g_yawTarget = g_yawAngle;
            g_yawError = 0.0f;
            g_edgeYawEnabled = 1U;
            g_edgeWhiteLatched = 0U;
            PID_Reset(&g_edgeYawPid);
        }
    }

    g_trackTurnPwm = correction_pwm;
    if (pivot_active != 0U) {
        CarChassis_SetPivotPWM(correction_pwm);
    } else {
        CarChassis_SetOpenLoopPWM(base_pwm, correction_pwm);
    }
}
