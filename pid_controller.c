#include "pid_controller.h"

float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
    float integral_limit, float min_output, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = integral_limit;
    pid->min_output = min_output;
    pid->max_output = max_output;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Calculate(PID_Controller_t *pid, float target, float actual)
{
    float error;
    float integral;
    float output;

    error = target - actual;
    integral = pid->integral + error;
    integral = PID_Clamp(integral, -pid->integral_limit, pid->integral_limit);

    output = pid->kp * error + pid->ki * integral +
             pid->kd * (error - pid->last_error);
    output = PID_Clamp(output, pid->min_output, pid->max_output);

    /*
     * 抗积分饱和：输出没有顶到限幅时正常积分；
     * 已经顶到限幅时，只允许积分向脱离限幅的方向变化。
     */
    if ((output > pid->min_output && output < pid->max_output) ||
        (output >= pid->max_output && error < 0.0f) ||
        (output <= pid->min_output && error > 0.0f)) {
        pid->integral = integral;
    }

    pid->error = error;
    pid->last_error = error;
    pid->output = output;

    return output;
}
