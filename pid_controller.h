#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float output;
    float integral_limit;
    float min_output;
    float max_output;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
    float integral_limit, float min_output, float max_output);
void PID_Reset(PID_Controller_t *pid);
float PID_Calculate(PID_Controller_t *pid, float target, float actual);
float PID_Clamp(float value, float min_value, float max_value);

#endif
