#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include "common.h"

typedef struct 
{
    float   p_rate,
            i_rate,
            d_rate;

    float   error,
            prev_error;

    float   integr_sum;
    float   integr_limit;

} pid_ctx_t;

float PID_getControl(pid_ctx_t *ctx);
void PID_reset(pid_ctx_t *ctx);
void PID_init(pid_ctx_t *ctx);

#endif // CONTROL_PID_H_
