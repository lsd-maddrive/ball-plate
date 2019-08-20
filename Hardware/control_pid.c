#include <string.h>

#include "control_pid.h"

float PID_getControl(pid_ctx_t *ctx)
{
    float control = 0;

    control += ctx->error * ctx->p_rate;
    ctx->integr_sum += ctx->error * ctx->i_rate;

    if ( ctx->integr_limit != 0 )
    {
        /* Here we have to limit maximum sum of I part */
        if ( ctx->integr_sum > ctx->integr_limit )
            ctx->integr_sum = ctx->integr_limit;
        if ( ctx->integr_sum < -ctx->integr_limit )
            ctx->integr_sum = -ctx->integr_limit;
    }
    
    control += ctx->integr_sum;
    control += (ctx->error - ctx->prev_error) * ctx->d_rate;
    ctx->prev_error = ctx->error;

    return control;
}

void PID_reset(pid_ctx_t *ctx)
{
    ctx->prev_error = 0;
    ctx->integr_sum = 0;
}

void PID_init(pid_ctx_t *ctx)
{
    memset( ctx, 0, sizeof(*ctx) );
}
