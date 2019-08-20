#include <motorControl.h>
#include <positionFB.h>

#include <stdlib.h>
#include <string.h>

#include "control_pid.h"

#define MAX_REFERENCE_VALUE     40
#define MIN_REFERENCE_VALUE     -MAX_REFERENCE_VALUE

bool flag_control_system_thread = false;

typedef struct
{
    uint32_t    idx;
    float       reference_val;
    float       deadzone;

    pid_ctx_t   pid_ctx;
    float       offset;
} cs_ctx_t;

cs_ctx_t ctxs[SERVO_COUNT];

static THD_WORKING_AREA(waCS_thread_0, 1024);
static THD_WORKING_AREA(waCS_thread_1, 1024);
static THD_FUNCTION(CS_thread, arg) 
{
    cs_ctx_t *ctx = arg;
    
    systime_t time = chVTGetSystemTimeX();

    while (true)
    {
        time += MS2ST(5);
        if(flag_control_system_thread)
        {
            ctx->pid_ctx.error = (ctx->reference_val + ctx->offset) 
                                    - positionFB_getValue(ctx->idx);

            if ( abs(ctx->pid_ctx.error) < ctx->deadzone )
                ctx->pid_ctx.error = 0;

            float servo_control = PID_getControl(&ctx->pid_ctx);
            motors_setPower(ctx->idx, servo_control);
        }
        else
        {
            PID_reset(&ctx->pid_ctx);
            motors_setPower(ctx->idx, 0);
        }
        
        chThdSleepUntil(time);
    }
}

void servoCS_setOffset( uint32_t idx, float offset )
{
    if (idx >= SERVO_COUNT)
        return;

    ctxs[idx].offset = offset;
}

void servoCS_init(void)
{
    positionFB_init();
    motors_init();
    
    for (size_t i = 0; i < SERVO_COUNT; i++)
    {
        cs_ctx_t  *cs_ctx   = &ctxs[i];
        pid_ctx_t *pid_ctx  = &cs_ctx->pid_ctx;
        PID_init(pid_ctx);

        memset( cs_ctx, 0, sizeof(*cs_ctx) );
        cs_ctx->idx = i;

        /* ALL SETUP IS HERE */
        cs_ctx->deadzone = 1;

        pid_ctx->p_rate = 2;
        pid_ctx->i_rate = 0.005;
        pid_ctx->d_rate = 20;
        pid_ctx->integr_limit = 100;
    }

    ctxs[0].offset = 0;
    ctxs[1].offset = 0;

    chThdCreateStatic(waCS_thread_0, 
                        sizeof(waCS_thread_0), 
                        NORMALPRIO+2, 
                        CS_thread, 
                        &ctxs[0]);

    chThdCreateStatic(waCS_thread_1, 
                        sizeof(waCS_thread_1), 
                        NORMALPRIO+2, 
                        CS_thread, 
                        &ctxs[1]);
}

void servoCS_setReference(uint32_t idx, float ref)
{
    if ( idx >= SERVO_COUNT )
        return;

    ref = CLIP_VALUE(ref, MIN_REFERENCE_VALUE, MAX_REFERENCE_VALUE);

    ctxs[idx].reference_val = ref;
}

void servoCS_enable(void)
{
    flag_control_system_thread = true;
}

void servoCS_disable(void)
{
    flag_control_system_thread = false;
}



