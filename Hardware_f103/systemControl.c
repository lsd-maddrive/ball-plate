#include <motorControl.h>
#include <positionFB.h>
#include <ch.h>

#include <string.h>

bool flag_control_system_thread = false;

typedef struct 
{
    float   p_rate,
            i_rate,
            d_rate;

    float   error,
            prev_error;

    float   integr_sum;

} pid_ctx_t;

typedef struct
{
    uint32_t    idx;
    float       reference_val;
    pid_ctx_t   pid_ctx;
} cs_ctx_t;


static float PID_getControl(pid_ctx_t *ctx)
{
    float control = 0;

    control += ctx->error * ctx->p_rate;
    ctx->integr_sum += ctx->error * ctx->i_rate;
    /* Here we have to limit maximum sum of I part */
    control += ctx->integr_sum;
    control += (ctx->error - ctx->prev_error) * ctx->d_rate;
    ctx->prev_error = ctx->error;

    return control;
}

static void PID_reset(pid_ctx_t *ctx)
{
    ctx->prev_error = 0;
    ctx->integr_sum = 0;
}

cs_ctx_t ctxs[SERVO_COUNT];

static THD_WORKING_AREA(waCS_thread_0, 1024);
static THD_WORKING_AREA(waCS_thread_1, 1024);
static THD_FUNCTION(CS_thread, arg) 
{
    cs_ctx_t *ctx = arg;
    
    systime_t time = chVTGetSystemTimeX();

    while (true)
    {
        time += MS2ST(20);
        if(flag_control_system_thread)
        {
            ctx->pid_ctx.error = ctx->reference_val - positionFB_getValue(ctx->idx);

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


void servoCS_init(void)
{
    positionFB_init();
    motors_init();
    
    for (size_t i = 0; i < SERVO_COUNT; i++)
    {
        cs_ctx_t  *cs_ctx   = &ctxs[i];
        memset( cs_ctx, 0, sizeof(*cs_ctx) );
        cs_ctx->idx = i;

        pid_ctx_t *pid_ctx  = &cs_ctx->pid_ctx;
        pid_ctx->p_rate = 1.3;
        pid_ctx->i_rate = 0.01;
        pid_ctx->d_rate = 4;
    }

    chThdCreateStatic(waCS_thread_0, 
                        sizeof(waCS_thread_0), 
                        NORMALPRIO+1, 
                        CS_thread, 
                        &ctxs[0]);

    chThdCreateStatic(waCS_thread_1, 
                        sizeof(waCS_thread_1), 
                        NORMALPRIO+1, 
                        CS_thread, 
                        &ctxs[1]);
}

void setTaskFirstServo(float task)
{
    ctxs[0].reference_val = task;
}

void setTaskSecondServo(float task)
{
    ctxs[1].reference_val = task;
}

void servoCS_enable(void)
{
    flag_control_system_thread = true;
}

void servoCS_disable(void)
{
    flag_control_system_thread = false;
}



