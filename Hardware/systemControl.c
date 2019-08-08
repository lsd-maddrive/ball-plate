#include <motorControl.h>
#include <positionFB.h>
#include <motorControl.h>
#include <hal.h>

#include <string.h>

float previous_value_error = 0;

float the_task_PID_first_serv = 0;
float the_task_PID_second_serv = 0;

float P_coefficient_first_serv = 0.3;
float I_coefficient_first_serv = 0.01;
float D_coefficient_first_serv = 0;

float P_coefficient_second_serv = 0.3;
float I_coefficient_second_serv = 0.01;
float D_coefficient_second_serv = 0;

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

float PID_getControl(pid_ctx_t *ctx)
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

void PID_reset(pid_ctx_t *ctx)
{
    ctx->prev_error = 0;
    ctx->integr_sum = 0;
}

static THD_WORKING_AREA(waPID_action_serv_n, 1024);
static THD_FUNCTION(PID_action_serv_n, arg) 
{
    pid_ctx_t first_servo_ctx;
    pid_ctx_t second_servo_ctx;

    memset( &first_servo_ctx, 0, sizeof(first_servo_ctx) );
    memset( &second_servo_ctx, 0, sizeof(second_servo_ctx) );

    first_servo_ctx.p_rate = 1.3;
    first_servo_ctx.i_rate = 0.01;
    first_servo_ctx.d_rate = 4;
    
    second_servo_ctx.p_rate = first_servo_ctx.p_rate;
    second_servo_ctx.i_rate = first_servo_ctx.i_rate;
    second_servo_ctx.d_rate = first_servo_ctx.d_rate;

    arg = arg;
    
    systime_t time = chVTGetSystemTimeX();

    while (true)
    {
        time += MS2ST(20);
        if(flag_control_system_thread)
        {
            first_servo_ctx.error = the_task_PID_first_serv - getPositionFirstServo();
            second_servo_ctx.error = the_task_PID_second_serv - getPositionSecondServo();

            float first_servo_control = PID_getControl(&first_servo_ctx);
            float second_servo_control = PID_getControl(&second_servo_ctx);

            turnFirstMotor(first_servo_control);
            turnSecondMotor(second_servo_control);
        }
        else
        {
            PID_reset(&first_servo_ctx);
            PID_reset(&second_servo_ctx);

            turnFirstMotor(0);
            turnSecondMotor(0);
        }
        
        chThdSleepUntil(time);
    }
}


void initControlPID(void)
{
    initADC();
    initMotorPWM();
    
    chThdCreateStatic(waPID_action_serv_n, 
                        sizeof(waPID_action_serv_n), 
                        NORMALPRIO+1, 
                        PID_action_serv_n, 
                        NULL /* arg is NULL */);
}

void setTaskFirstServo(float task)
{
    the_task_PID_first_serv = task;
}

void setTaskSecondServo(float task)
{
    the_task_PID_second_serv = task;
}

void enableThreadControl(void)
{
    flag_control_system_thread = true;
}

void disableThreadControl(void)
{
    flag_control_system_thread = false;
}



