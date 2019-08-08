#include <motorControl.h>
#include <hal.h>

PWMDriver *motor_driver_control = &PWMD1; 

#define MIN_VALUE_MOTOR     -100
#define MAX_VALUE_MOTOR     100
#define COEF_PERIOD         200

PWMConfig pwmConf = {
    .frequency      = 1000000,  // 1MHz
    .period         = 20000,    // 20ms ~ 50Hz
    .callback       = NULL,
    .channels       = {
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL},
                          {.mode = PWM_OUTPUT_DISABLED,    .callback = NULL},
                          {.mode = PWM_OUTPUT_DISABLED,    .callback = NULL}
                      },
    .cr2            = 0,
    .dier           = 0
};

typedef struct {
    ioline_t    line_a,
                line_b;
    
    uint32_t    idx;
} motor_ctx_t;

motor_ctx_t     motors[SERVO_COUNT];

void motors_init( void )
{
    palSetPadMode( GPIOA, 8, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    palSetPadMode( GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

    motors[0].idx = 0;
    motors[1].idx = 1;

    motors[0].line_a    = PAL_LINE(GPIOB, 12);
    motors[0].line_b    = PAL_LINE(GPIOB, 13);
    motors[1].line_a    = PAL_LINE(GPIOB, 14);
    motors[1].line_b    = PAL_LINE(GPIOB, 15);

    pwmStart( motor_driver_control, &pwmConf );

    for (size_t i = 0; i < SERVO_COUNT; i++)
    {
        palSetLineMode( motors[i].line_a, PAL_MODE_OUTPUT_PUSHPULL);
        palSetLineMode( motors[i].line_b, PAL_MODE_OUTPUT_PUSHPULL);

        motors_setPower(0, i);
    }
}

void motors_setPower(uint32_t motor_idx, float power)
{
    if (motor_idx >= SERVO_COUNT)
        return;

    motor_ctx_t *ctx = &motors[motor_idx];

    power = CLIP_VALUE(power, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
    power = power * COEF_PERIOD;
    
    if(power > 0)
    {
        palSetLine(ctx->line_a);
        palClearLine(ctx->line_b);

        pwmEnableChannel( motor_driver_control, ctx->idx, power );
    }
    else if (power < 0)
    {
        power = power * (-1);

        palSetLine(ctx->line_b);
        palClearLine(ctx->line_a);

        pwmEnableChannel( motor_driver_control, ctx->idx, power );
    }
    else
    {
        palClearLine(ctx->line_a);
        palClearLine(ctx->line_b);

        pwmDisableChannel( motor_driver_control, ctx->idx );
    }
}

