#include <motorControl.h>
#include <hal.h>

#define MIN_VALUE_MOTOR     -100
#define MAX_VALUE_MOTOR     100
#define COEF_PERIOD         200

#define FIRST_MOTOR_IDX   2
#define SECOND_MOTOR_IDX  3


PWMConfig pwmConf = {
    .frequency      = 1000000,  // 1MHz
    .period         = 20000,    // 20ms ~ 50Hz
    .callback       = NULL,
    .channels       = {
                          {.mode = PWM_OUTPUT_DISABLED,    .callback = NULL},
                          {.mode = PWM_OUTPUT_DISABLED,    .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL}
                      },
    .cr2            = 0,
    .dier           = 0
};

PWMDriver *motor_driver_control = &PWMD2; 


// PWMDriver *pwm_hrz_dr = &PWMD2;
// PWMDriver *pwm_vrt_dr = &PWMD3;

void initMotorPWM( void )
{
    palSetPadMode( GPIOB, 10, PAL_MODE_ALTERNATE(1));
    palSetPadMode( GPIOB, 11, PAL_MODE_ALTERNATE(1));

#define FIRST_MOTOR_A   PAL_LINE(GPIOE, 12)
#define FIRST_MOTOR_B   PAL_LINE(GPIOE, 10)
#define SECOND_MOTOR_A  PAL_LINE(GPIOE, 15)
#define SECOND_MOTOR_B  PAL_LINE(GPIOE, 14)

    palSetLineMode( FIRST_MOTOR_A, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode( FIRST_MOTOR_B, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode( SECOND_MOTOR_A, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode( SECOND_MOTOR_B, PAL_MODE_OUTPUT_PUSHPULL);
    
    pwmStart( motor_driver_control, &pwmConf );

    pwmDisableChannel( motor_driver_control, FIRST_MOTOR_IDX );
    pwmDisableChannel( motor_driver_control, SECOND_MOTOR_IDX );
}

void turnFirstMotor(float value_first_motor){

    value_first_motor = CLIP_VALUE(value_first_motor, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
    value_first_motor = value_first_motor * COEF_PERIOD;
    
    if(value_first_motor > 0)
    {
        palSetLine(FIRST_MOTOR_A);
        palClearLine(FIRST_MOTOR_B);

        pwmEnableChannel( motor_driver_control, FIRST_MOTOR_IDX, value_first_motor );
    }
    else if (value_first_motor < 0)
    {
        value_first_motor = value_first_motor * (-1);

        palSetLine(FIRST_MOTOR_B);
        palClearLine(FIRST_MOTOR_A);

        pwmEnableChannel( motor_driver_control, FIRST_MOTOR_IDX, value_first_motor );
    }
    else
    {
        palClearLine(FIRST_MOTOR_A);
        palClearLine(FIRST_MOTOR_B);

        pwmDisableChannel( motor_driver_control, FIRST_MOTOR_IDX );
    }
}


void turnSecondMotor(float value_second_motor)
{
    value_second_motor = CLIP_VALUE(value_second_motor, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
    value_second_motor = value_second_motor * COEF_PERIOD;
    
    if(value_second_motor > 0)
    {
        palSetLine(SECOND_MOTOR_A);
        palClearLine(SECOND_MOTOR_B);

        pwmEnableChannel( motor_driver_control, SECOND_MOTOR_IDX, value_second_motor );
    }
    else if (value_second_motor < 0)
    {
        value_second_motor = value_second_motor * (-1);

        palSetLine(SECOND_MOTOR_B);
        palClearLine(SECOND_MOTOR_A);

        pwmEnableChannel( motor_driver_control, SECOND_MOTOR_IDX, value_second_motor );
    }
    else
    {
        palClearLine(SECOND_MOTOR_A);
        palClearLine(SECOND_MOTOR_B);

        pwmDisableChannel( motor_driver_control, SECOND_MOTOR_IDX );
    }
}


