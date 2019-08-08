#include <motorControl.h>
#include <hal.h>


static void tilt_unit_set_horizontal_pos( int16_t pos_val );
static void tilt_unit_set_vertical_pos( int16_t pos_val );


#define MIN_VALUE_MOTOR     -100
#define MAX_VALUE_MOTOR     100
#define COEF_PERIOD         200

#define FIRST_MOTOR_RIGHT   0
#define FIRST_MOTOR_LEFT    1
#define SECOND_MOTOR_RIGHT  2
#define SECOND_MOTOR_LEFT   3


PWMConfig pwmConf = {
    .frequency      = 1000000,  // 1MHz
    .period         = 20000,    // 20ms ~ 50Hz
    .callback       = NULL,
    .channels       = {
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL}, //PE_9
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL}, //PE_11
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL}, //PE_13
                          {.mode = PWM_OUTPUT_ACTIVE_HIGH,    .callback = NULL}  //PE_14
                      },
    .cr2            = 0,
    .dier           = 0
};

PWMDriver *motor_driver_control = &PWMD1; 


// PWMDriver *pwm_hrz_dr = &PWMD2;
// PWMDriver *pwm_vrt_dr = &PWMD3;

void initMotorPWM( void )
{

    palSetPadMode( GPIOE, 9,  PAL_MODE_ALTERNATE(1));
    palSetPadMode( GPIOE, 11, PAL_MODE_ALTERNATE(1));
    palSetPadMode( GPIOE, 13, PAL_MODE_ALTERNATE(1));
    palSetPadMode( GPIOE, 14, PAL_MODE_ALTERNATE(1));

    pwmStart( motor_driver_control, &pwmConf );


    pwmDisableChannel( motor_driver_control, FIRST_MOTOR_RIGHT );
    pwmDisableChannel( motor_driver_control, FIRST_MOTOR_LEFT );
    pwmDisableChannel( motor_driver_control, SECOND_MOTOR_RIGHT );
    pwmDisableChannel( motor_driver_control, SECOND_MOTOR_LEFT );

}

void turnFirstMotor(int32_t value_first_motor){

    value_first_motor = CLIP_VALUE(value_first_motor, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);
    value_first_motor = value_first_motor * COEF_PERIOD;
    
    if(value_first_motor > 0)
    {
        pwmEnableChannel( motor_driver_control, FIRST_MOTOR_RIGHT, value_first_motor );
        pwmDisableChannel( motor_driver_control, FIRST_MOTOR_LEFT );
    }

    else if (value_first_motor < 0)
    {

        value_first_motor = value_first_motor * (-1);

        pwmEnableChannel( motor_driver_control, FIRST_MOTOR_LEFT, value_first_motor );
        pwmDisableChannel( motor_driver_control, FIRST_MOTOR_RIGHT );
    }

    else
    {
        pwmDisableChannel( motor_driver_control, FIRST_MOTOR_RIGHT );
        pwmDisableChannel( motor_driver_control, FIRST_MOTOR_LEFT );
    }
    
}


void turnSecondMotor(int32_t value_second_motor){
    value_second_motor = CLIP_VALUE(value_second_motor, MIN_VALUE_MOTOR, MAX_VALUE_MOTOR);

    value_second_motor = value_second_motor * COEF_PERIOD;

    if(value_second_motor > 0)
    {

        pwmEnableChannel( motor_driver_control, SECOND_MOTOR_RIGHT, value_second_motor );
        pwmDisableChannel( motor_driver_control, SECOND_MOTOR_LEFT );

        
    }

    else if (value_second_motor < 0)
    {
        value_second_motor = value_second_motor * (-1);

        pwmEnableChannel( motor_driver_control, SECOND_MOTOR_LEFT, value_second_motor );
        pwmDisableChannel( motor_driver_control, SECOND_MOTOR_RIGHT );
    }

    else
    {
        pwmDisableChannel( motor_driver_control, SECOND_MOTOR_RIGHT );
        pwmDisableChannel( motor_driver_control, SECOND_MOTOR_LEFT );
    }
    
}


