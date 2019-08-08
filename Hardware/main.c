#include <ch.h>
#include <hal.h>
#include <communication.h>
#include <positionFB.h>
#include <chprintf.h>

#include <motorControl.h>
/* For module test.
Obtaining speed and angle values and sending them is processed.
*/



int main(void)
{
    chSysInit();
    halInit();

    static const SerialConfig sd_st_cfg = {
        .speed = 115200,
        .cr1 = 0,
        .cr2 = 0,
        .cr3 = 0};

    sdStart(&SD3, &sd_st_cfg);
    palSetPadMode(GPIOD, 8, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOD, 9, PAL_MODE_ALTERNATE(7));

    // debug_stream = (BaseSequentialStream *)&SD3;
    // comm_chn = (BaseChannel *)&SD3;


 
    int32_t value_motor_first = 0;

    initMotorPWM();
    initADC();

    while ( true )
    {   
        palToggleLine(LINE_LED1);
        chprintf(&SD3, "W: %d\n", getPositionFirstServo());
        msg_t msg  = sdGetTimeout(&SD3,MS2ST(100));
        if(msg == MSG_TIMEOUT)
        {
            continue;
        }
        
        char val = msg;

        switch (val)
        {
            case 'q':
                value_motor_first += 5;
                break;
                // palToggleLine(LINE_LED1);

            case 'w':
                value_motor_first -= 5;
                
                break;

            case 'e':
                value_motor_first = 0;
                break;

        }

        
        
        turnFirstMotor(value_motor_first);

        
    }
}
