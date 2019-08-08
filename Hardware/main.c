#include <ch.h>
#include <hal.h>
#include <communication.h>
#include <positionFB.h>
#include <chprintf.h>
#include <systemControl.h>
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


    float task = 0;
    int32_t value_motor_first = 0;

#define TEST_CONTROL

#ifdef TEST_CONTROL
    initControlPID();
#endif

#ifdef TEST_SPEED
    initMotorPWM();
    initADC();
#endif

    while ( true )
    {   
        palToggleLine(LINE_LED1);

        chprintf(&SD3, "W: %d, %d\n", 
                        (int)(getPositionFirstServo()*10), 
                        (int)(getPositionSecondServo()*10));
        
        msg_t msg  = sdGetTimeout(&SD3,MS2ST(100));
        if(msg == MSG_TIMEOUT)
        {
            continue;
        }
        
        char val = msg;

        switch (val)
        {
#ifdef TEST_CONTROL
            case 'a':
                enableThreadControl();
                break;               

            case 's':
                disableThreadControl();
                break;

            case 'q':
                task += 5;
                setTaskFirstServo(task);
                setTaskSecondServo(task);
                chprintf(&SD3, "Set %d\n", (int)task);
                break;

            case 'w':
                task -= 5;
                setTaskFirstServo(task);
                setTaskSecondServo(task);
                chprintf(&SD3, "Set %d\n", (int)task);
                break;

            case ' ':
                task = 0;
                setTaskFirstServo(task);
                setTaskSecondServo(task);
                chprintf(&SD3, "Reset\n");
                break;
#endif

#ifdef TEST_SPEED
            case 'z':
                task += 5;
                turnFirstMotor( task);
                turnSecondMotor(task);
                chprintf(&SD3, "Set %d\n", (int)task);
                break;

            case 'x':
                task -= 5;
                turnFirstMotor(task);
                turnSecondMotor(task);
                chprintf(&SD3, "Set %d\n", (int)task);
                break;

            
            case 'c':
                task = 0;
                turnFirstMotor( task);
                turnSecondMotor(task);
                chprintf(&SD3, "Reset\n");
                break;
#endif

        }     
    }
}
