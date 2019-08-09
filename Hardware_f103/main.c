#include <ch.h>
#include <hal.h>

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

    SerialDriver            *debug_driver = &SD2;
    BaseSequentialStream    *debug_stream = (BaseSequentialStream *)debug_driver;

    sdStart(debug_driver, &sd_st_cfg);
    // palSetPadMode(GPIOA, 2, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    // palSetPadMode(GPIOA, 3, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

    palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);

    float task = 0;

#define TEST_CONTROL

#ifdef TEST_CONTROL
    servoCS_init();
#endif

#ifdef TEST_SPEED
    motors_init();
    initADC();
#endif

    while ( true )
    {   
        palTogglePad(GPIOC, 13);

        chprintf(debug_stream, "V: %d, %d --- ", 
                        (int)(positionFB_getValue(0)*10), 
                        (int)(positionFB_getValue(1)*10));
        chprintf(debug_stream, "R: %d, %d\n", 
                        positionFB_getRawValue(0), 
                        positionFB_getRawValue(1));

        msg_t msg  = sdGetTimeout(debug_driver, MS2ST(100));
        if(msg == MSG_TIMEOUT)
        {
            continue;
        }
        
        char val = msg;

        switch (val)
        {
#ifdef TEST_CONTROL
            case 'a':
                servoCS_enable();
                chprintf(debug_stream, "Enabled\n");
                break;               

            case 's':
                servoCS_disable();
                chprintf(debug_stream, "Disabled\n");
                break;

            case 'q':
                task += 5;
                setTaskFirstServo(task);
                setTaskSecondServo(task);
                chprintf(debug_stream, "Set %d\n", (int)task);
                break;

            case 'w':
                task -= 5;
                setTaskFirstServo(task);
                setTaskSecondServo(task);
                chprintf(debug_stream, "Set %d\n", (int)task);
                break;

            case ' ':
                task = 0;
                setTaskFirstServo(task);
                setTaskSecondServo(task);
                chprintf(debug_stream, "Reset\n");
                break;
#endif

#ifdef TEST_SPEED
            case 'z':
                task += 5;
                motors_setPower(0, task);
                motors_setPower(1, task);
                chprintf(debug_stream, "Set %d\n", (int)task);
                break;

            case 'x':
                task -= 5;
                motors_setPower(0, task);
                motors_setPower(1, task);
                chprintf(debug_stream, "Set %d\n", (int)task);
                break;

            
            case 'c':
                task = 0;
                motors_setPower(0, task);
                motors_setPower(1, task);
                chprintf(debug_stream, "Reset\n");
                break;
#endif

        }     
    }
}
