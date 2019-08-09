#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>

#include "usbcfg.h"

#include "positionFB.h"
#include "systemControl.h"
#include "motorControl.h"
/* For module test.
Obtaining speed and angle values and sending them is processed.
*/

SerialDriver            *debug_driver = &SD2;
BaseSequentialStream    *debug_stream = (BaseSequentialStream *)&SD2;

bool                program_control = false;
virtual_timer_t     prog_watchdog_vt;

static void wdg_prog_cb( void *arg )
{
    arg = arg;  // to avoid warnings

    program_control = false;
    chprintf(debug_stream, "Program control disabled\n");
}

static void check_prog_control( void )
{
    // chVTSet( &prog_watchdog_vt, MS2ST( 500 ), wdg_prog_cb, NULL );
}

static void reset_prog_control( void )
{
    // chVTReset( &prog_watchdog_vt );
}

static THD_WORKING_AREA(waUSB_rcvr, 2048);
static THD_FUNCTION(USB_rcvr, arg)
{
    arg = arg;

    /* USB */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);  

    chVTObjectInit(&prog_watchdog_vt);

    while ( 1 )
    {
        if (SDU1.config->usbp->state == USB_ACTIVE)
        {
            msg_t usb_msg = chnGetTimeout(&SDU1, TIME_IMMEDIATE);
            if ( usb_msg != MSG_TIMEOUT && usb_msg != MSG_RESET )
            {
                char usb_val = usb_msg;
                if ( usb_val == '#' )
                {
                    float buffer[2];
                    size_t result = chnRead(&SDU1, buffer, sizeof(buffer));
                    if ( result > 0 )
                    {
                        setTaskFirstServo(buffer[0]);
                        setTaskSecondServo(buffer[1]);

                        // chprintf(debug_stream, "T: %d %d\n", 
                        //             (int)(buffer[0] * 10),
                        //             (int)(buffer[1] * 10) );

                        check_prog_control();
                    }
                }
                else if ( usb_val == '$' )
                {
                    uint8_t ideal[2] = {167, 253};
                    uint8_t buffer[2];
                    size_t result = chnRead(&SDU1, buffer, sizeof(buffer));
                    if ( result > 0 )
                    {
                        if ( !memcmp(ideal, buffer, sizeof(ideal)) )
                        {   
                            program_control = false;
                            setTaskFirstServo(0);
                            setTaskSecondServo(0);
                            chprintf(debug_stream, "Reset table\n");

                            reset_prog_control();
                        } 
                    }
                }
                else if ( usb_val == '%' )
                {
                    uint8_t ideal[2] = {165, 252};
                    uint8_t buffer[2];
                    size_t result = chnRead(&SDU1, buffer, sizeof(buffer));
                    if ( result > 0 )
                    {
                        if ( !memcmp(ideal, buffer, sizeof(ideal)) )
                        {  
                            program_control = true;
                            servoCS_enable();
                            chprintf(debug_stream, "Access table\n");

                            check_prog_control();
                        } 
                    }
                }
            }
        }

        chThdSleepMilliseconds(2);
    }
}

int main(void)
{
    chSysInit();
    halInit();

    static const SerialConfig sd_st_cfg = {
        .speed = 115200,
        .cr1 = 0,
        .cr2 = 0,
        .cr3 = 0};

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

    chThdCreateStatic(waUSB_rcvr, 
                    sizeof(waUSB_rcvr), 
                    NORMALPRIO+1, 
                    USB_rcvr, 
                    NULL);

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
        char val = msg;

        /* Disable handy control when program has access */
        if ( program_control )
            continue;

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
