#include <string.h>

#include "common.h"

#include "positionFB.h"
#include "systemControl.h"
#include "motorControl.h"
/* For module test.
Obtaining speed and angle values and sending them is processed.
*/

#include "control_pid.h"

SerialDriver            *debug_driver = &SD3;
BaseSequentialStream    *debug_stream = (BaseSequentialStream *)&SD3;

#define ADC_PANEL_NUM_CHANNELS   1
#define ADC_PANEL_BUF_DEPTH      20
ADCDriver   *panel_driver = &ADCD2;
ioline_t    panel_line = LINE_ADC123_IN13;

int32_t panel_get_adc_value(void)
{
    int32_t value = 0;

    static adcsample_t samples[ADC_PANEL_NUM_CHANNELS * ADC_PANEL_BUF_DEPTH];

    static const ADCConversionGroup adcgrpcfg1 = {
        .circular     = false,                                          
        .num_channels = ADC_PANEL_NUM_CHANNELS,
        .end_cb       = NULL,
        .error_cb     = NULL,
        .cr1          = 0,             
        .cr2          = ADC_CR2_SWSTART,  
        .smpr1        = ADC_SMPR1_SMP_AN13(ADC_SAMPLE_144),             
        .smpr2        = 0,      
        .sqr1         = ADC_SQR1_NUM_CH(ADC_PANEL_NUM_CHANNELS),
        .sqr2         = 0,
        .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN13)
    };

    msg_t msg = adcConvert( panel_driver, &adcgrpcfg1, samples, ADC_PANEL_BUF_DEPTH );

    if ( msg != MSG_OK )
    {
        // chprintf(debug_stream, "Err: %d\n", msg);
        return 0;
    }

    for ( int i = 0; i < ADC_PANEL_BUF_DEPTH; i++ )
    {
        value += samples[i];
    }

    // chprintf(debug_stream, "Val: %d\n", msg);

    return value / ADC_PANEL_BUF_DEPTH;
}

ioline_t panel_lines[] = {
    PAL_LINE(GPIOD, 6),
    PAL_LINE(GPIOD, 5),
    PAL_LINE(GPIOD, 4),
    PAL_LINE(GPIOD, 3)
};

#define PANEL_ADC_STABILITY_TIME_US         200
#define PANEL_MAX_ADC_VALUE     2900
#define PANEL_MIN_ADC_VALUE     1100

static float PANEL_K_VAL;

int32_t panelRawX = 0;
int32_t panelRawY = 0;
int32_t panelRawInvX = 0;
int32_t panelRawInvY = 0;
bool panelIsPressed = false;

void panel_clearLines(void)
{
    for ( size_t i = 0; i < sizeof(panel_lines) / sizeof(*panel_lines); i++ )
    {
        palClearLine( panel_lines[i] );
    }
}

bool panel_isPressed(void)
{
    panel_clearLines();

    chThdSleepMicroseconds(PANEL_ADC_STABILITY_TIME_US);

    panelIsPressed = panel_get_adc_value() != 0;

    return panelIsPressed;
}

int32_t panel_getRawInvX(void)
{
    /* Mode X */
    palSetLine(panel_lines[0]);
    palSetLine(panel_lines[2]);
    palClearLine(panel_lines[1]);
    palClearLine(panel_lines[3]);

    chThdSleepMicroseconds(PANEL_ADC_STABILITY_TIME_US);

    panelRawInvX = panel_get_adc_value();

    panel_clearLines();

    return panelRawInvX;
}

int32_t panel_getRawX(void)
{
    /* Mode X */
    palSetLine(panel_lines[1]);
    palSetLine(panel_lines[3]);
    palClearLine(panel_lines[0]);
    palClearLine(panel_lines[2]);

    chThdSleepMicroseconds(PANEL_ADC_STABILITY_TIME_US);

    panelRawX = panel_get_adc_value();

    panel_clearLines();

    return panelRawX;
}

int32_t panel_getRawInvY(void)
{
    /* Mode Y */
    palSetLine(panel_lines[2]);
    palSetLine(panel_lines[3]);
    palClearLine(panel_lines[0]);
    palClearLine(panel_lines[1]);

    chThdSleepMicroseconds(PANEL_ADC_STABILITY_TIME_US);
    
    panelRawInvY = panel_get_adc_value();

    panel_clearLines();

    return panelRawInvY;
}

int32_t panel_getRawY(void)
{
    /* Mode Y */
    palSetLine(panel_lines[0]);
    palSetLine(panel_lines[1]);
    palClearLine(panel_lines[2]);
    palClearLine(panel_lines[3]);

    chThdSleepMicroseconds(PANEL_ADC_STABILITY_TIME_US);
    
    panelRawY = panel_get_adc_value();

    panel_clearLines();

    return panelRawY;
}

float panel_getX(void)
{
    int32_t raw = panel_getRawX();

    float result = (raw - PANEL_MIN_ADC_VALUE) * PANEL_K_VAL - 1.0;
    return result;
}

float panel_getY(void)
{
    int32_t raw = panel_getRawY();

    float result = (raw - PANEL_MIN_ADC_VALUE) * PANEL_K_VAL - 1.0;
    return result;
}

void panel_init(void)
{
    adcStart(panel_driver, NULL);
    palSetLineMode(panel_line, PAL_MODE_INPUT_ANALOG);
    for ( size_t i = 0; i < sizeof(panel_lines) / sizeof(*panel_lines); i++ )
    {
        palSetLineMode( panel_lines[i], PAL_MODE_OUTPUT_PUSHPULL );
    }

    PANEL_K_VAL = 2.0/(PANEL_MAX_ADC_VALUE-PANEL_MIN_ADC_VALUE);
}

static THD_WORKING_AREA(waBlinker_thd, 64);
static THD_FUNCTION(Blinker_thd, arg) 
{
    arg = arg;

    while ( 1 )
    {
        palToggleLine(LINE_LED1);
        chThdSleepMilliseconds(1000);
    }
}

float panelX;
float panelY;

float referenceX = 0;
float referenceY = 0;

bool panelSystemEnabled = false;

void panel_setCSEnabled(bool enabled)
{
    panelSystemEnabled = enabled;

    if ( panelSystemEnabled )
        servoCS_enable();
    else
        servoCS_disable();
}

static THD_WORKING_AREA(waPanelControl_thd, 1024);
static THD_FUNCTION(PanelControl_thd, arg) 
{
    arg = arg;

    systime_t time = chVTGetSystemTimeX();

    pid_ctx_t pid_ctx_x;
    pid_ctx_t pid_ctx_y;
    PID_init(&pid_ctx_x);
    PID_init(&pid_ctx_y);

    pid_ctx_x.p_rate = 20;
    pid_ctx_x.i_rate = 0;
    pid_ctx_x.d_rate = 400;

    /* Duplicate content */
    pid_ctx_y = pid_ctx_x;

    while (true)
    {
        time += MS2ST(10);

        panelX = panel_getX();
        panelY = panel_getY();

        panel_getRawInvX();
        panel_getRawInvY();

        pid_ctx_x.error = referenceX - panelX;
        pid_ctx_y.error = referenceY - panelY;

        if ( panel_isPressed() && panelSystemEnabled )
        {
            float controlX = PID_getControl(&pid_ctx_x);
            float controlY = PID_getControl(&pid_ctx_y);

            servoCS_setReference(0, -controlX);
            servoCS_setReference(1, -controlY);
        }
        else
        {   
            PID_reset(&pid_ctx_x);
            PID_reset(&pid_ctx_y);

            servoCS_setReference(0, 0);
            servoCS_setReference(1, 0);
        }

        chThdSleepUntil(time);
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
    palSetPadMode(GPIOD, 8, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOD, 9, PAL_MODE_ALTERNATE(7));

    chThdCreateStatic(waBlinker_thd, 
                        sizeof(waBlinker_thd), 
                        NORMALPRIO-1, 
                        Blinker_thd, 
                        NULL);

    chThdCreateStatic(waPanelControl_thd, 
                        sizeof(waPanelControl_thd), 
                        NORMALPRIO+1, 
                        PanelControl_thd, 
                        NULL);

    chprintf(debug_stream, "Start!\n");

    float task = 0;

#define TEST_CONTROL
// #define TEST_SPEED
#define DEBUG_MAIN

#ifdef TEST_CONTROL
    servoCS_init();

    servoCS_setOffset(0, 2);
    servoCS_setOffset(1, -17.5);
#endif

#ifdef TEST_SPEED
    positionFB_init();
    motors_init();
#endif

    panel_init();

    while ( true )
    {   
        palToggleLine( LINE_LED2 );
#ifdef DEBUG_MAIN
        chprintf(debug_stream, "V: %d, %d --- ", 
                        (int)(positionFB_getValue(0)*10), 
                        (int)(positionFB_getValue(1)*10));
        chprintf(debug_stream, "R: %d, %d --- ", 
                        positionFB_getRawValue(0), 
                        positionFB_getRawValue(1));
        chprintf(debug_stream, "T: (%d, %d) --- ", (int)(panelX * 100), 
                                                   (int)(panelY * 100));
        chprintf(debug_stream, "TR: (%d, %d) %d\n", panelRawX, panelRawY, panelIsPressed);
#endif
        msg_t msg  = sdGetTimeout(debug_driver, MS2ST(100));     
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

            case 'z':
                panel_setCSEnabled(true);
                chprintf(debug_stream, "Panel enabled\n");
                break;               

            case 'x':
                panel_setCSEnabled(false);
                chprintf(debug_stream, "Panel disabled\n");
                break;               

            case 'q':
                task += 5;
                servoCS_setReference(0, task);
                servoCS_setReference(1, task);
                chprintf(debug_stream, "Set %d\n", (int)task);
                break;

            case 'w':
                task -= 5;
                servoCS_setReference(0, task);
                servoCS_setReference(1, task);
                chprintf(debug_stream, "Set %d\n", (int)task);
                break;

            case ' ':
                task = 0;
                servoCS_setReference(0, task);
                servoCS_setReference(1, task);
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
