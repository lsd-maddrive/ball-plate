#include <ch.h>
#include <hal.h>
#include <communication.h>
#include <positionFB.h>
#include <chprintf.h>
/* For module test.
Obtaining speed and angle values and sending them is processed.
*/

typedef struct {
    void (*on_set)(uint8_t speed, uint8_t angle);
    void (*on_start)(void);
    void (*on_stop)(void);
} structEventFun_t;

void funct_on_stop(void){
    palToggleLine(LINE_LED1);
    chThdSleepMilliseconds(200);
}

void funct_on_start(void){
    palToggleLine(LINE_LED2);
    chThdSleepMilliseconds(200);
}

void fucnt_on_set(uint8_t speed, uint8_t street)
{
    palToggleLine(LINE_LED3);
    chThdSleepMilliseconds(200);
}

int main(void)
{
    chSysInit();
    halInit();

    initADC();

    communicationEventFun_t structForFunc = {NULL, NULL, NULL};
 
    structForFunc.on_set = &fucnt_on_set;
    structForFunc.on_start =&funct_on_start;
    structForFunc.on_stop =&funct_on_stop;
    
    comm_init(structForFunc);
    

    while ( true )
    {   
        comm_dbgprintf_info("First Serv %d \n",getPositionFirstServo());
        if (getPositionFirstServo() > 2000)
        {
            palSetLine(LINE_LED1);
        }
        else
        {
            palClearLine(LINE_LED1);
            
        }
       comm_dbgprintf_info("Second Serv %d \n",getPositionSecondServo());
        if (getPositionSecondServo() > 2000)
        {
            palSetLine(LINE_LED2);
        }
        else
        {
            palClearLine(LINE_LED2);
        
        }
        
        // getPositionSecondServo();
        chThdSleepMilliseconds(200);
    }
}
