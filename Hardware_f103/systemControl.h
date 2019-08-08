#ifndef _SYSTEMCONTROL_
#define _SYSTEMCONTROL_

#include <stdint.h>
#include <common.h>

void servoCS_init(void);

void setTaskFirstServo(float task);
void setTaskSecondServo(float task);

void servoCS_enable(void);
void servoCS_disable(void);


#endif //_SYSTEMCONTROL_
