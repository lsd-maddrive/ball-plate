#ifndef _SYSTEMCONTROL_
#define _SYSTEMCONTROL_

#include <stdint.h>
#include <common.h>

void servoCS_init(void);
void servoCS_setReference(uint32_t idx, float ref);
void servoCS_setOffset( uint32_t idx, float offset );

void servoCS_enable(void);
void servoCS_disable(void);


#endif //_SYSTEMCONTROL_
