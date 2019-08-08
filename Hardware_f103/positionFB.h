#ifndef _POSITIONFB_
#define _POSITIONFB_

#include <stdint.h>
#include <common.h>

void positionFB_init(void);
int32_t positionFB_getValue(uint32_t idx);

int32_t getRawPositionSecondServo(void);
int32_t getRawPositionFirstServo(void);

#endif //_POSITIONFB_
