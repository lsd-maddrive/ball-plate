#ifndef _POSITIONFB_
#define _POSITIONFB_

#include <stdint.h>
#include <common.h>

void positionFB_init(void);
float positionFB_getValue(uint32_t idx);
int32_t positionFB_getRawValue(uint32_t idx);

#endif //_POSITIONFB_
