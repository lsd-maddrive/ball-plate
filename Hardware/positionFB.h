#ifndef _POSITIONFB_
#define _POSITIONFB_


#include <stdint.h>

#define CLIP_VALUE(x, min, max) ( (x) < (min) ? (min) :     \
                                  (x) > (max) ? (max) : (x) )

void initADC(void);


float getPositionSecondServo(void);
float getPositionFirstServo(void);

int32_t getRawPositionSecondServo(void);
int32_t getRawPositionFirstServo(void);




#endif //_POSITIONFB_
