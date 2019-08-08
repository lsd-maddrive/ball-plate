#ifndef _POSITIONFB_
#define _POSITIONFB_


#include <stdint.h>

#define CLIP_VALUE(x, min, max) ( (x) < (min) ? (min) :     \
                                  (x) > (max) ? (max) : (x) )

void initADC(void);


int32_t getPositionSecondServo(void);
int32_t getPositionFirstServo(void);





#endif //_COMMUNICATION_
