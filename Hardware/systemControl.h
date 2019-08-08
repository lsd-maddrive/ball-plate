#ifndef _SYSTEMCONTROL_
#define _SYSTEMCONTROL_


#include <stdint.h>

#define CLIP_VALUE(x, min, max) ( (x) < (min) ? (min) :     \
                                  (x) > (max) ? (max) : (x) )



void initControlPID(void);

void setTaskFirstServo(float task);
void setTaskSecondServo(float task);

void enableThreadControl(void);
void disableThreadControl(void);



#endif //_SYSTEMCONTROL_
