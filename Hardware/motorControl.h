#ifndef _MOTORCONTROL_
#define _MOTORCONTROL_


#include <stdint.h>


#define CLIP_VALUE(x, min, max) ( (x) < (min) ? (min) :     \
                                  (x) > (max) ? (max) : (x) )


void initMotorPWM( void );

void turnFirstMotor(float value_first_motor);
void turnSecondMotor(float value_second_motor);







#endif //_COMMUNICATION_
