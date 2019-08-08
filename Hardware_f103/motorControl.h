#ifndef _MOTORCONTROL_
#define _MOTORCONTROL_

#include <stdint.h>
#include <common.h>

void motors_init( void );
void motors_setPower( uint32_t motor_idx, float power );

#endif //_COMMUNICATION_
