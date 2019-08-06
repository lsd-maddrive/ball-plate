#ifndef _COMMUNICATION_
#define _COMMUNICATION_

#include <stdint.h>

#define COMM_MODE_SERIAL_PROG   1
#define COMM_MODE_SERIAL_USB    2

#define COMM_MODE   COMM_MODE_SERIAL_PROG

typedef int8_t comm_speed_t;
typedef int8_t comm_steer_t;

typedef struct {

    void (*on_set)(uint8_t speed, uint8_t angle);
    void (*on_start)(void);
    void (*on_stop)(void);
    

} communicationEventFun_t;

void comm_init(communicationEventFun_t structWithFunc);
void comm_dbgprintf( const char* format, ... );
communicationEventFun_t getDefaultCfg(void);
comm_speed_t comm_get_speed( void );
comm_steer_t comm_get_steer( void );

#endif //_COMMUNICATION_
