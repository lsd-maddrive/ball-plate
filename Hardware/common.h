#ifndef COMMON_H_
#define COMMON_H_

#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#define CLIP_VALUE(x, min, max) ( (x) < (min) ? (min) :     \
                                  (x) > (max) ? (max) : (x) )
#define SERVO_COUNT 2

extern BaseSequentialStream    *debug_stream;

#endif // COMMON_H_
