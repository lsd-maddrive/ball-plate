#ifndef COMMON_H_
#define COMMON_H_

#define CLIP_VALUE(x, min, max) ( (x) < (min) ? (min) :     \
                                  (x) > (max) ? (max) : (x) )
#define SERVO_COUNT 2

#endif // COMMON_H_
