#include <ch.h>
#include <hal.h>

#include <chprintf.h>

#define ADC1_NUM_CHANNELS   1       // because only 2 sensors for now
#define ADC1_BUF_DEPTH      20

static adcsample_t adc_buffer[ADC1_NUM_CHANNELS * ADC1_BUF_DEPTH];