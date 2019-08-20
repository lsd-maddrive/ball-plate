#include <positionFB.h>
#include <hal.h>

#define TIMER_EXT_TRIGGER_SRC   (0b0100)
#define TIMER_EXT_TRIGGER       (&GPTD5)

#define ADC1_NUM_CHANNELS   2
#define ADC1_BUF_DEPTH      50

/*
    108/4 = 27 MHz - ADC Frequency
    (144 samples + 12 cycles) * 2 channels -> 86.53 kHz

    100 uS - Timer period -> 10 kHz
 */

#define ADC_LOWER_LIMIT     100.
#define ADC_UPPER_LIMIT     4000.

#define MIN_VALUE_PERCENT_ADC   -100.
#define MAX_VALUE_PERCENT_ADC   100.

#if defined(__GNUC__)
__attribute__((aligned (32)))
#endif
static adcsample_t samples1[ADC1_NUM_CHANNELS * ADC1_BUF_DEPTH];

int32_t average_value_of_the_first_ADC;
int32_t average_value_of_the_second_ADC;

float coefficient_b_for_line = 0;
float coefficient_k_for_line = 0;

static const GPTConfig gptcfg = {
    .frequency =  1000000,
    .callback  =  NULL,
    .cr2       =  TIM_CR2_MMS_1,  
    .dier      =  0U
};

/* n - number of buffer rows */
static void adccallback_1_ADC(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    average_value_of_the_first_ADC = 0;
    average_value_of_the_second_ADC = 0;

    size_t n_channels = adcp->grpp->num_channels;
    size_t n_samples = n * n_channels;

    dmaBufferInvalidate(buffer, n_samples * sizeof (adcsample_t));

    for(size_t i = 0; i < n_samples; i += n_channels)
    {
        average_value_of_the_first_ADC += buffer[i];
        average_value_of_the_second_ADC += buffer[i+1];
    }

    average_value_of_the_first_ADC /= n;
    average_value_of_the_second_ADC /= n;

    adcp=adcp;
    n=n;
}

static const ADCConversionGroup adcgrpcfg1 = {
    .circular     = true,                                          
    .num_channels = ADC1_NUM_CHANNELS,
    .end_cb       = adccallback_1_ADC,
    .error_cb     = NULL,
    .cr1          = 0,             
    .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(TIMER_EXT_TRIGGER_SRC),  
    .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),             
    .smpr2        = ADC_SMPR2_SMP_AN3(ADC_SAMPLE_144),      
    .sqr1         = ADC_SQR1_NUM_CH(ADC1_NUM_CHANNELS),
    .sqr2         = 0,
    .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) |
                    ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10)
};

void positionFB_init(void)
{
    gptStart(TIMER_EXT_TRIGGER, &gptcfg);

    // ADC driver
    adcStart(&ADCD1, NULL);

    palSetLineMode( LINE_ADC123_IN3, PAL_MODE_INPUT_ANALOG );
    palSetLineMode( LINE_ADC123_IN10, PAL_MODE_INPUT_ANALOG );
    
    adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC1_BUF_DEPTH);

    gptStartContinuous(TIMER_EXT_TRIGGER, gptcfg.frequency/10000);

    coefficient_b_for_line = (((-1)*ADC_LOWER_LIMIT*(MAX_VALUE_PERCENT_ADC - MIN_VALUE_PERCENT_ADC))/
                                (ADC_UPPER_LIMIT - ADC_LOWER_LIMIT)) + MIN_VALUE_PERCENT_ADC;

    coefficient_k_for_line = (MAX_VALUE_PERCENT_ADC - MIN_VALUE_PERCENT_ADC) / (ADC_UPPER_LIMIT - ADC_LOWER_LIMIT);
}

float positionFB_getValue(uint32_t idx)
{
    if (idx >= SERVO_COUNT)
        return 0;

    float result = 0;

    if (idx == 0)
    {
        result = average_value_of_the_first_ADC;
    } else if (idx == 1)
    {
        result = average_value_of_the_second_ADC;
    }

    result = CLIP_VALUE(result, ADC_LOWER_LIMIT, ADC_UPPER_LIMIT);
    return result * coefficient_k_for_line + coefficient_b_for_line;
}

int32_t positionFB_getRawValue(uint32_t idx)
{
    if (idx >= SERVO_COUNT)
        return 0;

    if (idx == 0)
    {
        return average_value_of_the_first_ADC;
    } else if (idx == 1)
    {
        return average_value_of_the_second_ADC;
    }

    return 0;
}
