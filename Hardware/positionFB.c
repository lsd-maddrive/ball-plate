#include <ch.h>
#include <hal.h>

#include <chprintf.h>
#include <positionFB.h>


#define ADC1_NUM_CHANNELS   1
#define ADC1_BUF_DEPTH      8

#define ADC3_NUM_CHANNELS   1
#define ADC3_BUF_DEPTH      8

static adcsample_t samples1[ADC1_NUM_CHANNELS * ADC1_BUF_DEPTH];
static adcsample_t samples3[ADC3_NUM_CHANNELS * ADC3_BUF_DEPTH];

int32_t average_value_of_the_first_ADC;
int32_t average_value_of_the_second_ADC;


// Cons for GPT PA4
static const GPTConfig gpt4cfg1 = {
    .frequency =  100000,
    .callback  =  NULL,
    .cr2       =  TIM_CR2_MMS_1,  
    .dier      =  0U
    
};

// Cons for GPT PF4
static const GPTConfig gpt4cfg3 = {
    .frequency =  100000,
    .callback  =  NULL,
    .cr2       =  TIM_CR2_MMS_1,  
    .dier      =  0U

};


static void adccallback_1_ADC(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    average_value_of_the_first_ADC = 0;
    for(int i = 0; i < n; i++)
    {
        average_value_of_the_first_ADC += buffer[i];
    }

    average_value_of_the_first_ADC = average_value_of_the_first_ADC / n;
    
    // value_buf_1 = buffer[0];
    adcp=adcp;
    n=n;
}

static void adccallback_3_ADC(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    average_value_of_the_second_ADC = 0;
    for(int i = 0; i < n; i++)
    {
        average_value_of_the_second_ADC += buffer[i];
    }

    average_value_of_the_second_ADC = average_value_of_the_second_ADC / n;
    // value_buf_3 = buffer[0];
    adcp=adcp;
    n=n;
}

// function for error ADC. Nothing happens when called.
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
    (void)adcp;
    (void)err;
}

// Const for ADC in PA4
static const ADCConversionGroup adcgrpcfg1 = {
    .circular     = true,                                          
    .num_channels = ADC1_NUM_CHANNELS,
    .end_cb       = adccallback_1_ADC,
    .error_cb     = adcerrorcallback,
    .cr1          = 0,             
    .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b1100),  
    .smpr1        = 0,             
    .smpr2        = ADC_SMPR2_SMP_AN3(ADC_SAMPLE_144),              
    .sqr1         = ADC_SQR1_NUM_CH(ADC1_NUM_CHANNELS),
    .sqr2         = 0,
    .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) 
    
};

// Const for ADC in PF4
static const ADCConversionGroup adcgrpcfg3 = {
    .circular     = true,                                           
    .num_channels = ADC3_NUM_CHANNELS,
    .end_cb       = adccallback_3_ADC,
    .error_cb     = adcerrorcallback,
    .cr1          = 0,             
    .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b1101),  
    .smpr1        = ADC_SMPR1_SMP_AN14(ADC_SAMPLE_144),             
    .smpr2        = 0,              
    .sqr1         = ADC_SQR1_NUM_CH(ADC3_NUM_CHANNELS),
    .sqr2         = 0,
    .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN14)
    
};

void initADC(void)
{
    gptStart(&GPTD4, &gpt4cfg1);
    gptStart(&GPTD6, &gpt4cfg3);

    // ADC driver
    adcStart(&ADCD1, NULL);
    adcStart(&ADCD3, NULL);


    palSetLineMode( LINE_ADC123_IN3, PAL_MODE_INPUT_ANALOG );  // PA3
    palSetLineMode( LINE_ADC3_IN14, PAL_MODE_INPUT_ANALOG );   // PF4
    
    adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC1_BUF_DEPTH);
    adcStartConversion(&ADCD3, &adcgrpcfg3, samples3, ADC3_BUF_DEPTH);

    gptStartContinuous(&GPTD4, gpt4cfg1.frequency/1000);   
    gptStartContinuous(&GPTD6, gpt4cfg3.frequency/1000);         // how often we need ADC value
    /* Just set the limit (interval) of timer counter, you can use this function
       not only for ADC triggering, but start infinite counting of timer for callback processing */
}


int32_t getPositionFirstServo(void)
{
    return average_value_of_the_first_ADC;
}


int32_t getPositionSecondServo(void)
{
    return average_value_of_the_second_ADC;
}




