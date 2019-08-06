typedef struct {

    void (*on_set)(uint8_t speed, uint8_t angle);
    void (*on_start)(void);
    void (*on_stop)(void);
    

} structEventFun_t;


structEventFun_t cpStructWithFunc;

structEventFun_t getDefaultCfg(void)
{
    return structEventFun_t structFuncNull = {NULL, NULL, NULL};
}


void comm_init(structEventFun_t structWithFunc)
{
    cpStructWithFunc.on_start = structWithFunc.on_start;
    cpStructWithFunc.on_stop = structWithFunc.on_stop;
    cpStructWithFunc.on_set = structWithFunc.on_set;
    
}


// in main.c

    structEventFun_t structForFunc = {NULL, NULL, NULL};
 
    structForFunc.on_set = &fucnt_on_set;
    structForFunc.on_start =&funct_on_start;
    structForFunc.on_stop =&funct_on_stop;