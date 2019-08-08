#include <motorControl.h>
#include <positionFB.h>
#include <motorControl.h>
#include <hal.h>

float previous_value_error = 0;

float the_task_PID_first_serv = 0;
float the_task_PID_second_serv = 0;

float P_coefficient_first_serv = 0.3;
float I_coefficient_first_serv = 0;
float D_coefficient_first_serv = 0;

float P_coefficient_second_serv = 0.3;
float I_coefficient_second_serv = 0;
float D_coefficient_second_serv = 0;


bool flag_control_system_thread = false;



// static THD_WORKING_AREA(waPID_action_first_serv_n, 2048);
// static THD_FUNCTION(PID_action_first_serv_n, arg) 
// {
//     arg = arg;
//     float P_control = 0;
//     float I_control = 0;
//     float D_control = 0;

//     float control_action = 0;

//     float previous_value_error =0;
//     float now_position = 0;
//     float error = 0;

//     while (true)
//     {
//         now_position = getPositionFirstServo();

//         error = the_task_PID_first_serv - now_position;

//         P_control = error * P_coefficient_first_serv;
//         I_control = I_control + error * I_coefficient_first_serv;
//         D_control = (error - previous_value_error) * D_coefficient_first_serv;

//         previous_value_error = error;

//         control_action = P_control + I_control + D_control;
//         turnFirstMotor(control_action);

//         chThdSleepSeconds(1);
//     }
// }

static THD_WORKING_AREA(waPID_action_serv_n, 2048);
static THD_FUNCTION(PID_action_serv_n, arg) 
{

    typedef struct 
    {
        float P_control;
        float I_control;
        float D_control;

        float control_action;

        float previous_value_error;
        float now_position;
        float error;
    } paramServControl_t;
    
    arg = arg;

    paramServControl_t first_serv = {0,0,0,0,0,0,0};
    paramServControl_t second_serv  = {0,0,0,0,0,0,0};
    
    systime_t time =chVTGetSystemTimeX();

    while (true)
    {
        time += MS2ST(20);
        if(flag_control_system_thread)
        {
            first_serv.now_position = getPositionFirstServo();
            second_serv.now_position = getPositionSecondServo();

            first_serv.error = the_task_PID_first_serv - first_serv.now_position;
            second_serv.error = the_task_PID_second_serv - second_serv.now_position;

            first_serv.P_control = first_serv.error * P_coefficient_first_serv;
            second_serv.P_control = second_serv.error * P_coefficient_second_serv;

            first_serv.I_control = first_serv.I_control + first_serv.error * I_coefficient_first_serv;
            second_serv.I_control = second_serv.I_control + second_serv.error * I_coefficient_second_serv;

            first_serv.D_control = (first_serv.error - first_serv.previous_value_error) * D_coefficient_first_serv;
            second_serv.D_control = (second_serv.error - second_serv.previous_value_error) * D_coefficient_second_serv;

            first_serv.previous_value_error = first_serv.error;
            second_serv.previous_value_error = second_serv.error;

            first_serv.control_action = first_serv.P_control + first_serv.I_control + first_serv.D_control;
            second_serv.control_action = second_serv.P_control + second_serv.I_control + second_serv.D_control;

            turnFirstMotor(first_serv.control_action);
            turnSecondMotor(second_serv.control_action);
        }
        else
        {
            first_serv.I_control = 0;
            second_serv.I_control = 0;

            first_serv.previous_value_error = 0;
            second_serv.previous_value_error = 0;

            turnFirstMotor(0);
            turnSecondMotor(0);
        }
        

        chThdSleepUntil(time);
    }
}


void initControlPID(void)
{
    initADC();
    initMotorPWM();
    

    chThdCreateStatic(waPID_action_serv_n, sizeof(waPID_action_serv_n), NORMALPRIO, PID_action_serv_n, NULL /* arg is NULL */);
    // chThdCreateStatic(waPID_action_second_serv_n, sizeof(waPID_action_second_serv_n), NORMALPRIO, PID_action_second_serv_n, NULL /* arg is NULL */);

}

void setTaskFirstServo(float task)
{
    the_task_PID_first_serv = task;
}

void setTaskSecondServo(float task)
{
    the_task_PID_second_serv = task;
}

void enableThreadControl(void)
{
    flag_control_system_thread = true;
}

void disableThreadControl(void)
{
    flag_control_system_thread = false;
}



