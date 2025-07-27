#include "app_foc.h"
#include "foc.h"
pid_t foc_pid_id;
pid_t foc_pid_iq;
pid_t foc_pid__speed;
pid_t foc_position;

foc_motor_t m1; 



typedef struct{
   McRunStarus run_status;
    
    
}foc_run_status_t;

static void FOC_PID_Init()
{
	pid_mode_config_t pid_mode_config;
	foc_pid_id.kp = 0;
	foc_pid_id.ki = 0;
	foc_pid_iq.kp = foc_pid_id.kp;
	foc_pid_iq.ki = foc_pid_id.ki;
    
}

//foc³õÊ¼»¯
void FOC_Init(foc_motor_t *motor, float u_dc, float phase_r, float phase_l,float pole)
{
	motor->u_dc = u_dc;
	motor->phase_r = phase_r;
	motor->phase_l = phase_l;
	motor->pole = pole;
	FOC_PID_Init();
    
}




//foc_run º¯Êý
void AppFoc_RunStep(void * motor)
{
   float controls;
   switch (((foc_motor_t*)motor)->mode)
   {
        case POSI_LOOP: 
            //FOC_PositionLoopCal();
            break;
        case SPEED_LOOP:
            
            break;
        case CURRENT_LOOP:
            break;
        case SVPWM:
            break;        
        default:
            break;
   }     
}









