#include "app_foc.h"
#include "foc.h"
#include "drv_speed_sensor.h"
#include "drv_hall.h"
#include "rtthread.h"
pid_t foc_pid_id;
pid_t foc_pid_iq;
pid_t foc_pid__speed;
pid_t foc_position;

speed_sensor_t m1_hall_sor;
foc_motor_t foc_m1; 

void AppFoc_CalibrationEnc(void * motor);
void AppFoc_CurrentLoop(void *arg);
typedef struct{
   McRunStarus run_status;
}foc_run_status_t;


//focFOC初始化
int FOC_m1_Init()
{

    // 速度传感器
    hall_sor1.posi_updata_dt = 0.0001f;
    m1_hall_sor.drv_handle  = &hall_sor1;
    m1_hall_sor.drv_init    = (void(*)(void*))DrvHall_Init;
    m1_hall_sor.get_posi    = (float(*)(void*))DrvHall_GetPosi;
    m1_hall_sor.get_speed   = (float(*)(void*))DrvHall_GetSpeed;
    m1_hall_sor.get_acc     = (float(*)(void*))DrvHall_GetAcc;
    
    SorSpeed_Init(&m1_hall_sor);
    foc_m1.speed_sensor = &m1_hall_sor;
    
    //电机参数
    foc_m1.phase_r = 5.0f;
    foc_m1.phase_l = 0.2f; 
    foc_m1.pole = 4;
    foc_m1.u_dc =24;
    foc_m1.flux=0.1f;
    foc_m1.mode = FOCMODE_SVPWM;
    //控制算法参数
    pid_mode_config_t pid_modconfig;
    pid_modconfig.deadband = 0;
    pid_modconfig.feedforward = 0;
    pid_modconfig.integral_isolated =0;
    pid_modconfig.integral_limit =1;
    pid_modconfig.integral_separate =0;
    
    foc_pid_id.kp = 3.0f;
    foc_pid_id.ki = 0.0f;
    foc_pid_id.kd = 0.0f;
    Pid_SetLimtParam(&foc_pid_id, 24/2*0.85f, 5000);
    Pid_Init(&foc_pid_id, &pid_modconfig);
    
    foc_pid_iq.kp = 3.0f;
    foc_pid_iq = foc_pid_id;
    Pid_Init(&foc_pid_iq, &pid_modconfig);
    
    foc_m1.ctrl.pid_currentLoop_id = &foc_pid_id;
    foc_m1.ctrl.pid_currentLoop_iq = &foc_pid_iq;
    
    foc_m1.ctrl.pid_getOutput = Pid_GetOutLimt;
    foc_m1.ctrl.pid_setRef = Pid_SetRef;
    foc_m1.ctrl.pid_getRef = Pid_GetRef;
    foc_m1.ctrl.pid_cal = Pid;
    //初始化
    Foc_AddCalBeforeHookFunc(&foc_m1, DrvHall_InterpolationPosi, &hall_sor1, "hall_posi");
    return 0;
}
INIT_DEVICE_EXPORT(FOC_m1_Init);

float debug_theta;
//foc_run 
void AppFoc_RunStep(void * arg)
{
    
   float controls;
   foc_motor_t* m1 = arg;
   foc_park_prarm_t park_prarm;
   park_prarm.d = 0;
   park_prarm.q = 3;
   
   foc_clark_prarm_t clk;
   static int cnt =0;
   static float theta = 0;
   cnt ++;
   Foc_BeforeHookRun(m1);
    //角度
   if(cnt> 1){
        theta = FOC_MapPi(theta + 0.005f);
        debug_theta = theta;
        cnt = 0;        
    }
   switch (m1->mode)
   {
        case FOCMODE_POSI_LOOP: 
            //FOC_PositionLoopCal();
            break;
        case FOCMODE_SPEED_LOOP:
            
            break;
        case FOCMODE_CURRENT_LOOP:
            AppFoc_CurrentLoop(m1);
            break;
        case FOCMODE_SVPWM:  
            FOC_InvPark(&park_prarm, theta, &clk);
            FOC_Svpwm(m1, &clk);
            break;
        case FOCMODE_CALI:
            AppFoc_CalibrationEnc((void*)m1);  
            break;
        default:
            break;
   }     
}


void AppFoc_Vf(void *arg)
{
    
    
    
    
}

//rt_messagequeue foc_idrefQueue
void AppFoc_CurrentLoop(void *arg)
{
    foc_motor_t* motor = (foc_motor_t*)arg;
    static float theta =0;
    theta = FOC_MapPi(theta + 0.01f);
    motor->theta_e = theta;
    FOC_CurrentLoopCal(motor, 0.1f);
    FOC_Svpwm(motor, &motor->u_alpha_beta);
}


//foc_run 
void AppFoc_CalibrationEnc(void * motor)
{
   if(!motor)return;
    
    foc_motor_t* m1 = motor;
    foc_park_prarm_t park_prarm;
    park_prarm.d = 1;
    park_prarm.q = 0;
    foc_clark_prarm_t clk;
    
    FOC_InvPark(&park_prarm, 0, &clk);
    FOC_Svpwm(m1, &clk);
    rt_thread_mdelay(300);
    if(m1->speed_sensor )
        m1->theta_e = SorSpeed_GetPosition(m1->speed_sensor);//获取电角度 
}

//棘轮
void AppFoc_Ratchet(void * motor)
{
    
   
}











