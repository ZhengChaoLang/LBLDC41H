#include "app_foc.h"
#include "foc.h"
#include "drv_speed_sensor.h"
#include "drv_hall.h"
#include "rtthread.h"
pid_t foc_pid_id;
pid_t foc_pid_iq;
pid_t foc_pid__speed;
pid_t foc_position;

foc_motor_t foc_m1; 



typedef struct{
   McRunStarus run_status;
}foc_run_status_t;


//focFOC初始化
int FOC_m1_Init()
{

    // 速度传感器
    static speed_sensor_t hall_sor1;
    hall_sor1.drv_init = (void(*)(void*))DrvHall_Init;
    hall_sor1.get_posi = (float(*)(void*))DrvHall_GetPosi;
    hall_sor1.get_speed = (float(*)(void*))DrvHall_GetAcc;
    SorSpeed_Init(&hall_sor1);
    
    //电机参数
    foc_m1.phase_r = 5.0f;
    foc_m1.phase_l = 0.2f; 
    foc_m1.pole = 4;
    foc_m1.u_dc =24;
    foc_m1.flux=0.1f;
    foc_m1.mode = FOCMODE_SVPWM;
    //控制算法参数
    
    
    //初始化
    Foc_AddCalBeforeHookFunc(&foc_m1, DrvHall_InterpolationPosi);
    return 0;
}
INIT_DEVICE_EXPORT(FOC_m1_Init);


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
    if(cnt> 1){
        theta = FOC_MapPi(theta + 0.01f);
        
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
            break;
        case FOCMODE_SVPWM:  
            FOC_InvPark(&park_prarm, theta, &clk);
            FOC_Svpwm(m1, &clk);
            break;        
        default:
            break;
   }     
}


//foc_run 
void AppFoc_CalibrationEnc(void * motor)
{
   
}

//棘轮
void AppFoc_Ratchet(void * motor)
{
    
   
}











