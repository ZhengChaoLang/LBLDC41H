#include "app_foc.h"
#include "foc.h"
#include "drv_speed_sensor.h"
#include "drv_hall.h"
#include "rtthread.h"
#include "filter.h"
pid_t foc_pid_id;
pid_t foc_pid_iq;
pid_t foc_pid_speed;
pid_t foc_position;
ema_filter_t speed_loop_lps;

speed_sensor_t m1_hall_sor;
foc_motor_t foc_m1; 

void AppFoc_CalibrationEnc(void * motor);
void AppFoc_CurrentLoop(void *arg);
void AppFoc_SpeedLoop(void *arg);

typedef struct{
   McRunStarus run_status;
}foc_run_status_t;


//focFOC初始化
int FOC_m1_Init()
{
    // 速度传感器
    hall_sor1.posi_updata_dt = 0.0001f;//更新周期
    SorSpeed_SetDrvHand(&m1_hall_sor, &hall_sor1);
    SorSpeed_SetSorFunction(&m1_hall_sor, (void(*)(void*))DrvHall_Init, (float(*)(void*))DrvHall_GetPosi,\
                            (float(*)(void*))DrvHall_GetSpeed, (float(*)(void*))DrvHall_GetAcc);
    SorSpeed_Init(&m1_hall_sor);
    foc_m1.speed_sensor = &m1_hall_sor;
    
    //电机参数
    foc_m1.phase_r = 5.0f;
    foc_m1.phase_l = 0.2f; 
    foc_m1.pole = 4;
    foc_m1.u_dc =24;
    foc_m1.flux=0.1f;
    foc_m1.mode = FOCMODE_SPEED_LOOP;
    //控制算法参数
    pid_mode_config_t pid_modconfig;
    pid_modconfig.deadband = 0;
    pid_modconfig.feedforward = 0;
    pid_modconfig.integral_isolated =0;
    pid_modconfig.integral_limit =1;
    pid_modconfig.integral_separate =1;

    //配置id
    Pid_SetGains(&foc_pid_id, 10.5f, 0.01f, 0.0f);
    Pid_SetLimtParam(&foc_pid_id, 24/2*0.85f, 1000);
    Pid_Init(&foc_pid_id, &pid_modconfig);

    //配置id
    foc_pid_iq = foc_pid_id;
    Pid_Init(&foc_pid_iq, &pid_modconfig);
        
    //速度环
    speed_loop_lps.a = 0.5f;
    
    Pid_SetGains(&foc_pid_speed, 0.008f, 0.001f, 0);
    Pid_SetLimtParam(&foc_pid_speed,5, 500);
    Pid_Init(&foc_pid_speed, &pid_modconfig);
    
    
    //foc连接控制器句柄
    Foc_SetCtrlHandle(&foc_m1, &foc_pid_id, &foc_pid_iq, &foc_pid_speed, NULL);
    Foc_SetCtrlFunction(&foc_m1, Pid, Pid_SetRef, Pid_GetRef, Pid_GetOutLimt);
    
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
            AppFoc_SpeedLoop(m1);
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
float debug_speed_ref =400;//100~400区间良好运行峰顶400
/**
 * @brief FOC控制中的速度环任务函数，周期性执行速度闭环控制逻辑
 * 
 * 该函数作为速度环控制的主任务，负责完成速度检测、速度环PID计算。
 * 
 * @param[in] arg 通用指针参数，实际指向foc_motor_t类型的电机控制结构体，
 *                包含电机控制所需的所有状态和配置信息
 * 
 * @note 1. 函数需在电机初始化完成后调用，确保motor指针及内部成员有效
 *       2. 执行周期需根据电机特性和控制精度要求合理设置
 *       3. 速度环输出经过低通滤波处理，可减少控制量波动
 */
void AppFoc_SpeedLoop(void *arg)
{
    float ref_iq = 0;  // q轴目标电流，作为电流环的输入参考值
    foc_motor_t* motor = (foc_motor_t*)arg;  // 将通用指针转换为电机控制结构体指针
    
    // 从速度传感器获取当前电角度（用于速度环计算）
    motor->det_theta_e = SorSpeed_GetSpeed(motor->speed_sensor);
    
    // 1. 执行速度环PID计算得到原始q轴电流参考值
    // 2. 通过低通滤波器(LPS)平滑处理电流参考值，减少高频噪声
    ref_iq = Filter_LPS(&speed_loop_lps, FOC_SpeedLoopCal(motor, debug_speed_ref));
    
    // 若速度传感器有效，更新电机电角度（范围映射到[-π, π]或[0, 2π]）
    if(motor->speed_sensor)
        motor->theta_e = FOC_MapPi(SorSpeed_GetPosition(motor->speed_sensor));
    
    // 执行电流环计算，根据目标q轴电流ref_iq进行电流闭环控制
    FOC_CurrentLoopCal(motor, ref_iq);       ///< 目标电流值
    
    // 基于电流环输出计算SVPWM（空间矢量脉宽调制）信号，驱动功率器件
    FOC_Svpwm(motor, &motor->u_alpha_beta);   
}
//rt_messagequeue foc_idrefQueue
void AppFoc_CurrentLoop(void *arg)
{
    foc_motor_t* motor = (foc_motor_t*)arg;
//    static float theta =0;
//    theta = FOC_MapPi(theta + 0.005F);
     if(motor->speed_sensor )
        motor->theta_e = FOC_MapPi(SorSpeed_GetPosition(motor->speed_sensor));
    //motor->theta_e = theta;
    
     FOC_CurrentLoopCal(motor, -1.0f);       ///< 目标电流值
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











