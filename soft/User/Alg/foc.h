#ifndef __FOC_H_
#define __FOC_H_

#include "arm_math.h"
#include "main.h"
#include "tim.h"
#include "pid.h"
#include "app_foc.h"
#include "drv_speed_sensor.h"
#define FOC_PWM_SET_VLAUE_U(val)			__HAL_TIM_SET_COMPARE(&htim1,	TIM_CHANNEL_1,val)
#define FOC_PWM_SET_VLAUE_V(val)			__HAL_TIM_SET_COMPARE(&htim1,   TIM_CHANNEL_2,val)
#define FOC_PWM_SET_VLAUE_W(val)			__HAL_TIM_SET_COMPARE(&htim1,	TIM_CHANNEL_3,val)
//foc_param_def
#define FOC_CLARK_GAIN			0.6666667f
#define FOC_SQRT_3				1.732050875f
#define FOC_PWM_T_COUNT			(5000)

#define FOC_SIN(x)					arm_sin_f32(x)
#define FOC_COS(x)					arm_cos_f32(x)

/*-- --*/
#ifndef PI
    #define PI 3.14159265359f
#endif


#define 	DEG_TO_RAD(deg)					(deg*PI/180.0f)
#define		RAD_TO_DEG(rad)					(rad*180.0f/PI)

#define 	OMEGA_TO_RPM(omega)			    (9.55f*omega)       
#define 	RPM_TO_OMEGA(rpm)				((float)rpm/9.55f)

typedef struct{
	float d;
	float q;
}foc_park_prarm_t;

typedef struct{
	float alpha;
	float beta;
}foc_clark_prarm_t;

typedef struct{
	float a;
	float b;
	float c;	
}foc_phase_prarm_t;

#define FOC_PID_T   pid_t
typedef float (*FocPid_Cal_t)(FOC_PID_T * pid_handle, float fb);
typedef void (*FocPid_SetRef_t)(FOC_PID_T * pid_handle, float ref);
typedef float (*FocPid_GetRef_t)(FOC_PID_T * pid_handle);
typedef float (*FocPid_GetOutput_t)(FOC_PID_T * pid_handle);

#define FOC_HOOK_NUMBER 10
typedef void (*Foc_Hook_t)(void *arg); 
typedef struct{
    Foc_Hook_t hook;       //回调函数
    void* param;           //参数
    const char* name;
}foc_hook_t;                                //回调函数


typedef struct{
	FOC_PID_T* pid_currentLoop_id;
    FOC_PID_T* pid_currentLoop_iq;	
    FOC_PID_T* pid_speedLoop;
    FOC_PID_T* pid_positionLoop;
    FocPid_Cal_t        pid_cal;
    FocPid_SetRef_t     pid_setRef;
    FocPid_GetRef_t     pid_getRef;
    FocPid_GetOutput_t  pid_getOutput;
}foc_Ctrl_prarm_t;


typedef struct{
	float u_dc;                         ///< ĸ�ߵ�ѹ
	uint8_t pole;                         ///< ������
	float phase_r;                      ///< �����
	float phase_l;                      ///<
    float phase_lq;                      ///<
    float phase_ld;                      ///<
	float flux;                         ///< ����
    
	float theta_m;                      ///<��е�Ƕ�
	float det_theta_m;                  ///<��е���ٶ�
	float theta_e;                      ///<��Ƕ�
	float det_theta_e;
        
    foc_phase_prarm_t   phase_i;        ///<�����
	foc_clark_prarm_t   u_alpha_beta;
    foc_park_prarm_t i_d_q;
    foc_Ctrl_prarm_t    ctrl;           ///<������
    FocMode mode;
    
    foc_hook_t cal_before_hook[FOC_HOOK_NUMBER];
    foc_hook_t cal_after_hook[FOC_HOOK_NUMBER];
    speed_sensor_t* speed_sensor;
}foc_motor_t;



// 克拉克变换
void FOC_Clark(foc_phase_prarm_t* phase_prarm, foc_clark_prarm_t* out_prarm);

// 帕克变换（Park变换）
void FOC_Park(foc_clark_prarm_t* clark_prarm, float theta, foc_park_prarm_t* out_prarm);

// 帕克逆变换
void FOC_InvPark(foc_park_prarm_t* park_prarm, float theta, foc_clark_prarm_t* out_param);

// SVPWM生成
void FOC_Svpwm(foc_motor_t* motor, foc_clark_prarm_t * u_two_aixs);

// FOC电流环计算
foc_clark_prarm_t* FOC_CurrentLoopCal(foc_motor_t * motor, float ref_iq);

// FOC速度环计算
float FOC_SpeedLoopCal(foc_motor_t * motor, float ref_speed);

// FOC位置环计算
float FOC_PositionLoopCal(foc_motor_t * motor, float ref_position_rad);

// 原函数实现实际映射到[0, 2π]
float FOC_MapPi(float rad);

// 添加FOC计算前的钩子函数
int Foc_AddCalBeforeHookFunc(foc_motor_t * motor, Foc_Hook_t hook_func, void *param, const char* name);

// 执行FOC计算前的钩子函数
void Foc_BeforeHookRun(foc_motor_t * motor);

// 设置FOC计算前钩子函数的参数
uint8_t Foc_SetBeforeHookParam(foc_motor_t * motor, const char* name, void* param);

// 设置FOC计算后钩子函数的参数
uint8_t Foc_SetAfterHookParam(foc_motor_t * motor, const char* name, void* param);

// 添加FOC计算后的钩子函数
int Foc_AddCalAfterHookFunc(foc_motor_t * motor, Foc_Hook_t hook_func, void *param, const char* name);

// 机械角度转换为电角度
float Foc_Mech2ElecAngle(foc_motor_t * motor, float angle);

// 电角度转换为机械角度
float Foc_Elec2MechAngle(foc_motor_t * motor, float angle);


uint8_t Foc_SetCtrlHandle(foc_motor_t * motor, FOC_PID_T* id_handle , FOC_PID_T* iq_handle ,FOC_PID_T* speed_handle ,FOC_PID_T* posi_handle);
uint8_t Foc_SetCtrlFunction(foc_motor_t * motor,FocPid_Cal_t cal, FocPid_SetRef_t set_ref, FocPid_GetRef_t get_ref, FocPid_GetOutput_t get_out);


#endif
