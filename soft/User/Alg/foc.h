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
typedef void (*FocPid_GetRef_t)(FOC_PID_T * pid_handle);
typedef float (*FocPid_GetOutput_t)(FOC_PID_T * pid_handle);

#define FOC_HOOK_NUMBER 10
typedef void (*Foc_Hook_t)(void *arg); 

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
	float u_dc;                         ///< 母线电压
	float pole;                         ///< 极对数
	float phase_r;                      ///< 相电阻
	float phase_l;                      ///<
    float phase_lq;                      ///<
    float phase_ld;                      ///<
	float flux;                         ///< 磁链
    
	float theta_m;                      ///<机械角度
	float det_theta_m;                  ///<机械角速度
	float theta_e;                      ///<电角度
	
    foc_phase_prarm_t   phase_i;        ///<相电流
	foc_clark_prarm_t   u_alpha_beta;
    foc_Ctrl_prarm_t    ctrl;           ///<控制器
    FocMode mode;
    
    Foc_Hook_t cal_before_hook[FOC_HOOK_NUMBER];
    Foc_Hook_t cal_after_hook[FOC_HOOK_NUMBER];
    speed_sensor_t* speed_sensor;
}foc_motor_t;




void FOC_Svpwm(foc_motor_t* motor, foc_clark_prarm_t * u_two_aixs);
foc_clark_prarm_t* FOC_CurrentLoopCal(foc_motor_t * motor ,float ref_iq);
float FOC_SpeedLoopCal(foc_motor_t * motor, float ref_speed);
float FOC_PositionLoopCal(foc_motor_t * motor, float ref_position_rad);

void FOC_InvPark(foc_park_prarm_t* park_prarm, float theta, foc_clark_prarm_t* out_param);
void Foc_AddCalBeforeHookFunc(foc_motor_t * motor, Foc_Hook_t hook_func );
void Foc_AddCalAfterHookFunc(foc_motor_t * motor, Foc_Hook_t hook_func );
float FOC_MapPi(float rad);

#endif
