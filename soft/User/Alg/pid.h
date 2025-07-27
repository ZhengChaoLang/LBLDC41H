#ifndef _PID_H__
#define _PID_H__

#include "main.h"

									

/* 模式选择
 * @{
 */
#define PID_MODE_INITEGRAL_LIMT_ENABLE						1					//积分限幅
#define	PID_MODE_INITEGRAL_SEPARATE_ENABLE				1					//积分抗饱和
#define PID_MODE_FEEDFORWARD_INPUT_ENABLE					1					//函数形式前馈
#define PID_MODE_FEEDFORWARD_CONST_ENABLE					2				 	//常数形式前馈
#define PID_MODE_INITEGRAL_ISOLATED_ENABLE				1					//积分分离
#define PID_MODE_INITEGRAL_DEADBAND_ENABLE				1					//积分分离
/*
 * @{
 */
typedef struct{
	uint8_t integral_limit:1;			//积分限幅
	uint8_t integral_separate:1;	//积分抗饱和
	uint8_t integral_isolated:1;		//积分分离
	uint8_t feedforward:2;				//0 关闭 1带输入的 2常数前馈
	uint8_t deadband:1;						//死区
}pid_mode_config_t;


typedef struct{
	float ref;
	float kp;
	float ki;
	float kd;	
	float integra_sum;
	float integral_limt;
	float integral_isolate_val;
	float out_limt;
	float out_last;
	float err_last;						
	float feedforward_const;										//前馈为常数时
	float (*feedforward_func)(float input);			                //前馈函数
	float deadband_val;												//死区值
	pid_mode_config_t mode;
}pid_t;



void Pid_Init(pid_t * pid_handle,const pid_mode_config_t *mode);
float Pid(pid_t * pid_handle, float fb);

void Pid_SetRef(pid_t * pid_handle,float ref);
void Pid_SetOutLimt(pid_t * pid_handle,float limt_value);
void Pid_SetFeedforward_ConstValue(pid_t * pid_handle,float ff_value);
void Pid_SetFeedForward_func(pid_t * pid_handle,float (*func)(float));
void Pid_SetLimtParam(pid_t * pid_handle,float out_limt,float integral_limt);

float Pid_GetOutLimt(pid_t * pid_handle);
float Pid_GetRef(pid_t * pid_handle);
#endif
