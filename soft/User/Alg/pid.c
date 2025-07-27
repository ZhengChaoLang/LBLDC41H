/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : pid代码
  ******************************************************************************
  */
#include "pid.h"

void Pid_Init(pid_t * pid_handle,const pid_mode_config_t *mode)
{
	pid_handle->err_last = 0;
	pid_handle->integra_sum =0;
	pid_handle->out_last = 0;
	pid_handle->mode = *mode;
}


 /*
 * @brief		计算本次pid
 * @param[pid_handle]	本次计算pid的句柄
 * @param[fb]	反馈值
 * @return	pid计算结果
 */
float Pid(pid_t * pid_handle, float fb)
{
	float out = 0;
	float err = pid_handle->ref - fb;
	
	//死区处理
	if(pid_handle->mode.deadband == PID_MODE_INITEGRAL_DEADBAND_ENABLE){
		if((err < pid_handle->deadband_val)&&(err > -pid_handle->deadband_val)){
			err = 0;
		}
	}

	uint8_t integral_flog = 1 ,integral_isolated_flog = 1;
	//积分抗饱和
	if(pid_handle->mode.integral_separate){
		if(!((pid_handle->out_last < pid_handle->out_limt)&&(pid_handle->out_last > -pid_handle->out_limt))){
				integral_flog =0;			//不积分
		}
	}
	//积分分离
	if(pid_handle->mode.integral_isolated){
		if((pid_handle->integral_isolate_val > err)||(pid_handle->integral_isolate_val < -err)){
			integral_flog =0;			//不积分
			integral_isolated_flog =0 ;
		}
	}
	
	
	if(integral_flog == 1)
		pid_handle->integra_sum += err;
	
	//积分限幅
	if(pid_handle->mode.integral_limit == 1){
		if(pid_handle->integra_sum >pid_handle->integral_limt)
		{
			pid_handle->integra_sum = pid_handle->integral_limt;	
		}
		else if(pid_handle->integra_sum < -pid_handle->integral_limt)
		{
			pid_handle->integra_sum = -pid_handle->integral_limt;
		}
	}
	///pid公式
	out = err * pid_handle->kp + pid_handle->ki* pid_handle->integra_sum*integral_isolated_flog + pid_handle->kd * (err - pid_handle->err_last);
	
	//前馈
	if(pid_handle->mode.feedforward != 0){
		switch(pid_handle->mode.feedforward)
		{
			case PID_MODE_FEEDFORWARD_INPUT_ENABLE:
				if(pid_handle->feedforward_func!=NULL)out += pid_handle->feedforward_func(pid_handle->ref);
				
				break;
			case PID_MODE_FEEDFORWARD_CONST_ENABLE:
				out += pid_handle->feedforward_const;
				break;

			default:
				break;
		}
	}
	
	///输出限幅
	if(out > pid_handle->out_limt){
		out = pid_handle->out_limt;
	}
	else if(out < -pid_handle->out_limt){
		out = -pid_handle->out_limt;
	}
	//k=k+1时刻
	pid_handle->out_last = out;
	pid_handle->err_last = err;
	
	return out;
}

 /**
 * @brief	设置pid期望值
 * @param[in] pid_handle	本次计算pid的句柄
 * @param[in]	期望值
 * @return	none
 */
void Pid_SetRef(pid_t * pid_handle,float ref)
{
	if(pid_handle)
		pid_handle->ref = ref;
}



/*
 * @brief: 设置输出极限
 * @param[in]: limt_value 输出极限
 * @return:
 */
void Pid_SetOutLimt(pid_t * pid_handle,float limt_value)
{
	pid_handle->out_limt = limt_value;

}


/*
 * @brief: Pid_SetFeedforward_ConstValue 设置常数前馈的值
 * @param[in]: ff_value 前馈值
 * @return: no
 */
void Pid_SetFeedforward_ConstValue(pid_t * pid_handle,float ff_value)
{
	pid_handle->feedforward_const = ff_value;
}

void Pid_SetFeedForward_func(pid_t * pid_handle,float (*func)(float))
{
	pid_handle->feedforward_func = func;
	
}

//设置极限参数
/*
 * @brief:
 * @param[in]: pid_handle pid的句柄
 * @param[in]: out_limt 输出极限
 * @param[in]: integral_limt 积分极限
 * @return: no
 */
void Pid_SetLimtParam(pid_t * pid_handle,float out_limt,float integral_limt)
{
	pid_handle->out_limt = out_limt;
	pid_handle->integral_limt = integral_limt;
}


/*
 * @brief: 获取输出极限
 * @param[in]: pid_handle
 * @return: 输出极限值
 */
float Pid_GetOutLimt(pid_t * pid_handle)
{
	return pid_handle->out_limt;
}


/*
 * @brief: 获取目标值
 * @param[in]: pid_handle
 * @return: 目标值
 */
float Pid_GetRef(pid_t * pid_handle)
{
	return pid_handle->ref;
}





