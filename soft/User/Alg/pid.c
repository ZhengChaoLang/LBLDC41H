/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : pid����
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


/**
 * @brief       位置式PID控制器计算函数
 * @param[in]   pid_handle  PID控制器句柄（包含PID参数、状态等信息）
 * @param[in]   fb          反馈值（当前实际测量值）
 * @return      PID控制器计算输出值
 */
float Pid(pid_t * pid_handle, float fb)
{
    float out = 0;                  // PID输出结果
    float err = pid_handle->ref - fb; // 计算当前误差（设定值 - 反馈值）
    
    // 死区处理：当使能积分死区模式时，若误差在死区范围内则清零误差，避免微小波动累积
    if(pid_handle->mode.deadband == PID_MODE_INITEGRAL_DEADBAND_ENABLE){
        if((err < pid_handle->deadband_val) && (err > -pid_handle->deadband_val)){
            err = 0;
        }
    }

    uint8_t integral_flog = 1;              // 积分使能标志（1：允许积分，0：禁止）
    uint8_t integral_isolated_flog = 1;     // 积分隔离标志（1：允许积分，0：禁止）
    
    // 积分分离逻辑：若使能积分分离，且上一次输出超出输出限制，则禁止本次积分
    // 作用：避免输出饱和时积分继续累积导致超调
    if(pid_handle->mode.integral_separate){
        if(!((pid_handle->out_last < pid_handle->out_limt) && (pid_handle->out_last > -pid_handle->out_limt))){
            integral_flog = 0;        // 禁止积分
        }
    }
    
    // 积分隔离逻辑：若使能积分隔离，且误差绝对值超过隔离阈值，则禁止本次积分
    // 作用：大误差时停止积分，减少超调风险
    if(pid_handle->mode.integral_isolated){
        if((err > pid_handle->integral_isolate_val) || (err < -pid_handle->integral_isolate_val)){
            integral_flog = 0;        // 禁止积分
            integral_isolated_flog = 0;
        }
    }
    
    // 积分累加：仅当积分使能时，累加误差到积分和
    if(integral_flog == 1)
        pid_handle->integra_sum += err;
    
    // 积分限幅：若使能积分限幅，限制积分和在设定范围内，防止积分饱和
    if(pid_handle->mode.integral_limit == 1){
        if(pid_handle->integra_sum > pid_handle->integral_limt)
        {
            pid_handle->integra_sum = pid_handle->integral_limt;    
        }
        else if(pid_handle->integra_sum < -pid_handle->integral_limt)
        {
            pid_handle->integra_sum = -pid_handle->integral_limt;
        }
    }
    
    // PID输出计算：比例项(P) + 积分项(I) + 微分项(D)
    // 比例项：当前误差 * 比例系数
    // 积分项：积分和 * 积分系数 * 积分隔离标志
    // 微分项：（当前误差 - 上一次误差）* 微分系数
    out = err * pid_handle->kp + pid_handle->ki * pid_handle->integra_sum * integral_isolated_flog + pid_handle->kd * (err - pid_handle->err_last);
    
    // 前馈补偿：若使能前馈，根据模式添加前馈值，提高响应速度
    if(pid_handle->mode.feedforward != 0){
        switch(pid_handle->mode.feedforward)
        {
            case PID_MODE_FEEDFORWARD_INPUT_ENABLE:  // 输入前馈：通过函数计算前馈值（基于设定值）
                if(pid_handle->feedforward_func != NULL)
                    out += pid_handle->feedforward_func(pid_handle->ref);
                break;
            case PID_MODE_FEEDFORWARD_CONST_ENABLE:  // 常数前馈：添加固定前馈值
                out += pid_handle->feedforward_const;
                break;
            default:
                break;
        }
    }
    
    // 输出限幅：限制输出在设定范围内，避免执行器超限
    if(out > pid_handle->out_limt){
        out = pid_handle->out_limt;
    }
    else if(out < -pid_handle->out_limt){
        out = -pid_handle->out_limt;
    }
    
    // 保存当前状态：用于下一次计算（上一次输出、上一次误差）
    pid_handle->out = out;
    pid_handle->out_last = out;
    pid_handle->err_last = err;
    
    return out;
}

/**
 * @brief       设置PID控制器的目标参考值（设定值）
 * @param[in]   pid_handle  PID控制器句柄（指向PID控制结构体的指针）
 * @param[in]   ref         要设置的目标参考值（即期望达到的目标值）
 * @return      none
 */
void Pid_SetRef(pid_t * pid_handle, float ref)
{
    // 检查PID句柄是否有效（非空），有效则更新参考值
    if(pid_handle)
        pid_handle->ref = ref;
}


/**
 * @brief       设置PID控制器的输出限幅值
 * @param[in]   pid_handle  PID控制器句柄（指向PID控制结构体的指针）
 * @param[in]   limt_value  要设置的输出限幅值（限制PID输出的最大绝对值）
 * @return      无
 */
void Pid_SetOutLimt(pid_t * pid_handle, float limt_value)
{
    // 将指定的限幅值赋值给PID控制器的输出限幅成员变量
    pid_handle->out_limt = limt_value;
}

/**
 * @brief       设置PID控制器的常数前馈值
 * @param[in]   pid_handle  PID控制器句柄（指向PID控制结构体的指针）
 * @param[in]   ff_value    要设置的常数前馈值
 * @return      无
 */
void Pid_SetFeedforward_ConstValue(pid_t * pid_handle,float ff_value)
{
    pid_handle->feedforward_const = ff_value;
}

/**
 * @brief       设置PID控制器的前馈函数
 * @param[in]   pid_handle  PID控制器句柄（指向PID控制结构体的指针）
 * @param[in]   func        前馈函数指针，函数接收一个float参数并返回float值
 * @return      无
 */
void Pid_SetFeedForward_func(pid_t * pid_handle,float (*func)(float))
{
    pid_handle->feedforward_func = func;
}

/**
 * @brief       设置PID控制器的输出限幅和积分限幅参数
 * @param[in]   pid_handle      PID控制器句柄（指向PID控制结构体的指针）
 * @param[in]   out_limt        输出限幅值（限制PID输出的最大绝对值）
 * @param[in]   integral_limt   积分限幅值（限制积分项累加的最大绝对值）
 * @return      无
 */
void Pid_SetLimtParam(pid_t * pid_handle,float out_limt,float integral_limt)
{
    pid_handle->out_limt = out_limt;
    pid_handle->integral_limt = integral_limt;
}

/**
 * @brief       获取PID控制器的输出限幅值
 * @param[in]   pid_handle  PID控制器句柄（指向PID控制结构体的指针）
 * @return      当前设置的输出限幅值
 */
float Pid_GetOutLimt(pid_t * pid_handle)
{
    return pid_handle->out_limt;
}

float Pid_GetOupPut(pid_t * pid_handle)
{
    return pid_handle->out;
}
/**
 * @brief       获取PID控制器的目标参考值（设定值）
 * @param[in]   pid_handle  PID控制器句柄（指向PID控制结构体的指针）
 * @return      当前的目标参考值
 */
float Pid_GetRef(pid_t * pid_handle)
{
    return pid_handle->ref;
}



