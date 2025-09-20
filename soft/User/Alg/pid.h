#ifndef _PID_H__
#define _PID_H__

#include "main.h"

									

/* 模式选择宏定义
 * @{
 */
#define PID_MODE_INITEGRAL_LIMT_ENABLE					1		// 使能积分限幅（限制积分项的最大/最小值）
#define	PID_MODE_INITEGRAL_SEPARATE_ENABLE				1		// 使能积分分离（误差较大时停止积分，减少超调）
#define PID_MODE_FEEDFORWARD_INPUT_ENABLE				1		// 使能输入型前馈（基于输入量的前馈补偿）
#define PID_MODE_FEEDFORWARD_CONST_ENABLE				2		// 使能常数型前馈（固定值的前馈补偿）
#define PID_MODE_INITEGRAL_ISOLATED_ENABLE				1		// 使能积分隔离（误差超过阈值时停止积分）
#define PID_MODE_INITEGRAL_DEADBAND_ENABLE				1		// 使能积分死区（误差在小范围内时不积累积分）
/* 模式失能宏定义（0表示失能）
 * @{
 */
#define PID_MODE_INITEGRAL_LIMT_DISABLE					0		// 失能积分限幅
#define	PID_MODE_INITEGRAL_SEPARATE_DISABLE				0		// 失能积分分离
#define PID_MODE_FEEDFORWARD_INPUT_DISABLE				0		// 失能输入型前馈
#define PID_MODE_FEEDFORWARD_CONST_DISABLE				0		// 失能常数型前馈
#define PID_MODE_INITEGRAL_ISOLATED_DISABLE				0		// 失能积分隔离
#define PID_MODE_INITEGRAL_DEADBAND_DISABLE				0		// 失能积分死区

/*
 * @{
 */
/**
 * @brief PID控制器模式配置结构体
 * 采用位域定义，节省内存空间，各成员仅占用指定位数
 */
typedef struct{
    uint8_t integral_limit:1;      // 积分限幅使能位（1：使能，0：失能）
    uint8_t integral_separate:1;   // 积分分离使能位（1：使能，0：失能）
    uint8_t integral_isolated:1;   // 积分隔离使能位（1：使能，0：失能）
    uint8_t feedforward:2;         // 前馈模式选择位（2位，0：失能前馈，1：输入型前馈，2：常数型前馈）
    uint8_t deadband:1;            // 积分死区使能位（1：使能，0：失能）
}pid_mode_config_t;


/**
 * @brief PID控制器结构体，存储PID控制所需的参数、状态和配置信息
 */
typedef struct{
    float ref;                         // 目标参考值（设定值）
    float kp;                          // 比例系数（Proportional gain）
    float ki;                          // 积分系数（Integral gain）
    float kd;                          // 微分系数（Derivative gain）
    float integra_sum;                 // 积分项累加和（用于计算积分控制量）
    float integral_limt;               // 积分限幅阈值（限制积分项的最大/最小值）
    float integral_isolate_val;        // 积分隔离阈值（误差超过此值时停止积分）
    float out_limt;                    // 输出限幅阈值（限制PID最终输出的最大/最小值）
    float out_last;                    // 上一次PID输出值（用于积分分离等逻辑）
    float err_last;                    // 上一次误差值（用于计算微分项）
    float out;
    float feedforward_const;           // 常数型前馈值（当前馈模式为常数型时使用）
    float (*feedforward_func)(float input); // 输入型前馈函数指针（当前馈模式为输入型时使用）
    float deadband_val;                // 死区阈值（误差在此范围内时不积累积分）
    pid_mode_config_t mode;            // PID模式配置结构体（包含各项功能的使能状态）
}pid_t;


void Pid_Init(pid_t * pid_handle,const pid_mode_config_t *mode);
float Pid(pid_t * pid_handle, float fb);
void Pid_SetGains(pid_t * pid_handle, float kp, float ki, float kd);
void Pid_SetRef(pid_t * pid_handle,float ref);
void Pid_SetOutLimt(pid_t * pid_handle,float limt_value);
void Pid_SetFeedforward_ConstValue(pid_t * pid_handle,float ff_value);
void Pid_SetFeedForward_func(pid_t * pid_handle,float (*func)(float));
void Pid_SetLimtParam(pid_t * pid_handle,float out_limt,float integral_limt);

float Pid_GetOutLimt(pid_t * pid_handle);
float Pid_GetRef(pid_t * pid_handle);
float Pid_GetOupPut(pid_t * pid_handle);
#endif
