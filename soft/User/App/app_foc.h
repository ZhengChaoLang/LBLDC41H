#ifndef __APP_FOC_H_
#define __APP_FOC_H_


/**
 * @brief 电机运行状态枚举
 * 定义电机的基本运行状态，用于电机控制系统中标识当前的工作模式或状态，
 */
typedef enum{
    STOP = 0,   // 停止状态，电机未启动或已停止运行，所有绕组无驱动信号
    RUN,        // 运行状态，电机正常工作中，按照控制逻辑进行换相和转速调节
    FAULT       // 故障状态，电机检测到异常（如过流、过压、霍尔信号错误等），触发保护机制
}McRunStarus;


/**
 * @brief FOC（磁场定向控制）模式枚举
 * 定义FOC控制系统支持的不同控制模式，用于标识当前系统采用的控制策略层级，
 * 不同模式对应不同的控制环路和调节目标。
 */
typedef enum{
    FOCMODE_SVPWM,            // SVPWM模式（空间矢量脉宽调制），基础的PWM生成模式，直接输出电压矢量
    FOCMODE_CURRENT_LOOP,     // 电流环模式，通过闭环控制实现对电机相电流的精确调节
    FOCMODE_SPEED_LOOP,       // 速度环模式，在电流环基础上增加速度闭环控制，实现目标转速跟踪
    FOCMODE_POSI_LOOP         // 位置环模式，在速度环基础上增加位置闭环控制，实现精确的位置控制
}FocMode;

void AppFoc_RunStep(void * motor);

int FOC_m1_Init();
 
#endif
