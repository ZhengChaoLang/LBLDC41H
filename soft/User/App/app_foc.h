#ifndef __APP_FOC_H_
#define __APP_FOC_H_


// 电机运行状态
typedef enum{
    STOP = 0,
    RUN,
    FAULT               //故障
}McRunStarus;


typedef enum{
    SVPWM           ,   //svpwm开环
    CURRENT_LOOP    ,   //电流环
    SPEED_LOOP      ,   //速度环
    POSI_LOOP       ,   //位置环
}FocMode;

void AppFoc_RunStep(void * motor);

#endif
