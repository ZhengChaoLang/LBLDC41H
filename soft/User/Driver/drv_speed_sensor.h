#ifndef __DVR_SPEED_SENSOR_H_
#define __DVR_SPEED_SENSOR_H_



typedef void(*speed_sor_init)(void*arg);          //sensor init function point
typedef float(*speed_sor_get)(void*arg);          //get sensor  data function point

typedef struct{
    speed_sor_init  drv_init;           // 初始化函数
    speed_sor_get   get_posi;           // 获取位置
    speed_sor_get   get_speed;          // 获取转速
    speed_sor_get   get_acc;            // 获取角加速度
    void* drv_handle;                   // 设备的控制句柄 无则NULL
   
    float posi;                         // 获取位置
    float speed;                        // 获取速度
    float acc;                          // 获取加速度   
    
}speed_sensor_t;





#endif
