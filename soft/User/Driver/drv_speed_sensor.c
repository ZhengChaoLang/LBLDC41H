#include "drv_speed_sensor.h"
#include "main.h"


/* 
 * @brief 初始化速度传感器
 * @param a sensor handle  
 * @return 
 */
void SorSpeed_Init(speed_sensor_t *sor_init)
{    
    sor_init->posi = 0;
    sor_init->speed = 0;
    sor_init->acc = 0;
    sor_init->drv_init(sor_init->drv_handle); 
}


float SorSpeed_GetPosition(speed_sensor_t *sensor)
{

    if(sensor && sensor->get_posi &&sensor->drv_handle) {
        sensor->posi = sensor->get_posi(sensor->drv_handle);
        return sensor->posi;
    }  
    return 0;
}

/* 
 * @brief get sensor data of Speed
 * @param a sensor handle  
 * @return acceleration rad/s
 */
float SorSpeed_GetSpeed(speed_sensor_t *sensor)
{
    if(sensor){
        sensor->speed = sensor->get_speed(sensor->drv_handle);
        return sensor->speed;
    }
        return 0;   
}

/* 
 * @brief get sensor data of Acceleration
 * @param a sensor handle  
 * @return acceleration rad/s^2
 */
float SorSpeed_GetAcceleration(speed_sensor_t *sensor)
{
    if(sensor){
        sensor->acc = sensor->get_acc(sensor->drv_handle);
        return  sensor->acc;
    } 
    return 0;
}

/**
 * @brief: 配置速度传感器的功能回调函数
 *         用于为速度传感器结构体设置初始化及数据获取相关的回调接口，
 *         建立传感器操作函数与传感器实例的关联
 * @param[in]: sensor - 指向速度传感器结构体的指针，待配置的传感器实例
 * @param[in]: init_func - 传感器初始化函数指针，用于完成传感器的初始化操作
 * @param[in]: get_posi - 获取位置数据的函数指针
 * @param[in]: get_speed - 获取速度数据的函数指针
 * @param[in]: get_acc - 获取加速度数据的函数指针
 * @return: 0 - 配置成功；1 - 配置失败（sensor、init_func或get_speed为NULL）
 */
uint8_t SorSpeed_SetSorFunction(speed_sensor_t *sensor, speed_sor_init init_func, speed_sor_get get_posi, speed_sor_get get_speed, speed_sor_get get_acc)
{
    if(!(sensor && init_func && get_speed))return 1;

    sensor->drv_init = init_func;
    sensor->get_posi = get_posi;
    sensor->get_speed = get_speed;
    sensor->get_acc = get_acc;

    return 0;   
}

/**
 * @brief: 配置速度传感器的驱动句柄
 * @param[in]: sensor - 指向速度传感器结构体的指针，待配置的传感器实例
 * @param[in]: drv_sor_handle - 传感器驱动句柄（指向具体传感器驱动的结构体）
 * @return: 0 - 配置成功；1 - 配置失败（sensor或drv_sor_handle为NULL）
 */
uint8_t SorSpeed_SetDrvHand(speed_sensor_t *sensor, void * drv_sor_handle)
{
    if(!(sensor && drv_sor_handle))return 1;
    sensor->drv_handle = drv_sor_handle;
    return 0;   
}
