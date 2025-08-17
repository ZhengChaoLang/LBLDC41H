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

