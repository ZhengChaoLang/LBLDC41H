#include "drv_speed_sensor.h"
#include "main.h"


void SorSpeed_Init(speed_sensor_t *sor_init)
{
     
    
        
}


float SorSpeed_GetPosition(speed_sensor_t *sensor)
{
    
    if(!sensor) 
        return sensor->get_posi(sensor->drv_handle);  
}

float SorSpeed_GetSpeed(speed_sensor_t *sensor)
{
    if(!sensor) 
        return sensor->get_speed(sensor->drv_handle);   
}

/* 
 * @brief get sensor data of Acceleration
 * @param a sensor handle  
 * @return acceleration rad/s^2
 */
float SorSpeed_GetAcceleration(speed_sensor_t *sensor)
{
    if(!sensor) 
        return sensor->get_acc(sensor->drv_handle);   
}


