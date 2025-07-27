#ifndef __ALGO_MATH_H__

#include <stdint.h>
#include "arm_math.h"
#include "stdbool.h"
/*
 * @ 导数结构体
 */
typedef struct{
	float value_last_k;
	float dt;
	float diff_value;
}algo_ddiff_t;
/*
 *
 */
/*
 * @ 积分结构体
 */
typedef struct{
	float dt;
	float integrate_value;
}algo_integrate_t;
/*
 *
 */

// 定义斜坡发生器状态的结构体
typedef struct RampGenerator
{
    float currentValue; // 当前值
    float targetValue;  // 目标值
    float step;           // 每个控制周期应当改变的数值大小
		float cycleTime; 
    bool isBusy;        // 指示斜坡发生器是否正在调整中
} RampGenerator;

#ifndef     PI
    #define 	PI						3.1415926f
#endif
#define 		HAIF_PI				(PI/2)

#define 		GRAVITY_ACC		9.8f



#define     GRAVITY_TO_MS(g)				(g*GRAVITY_ACC)				//g转成m.s

#define 	ALGO_CONSTRAIN(amt,low,high)  		(amt < low ? low : (amt > high ? high : amt))				//饱和
#define   ALGO_DEAD_ZONE(vla,range)					((vla > range||vla < -range) ? vla : 0)


//线性映射 函数
int float_to_uint(float float_number, float number_min, float number_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_int(float float_val, float float_min, float float_max, uint8_t bits);
float int_to_float(int int_val, float float_min, float float_max, uint8_t bits);

//初始化一个离散求导句柄
void ALGO_DiscreteDiffInit(algo_ddiff_t*algo_ddiff, float dt);
//求导
float ALGO_DiscreteDiff(algo_ddiff_t* diff_hanlde,float value_k);
void ALGO_IntegratInit(algo_integrate_t*algo_integrate, float dt);
float AlGO_DiscreteIntegrat(algo_integrate_t* integrate_hanlde,float value_k);


void RampSet(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime);
float RampGetCurrentValue(RampGenerator *ramp);
void RampSetTarget(RampGenerator *ramp, float targetValue, float time);
void RampIterate(RampGenerator *ramp);
#endif
