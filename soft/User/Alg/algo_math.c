#include "algo_math.h"


//浮点无符号线性映射
int float_to_uint(float float_number, float number_min, float number_max, uint8_t bits)
{
	
	uint32_t uint_max = ((1<<bits)-1);
	///检查参数 
	if(number_min >= number_max){
		return 0;
	}
	if(float_number < number_min || float_number > number_max)
	{
		return 0;
	}
	if(bits >31){
		return 0;
	}
	
	return (int)((float_number - number_min)*(float)uint_max/(number_max - number_min));
}



/*
 * @brief: 无符号/线性映射
 * @param[in]:x_int 需要转换的整数
 * @param[in]:x_min x_min  转换后最小
 * @param[in]:x_max x_max 转换后最大
 * @param[in]:bits 位数
 * @return:
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
 /// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief 将整数线性映射到浮点数范围
 * @param int_val   输入整数值
 * @param float_min 输出浮点数下限
 * @param float_max 输出浮点数上限
 * @param bits      整数位数（决定输入范围，如8位对应-128~127）
 * @return float    映射后的浮点数
 */
float int_to_float(int int_val, float float_min, float float_max, uint8_t bits) {
    // 计算整数范围
    int int_min = -(1 << (bits - 1));      // 如8位: -128
    int int_max = (1 << (bits - 1)) - 1;   // 如8位: 127

    // 边界约束
    int_val = (int_val < int_min) ? int_min : int_val;
    int_val = (int_val > int_max) ? int_max : int_val;

    // 线性映射公式
    float normalized = (float)(int_val - int_min) / (float)(int_max - int_min);
    return float_min + normalized * (float_max - float_min);
}


/**
 * @brief 将浮点数线性映射回整数范围
 * @param float_val 输入浮点数值
 * @param float_min 浮点数下限（必须与int_to_float使用相同的值）
 * @param float_max 浮点数上限（必须与int_to_float使用相同的值）
 * @param bits      整数位数（决定输出范围，如8位对应-128~127）
 * @return int      映射后的整数值
 */
int float_to_int(float float_val, float float_min, float float_max, uint8_t bits) {
    // 计算整数范围
    int int_min = -(1 << (bits - 1));      // 如8位: -128
    int int_max = (1 << (bits - 1)) - 1;   // 如8位: 127

    // 边界约束
    float_val = (float_val < float_min) ? float_min : float_val;
    float_val = (float_val > float_max) ? float_max : float_val;

    // 逆向线性映射公式
    float normalized = (float_val - float_min) / (float_max - float_min);
    return int_min + (int)(normalized * (int_max - int_min) + 0.5f); // +0.5f用于四舍五入
}

//初始化一个离散求导句柄
void ALGO_DiscreteDiffInit(algo_ddiff_t*algo_ddiff, float dt)
{
	algo_ddiff->diff_value = 0;
	algo_ddiff->dt = dt;
	algo_ddiff->value_last_k = 0;
}
//求导
float ALGO_DiscreteDiff(algo_ddiff_t* diff_hanlde,float value_k){
	
	 diff_hanlde->diff_value = (value_k - diff_hanlde->value_last_k)/diff_hanlde->dt;
	 diff_hanlde->value_last_k = value_k;
	return diff_hanlde->diff_value;
}
//初始化一个积分句柄
void ALGO_IntegratInit(algo_integrate_t*algo_integrate, float dt)
{
	algo_integrate->dt = dt;
	algo_integrate->integrate_value = 0;
}
//积分
float AlGO_DiscreteIntegrat(algo_integrate_t* integrate_hanlde,float value_k)
{
	integrate_hanlde->integrate_value += 	(value_k*integrate_hanlde->dt);
	return integrate_hanlde->integrate_value;
}






/**
 * @ 一个周期内对斜坡发生器状态的更新
 * @param[in] ramp 斜坡对象
 */
void RampIterate(RampGenerator *ramp)
{
    if (ramp->isBusy)
    {
        //printf("Current Value Updated: %f\n", ramp->currentValue); // 添加此行代码以在每次迭代后打印当前值
        ramp->currentValue += ramp->step; // 增大当前值 
        if ((ramp->currentValue > ramp->targetValue && ramp->step>0)||(ramp->currentValue < ramp->targetValue && ramp->step<0))
        {                                     // 如果当前值小于目标值
           ramp->currentValue = ramp->targetValue;  
        }
        // 判断是否达到目标
        if (ramp->currentValue == ramp->targetValue)
        {
            ramp->isBusy = false; // 达到目标，标记为不忙碌
        }
    }
}

/**
 * @ 返回当前值
 * @param[in] ramp 斜坡对象
 */
float RampGetCurrentValue(RampGenerator *ramp){
  return ramp->currentValue;
}


/**
 * @ 不设置当前值，直接设置目标值
 * @param[in] ramp 斜坡对象
 * @param[in] targetValue 目标
 * @param[in] time 到达时间
 */
void RampSetTarget(RampGenerator *ramp, float targetValue, float time){
		RampSet(ramp,ramp->currentValue,targetValue,time, ramp->cycleTime);
}
/**
 * @ Set斜坡发生器
 * @param[in] startValue 设置开始值
 * @param[in] targetValue 目标值
 * @param[in] time 所用时间 s
 * @param[in] cycleTime 采样时间 s
 */
void RampSet(RampGenerator *ramp, float startValue, float targetValue, float time, float cycleTime)
{
    ramp->currentValue = startValue;
    ramp->targetValue = targetValue;
    // 计算步进值，这里需要注意的是，确保斜坡时间和周期时间都不为零来避免除以零的错误
    if (time != 0 && cycleTime != 0)
    {
        ramp->step = (targetValue - startValue) * (cycleTime / time);
				ramp->cycleTime = cycleTime;
    }
    else
    {
        ramp->step = 0; // 出错情况下设置为0，避免非法操作
    }
    ramp->isBusy = true; // 标记为忙碌
}



