#ifndef __FILTER_H__
#define  __FILTER_H__

typedef struct{
	float a;			///滤波系数
	float data_last;	///上一次的数据
}ema_filter_t;


extern ema_filter_t filter_m2_speed;


void Filter_Init(ema_filter_t * filter_handle,float a);
float Filter_LPS(ema_filter_t * filter_handle,float now_data);
#endif
