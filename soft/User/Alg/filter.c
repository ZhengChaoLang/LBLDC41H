/**
  ******************************************************************************
  * @file           : filter.c
  * @brief          : ÂË²¨´úÂë
  ******************************************************************************
  */

#include "filter.h"

/*------user code---*/
///@brief ÂË²¨Æ÷³õÊ¼»¯
void Filter_Init(ema_filter_t * filter_handle,float a)
{
	if(a>1 || a< 0)return;
	filter_handle->a = a;				//<ÖĞÖµÆ½¾ù
	filter_handle->data_last = 0;
}

/*---------------------------------*/
/* @brief Ò»½×µÍÍ¨ÂË²¨Æ÷
 * @param filter_handle ÂË²¨Æ÷¾ä±ú
 * @param now_data	ĞÂÊı¾İ
 */
float Filter_LPS(ema_filter_t * filter_handle,float now_data)
{
	float data = filter_handle->a* now_data + (1-filter_handle->a)* filter_handle->data_last;
	filter_handle->data_last = data;
	return data;
}



