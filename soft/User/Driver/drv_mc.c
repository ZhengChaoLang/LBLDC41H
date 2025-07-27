#include "drv_mc.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "main.h"
#include "app_foc.h"
#include "rtthread.h"
typedef void (*ad_iqr_hook_t)(void* param);
uint16_t adc_current_val[3];    //电压采集原始值               
#define DRV_ADC_HOOK_NUB    10
//adc中断回调钩子结构体
typedef struct{
    struct{
        ad_iqr_hook_t func;
        const char* name;
        void *param;
    }hook[DRV_ADC_HOOK_NUB];
}drv_adc_t;

drv_adc_t drv_adc1={0};

//设置adc回调函数
void DrvAdc_SetIqrHook(drv_adc_t * adcx, ad_iqr_hook_t func,const char* func_name){
   uint8_t i=0;
   if((func == NULL)||(func_name == NULL))
       return;
   while(i < DRV_ADC_HOOK_NUB){
       if(adcx->hook[i].func ==NULL){
           adcx->hook[i].func = func;
           adcx->hook[i].name = func_name;
           return;
       }
   }       
}
//设置钩子函数参数
uint8_t DrvAdc_SetHookParam(drv_adc_t * adcx,const char* func_name,void * param){
   
   if(adcx == NULL)
       return 1;
   for(uint8_t i=0; i < DRV_ADC_HOOK_NUB; i++){
        if(rt_strcmp(func_name, adcx->hook[i].name) == 0){
            adcx->hook[i].param =  param;
            return 0;    
        }                  
    }  
    return 1;
}


//调用hook
void DrvAdc_IqrHook(drv_adc_t * adcx){
   uint8_t i=0;
   while(i < DRV_ADC_HOOK_NUB){
       if(adcx->hook[i].func !=NULL){
           adcx->hook[i].func(adcx->hook[i].param);           
       }
   }       
}

float Adc_SempVoltCurrent(float vlot){    
    return vlot;
}


void AdcMc_CurrentUpdata(foc_motor_t *motor, uint16_t*adc_val)
{    
	motor->phase_i.a = Adc_SempVoltCurrent(adc_val[0]*3.3f/4095.0f);
	motor->phase_i.b = Adc_SempVoltCurrent(adc_val[1]*3.3f/4095.0f);
	motor->phase_i.c = Adc_SempVoltCurrent(adc_val[2]*3.3f/4095.0f);   
}

//电流采样更新回调函数
void Adc_CurrentUpdate_Hook(void* arg){
    extern foc_motor_t m1; 
    AdcMc_CurrentUpdata(&m1, (uint16_t *)arg);
}


//adc 注入中断回调
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
	adc_current_val[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	adc_current_val[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	adc_current_val[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3); 
    DrvAdc_IqrHook(&drv_adc1);
}

//
int DrvMc_Init(){
	HAL_ADCEx_InjectedStart_IT(&hadc1);
    DrvAdc_SetIqrHook(&drv_adc1, Adc_CurrentUpdate_Hook, "CurUpdate");
    DrvAdc_SetIqrHook(&drv_adc1, AppFoc_RunStep, "Foc_Run");
	DrvAdc_SetHookParam(&drv_adc1, "CurUpdate", (void*)adc_current_val);
    extern foc_motor_t m1; 
    DrvAdc_SetHookParam(&drv_adc1, "Foc_Run",(void*)&m1);
}
//INIT_DEVICE_EXPORT(DrvMc_Init);

