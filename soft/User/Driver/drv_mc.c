#include "drv_mc.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "main.h"
#include "app_foc.h"
#include "rtthread.h"
typedef void (*ad_iqr_hook_t)(void* param);
volatile uint16_t adc_current_val[3];    //电流 adc原始采样值ֵ 
 int debug_adc=0;
#define DRV_ADC_HOOK_NUB    10
//adc回调HOOK结构体
typedef struct{
    struct{
        ad_iqr_hook_t func;
        const char* name;
        void *param;
    }hook[DRV_ADC_HOOK_NUB];
}drv_adc_t;

drv_adc_t drv_adc1={0};

//
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
       i++;
   }       
}
//设置
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


//adc回调函数hook
void DrvAdc_IqrHook(drv_adc_t * adcx){
   uint8_t i=0;
   while(i < DRV_ADC_HOOK_NUB){
       if(adcx->hook[i].func !=NULL){
           adcx->hook[i].func(adcx->hook[i].param);           
       }
       i++;
   }
    debug_adc++;   
}

//电压转换电流
float Adc_SempVoltCurrent(float vlot){    
    return vlot;
}


void AdcMc_CurrentUpdata(foc_motor_t *motor, uint16_t*adc_val)
{    
	motor->phase_i.a = Adc_SempVoltCurrent(adc_val[0]*3.3f/4095.0f);
	motor->phase_i.b = Adc_SempVoltCurrent(adc_val[1]*3.3f/4095.0f);
	motor->phase_i.c = Adc_SempVoltCurrent(adc_val[2]*3.3f/4095.0f);   
}

//回调函数：三相电流值
void Adc_CurrentUpdate_Hook(void* arg){
    extern foc_motor_t m1; 
    AdcMc_CurrentUpdata(&m1, (uint16_t *)arg);
}


//adc 注入转换结束回调
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
	adc_current_val[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	adc_current_val[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	adc_current_val[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3); 
    DrvAdc_IqrHook(&drv_adc1); 
}

// 使能电机驱动
void DrvMc_EnablePwm(void *param){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    DRV_EN_PIN(DRV_EN);
}

// 失能电机驱动
void DrvMc_DisabilityPwm(void *param){
    DRV_EN_PIN(DRV_DE);  
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
      
}
// 
int DrvMc_Init(){
	HAL_ADCEx_InjectedStart_IT(&hadc1);    
    DrvMc_EnablePwm(NULL);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MC_TIM_ARR-5);
    DrvAdc_SetIqrHook(&drv_adc1, Adc_CurrentUpdate_Hook, "CurUpdate");
    DrvAdc_SetIqrHook(&drv_adc1, AppFoc_RunStep, "Foc_Run");
	DrvAdc_SetHookParam(&drv_adc1, "CurUpdate", (void*)adc_current_val);
    extern foc_motor_t m1; 
    DrvAdc_SetHookParam(&drv_adc1, "Foc_Run",(void*)&m1);
    return 0;
}
INIT_DEVICE_EXPORT(DrvMc_Init);


//msh打印三相电流
void ad_debug_thread(void* arg)
{
    while(1){
        rt_kprintf("%d,%d,%d\n", adc_current_val[0], adc_current_val[1],adc_current_val[2]);
        rt_thread_mdelay(10);
    }
}
void COM_DrvDebug()
{
    static rt_thread_t debug_ad =NULL; 
    if(debug_ad == NULL){
        debug_ad =rt_thread_create( "debug_ad",
                                    ad_debug_thread,
                                    NULL,
                                    256, 20, 2);
        rt_thread_startup(debug_ad);
    }
    else{
        rt_thread_delete(debug_ad);
        debug_ad = NULL;
    }   
}
MSH_CMD_EXPORT(COM_DrvDebug , printf ad run);
