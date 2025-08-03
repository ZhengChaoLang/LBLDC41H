#include "drv_hall.h"
#include "tim.h"
#include "rtthread.h"
#include "filter.h"

#define HALL1_TIM               htim2
#define HALL1_TIMONECNT_FREQ    (1000000)     // 1Mhz

ema_filter_t hall1_speed_filter;
DRV_HallSenSor_t hall_sor1;
rt_timer_t hall_timer;

void DrvHall_Config(DRV_HallSenSor_t* hall_sor);
void DrvHall_RunStep(DRV_HallSenSor_t* hall_sor);
void DrvHall_enable(DRV_HallSenSor_t* hall_sor);
void DrvHall_Disabile(DRV_HallSenSor_t* hall_sor);

void HALL_TIM_TimeOut(DRV_HallSenSor_t* hall_sor);
DRVHALL_SECTOR DrvHall_GetSector(DRV_HallSenSor_t* hall_sor);
/* ------------------------user prot ------------------------j'm'c---- */

int DrvHall_HwInit(void)
{
    // 外设初始化 or hw 初始化
    
    DrvHall_enable(&hall_sor1);
    
    // 相关变量
    Filter_Init(&hall1_speed_filter,0.8f);   
    hall_timer = rt_timer_create("hall_timer",(void(*)(void*))HALL_TIM_TimeOut,&hall_sor1,\
                                    100,RT_TIMER_FLAG_HARD_TIMER|RT_TIMER_FLAG_ONE_SHOT);
    return 0;
}
INIT_DEVICE_EXPORT(DrvHall_HwInit);


void DrvHall_enable(DRV_HallSenSor_t* hall_sor)
{
   if(hall_sor == &hall_sor1){
       __HAL_TIM_SET_COUNTER(&HALL1_TIM, 0);
      HAL_TIMEx_HallSensor_Start_IT(&HALL1_TIM);
        __HAL_TIM_ENABLE_IT(&HALL1_TIM,TIM_IT_UPDATE);
   }
}

void DrvHall_Disabile(DRV_HallSenSor_t* hall_sor)
{
   if(hall_sor == &hall_sor1){
      HAL_TIMEx_HallSensor_Start_IT(&HALL1_TIM);
   }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim==&HALL1_TIM)
	{
        DrvHall_RunStep(&hall_sor1);
        __HAL_TIM_SET_COUNTER(&HALL1_TIM, 0);
    }
    
}


void HALL_TIM_TimeOut(DRV_HallSenSor_t* hall_sor)
{
    if(hall_sor == &hall_sor1){
       
        hall_sor1.speed = 0;
    }
     
}

/*
 * @brief: 更新hall数据，放在硬件回调
 * @param[in]:
 * @return:
 */
void DrvHall_RunStep(DRV_HallSenSor_t* hall_sor)
{
    uint32_t ccr_vla =0;
    float speed =0;
    if(hall_sor == &hall_sor1 && hall_sor1.init_flag){
        ccr_vla = __HAL_TIM_GET_COMPARE(&HALL1_TIM, TIM_CHANNEL_1);
        speed = PI_DIV3/((float)ccr_vla/HALL1_TIMONECNT_FREQ);      //求速度 dr/dt 单位 rad/s
        hall_sor->speed = Filter_LPS(&hall1_speed_filter, speed);  
        hall_sor->posi = DrvHall_GetSector(hall_sor)*PI_DIV3;       //更新角度
        if(hall_timer != NULL){
            rt_timer_stop(hall_timer);
            rt_timer_start(hall_timer);
        }
        else
            rt_kprintf("hall_timer err \n");
        
    }    
}


/*
 * @brief: 获取当前的扇区
 * @param[in]:
 * @return:
 */
DRVHALL_SECTOR DrvHall_GetSector(DRV_HallSenSor_t* hall_sor){
    if(!hall_sor)return SECTOR_ERR;
    uint8_t sector = 0;
    uint8_t pin_val = 0;
    if(hall_sor == &hall_sor1){
        pin_val |= HAL_GPIO_ReadPin(HALL_U_GPIO_Port,HALL_U_Pin);
        pin_val |= (HAL_GPIO_ReadPin(HALL_V_GPIO_Port,HALL_V_Pin)<<1);
        pin_val |= (HAL_GPIO_ReadPin(HALL_W_GPIO_Port,HALL_W_Pin)<<2);
    }
    if(pin_val >=1 || pin_val <=6)
        sector = hall_sor->search_table[pin_val];
    return (DRVHALL_SECTOR)sector;       
}


//msh
void hall_test_thread(void* arg)
{
    while(1){
        rt_kprintf("%d,%d\n", (int)hall_sor1.speed,__HAL_TIM_GET_COUNTER(&htim2));
        rt_thread_mdelay(10);
    }
}
void COM_HALL_TEST()
{
    static rt_thread_t debug =NULL; 
    if(debug == NULL){
        debug =rt_thread_create( "debug_hall",
                                    hall_test_thread,
                                    NULL,
                                    256, 20, 2);
        rt_thread_startup(debug);
    }
    else{
        rt_thread_delete(debug);
        debug = NULL;
    }   
}
MSH_CMD_EXPORT(COM_HALL_TEST , printf test hall);



/*-------------------- Hall 基本不改动----------------------------*/
void DrvHall_Init(DRV_HallSenSor_t* hall_sor)
{   
    if(!hall_sor)return;
    
    uint8_t table60[6]  = DRV_HALL_ANGLE60_ORDER;
    uint8_t table120[6] = DRV_HALL_ANGLE120_ORDER;
    uint8_t*table_point = 0;
    switch(hall_sor->installation_angle)
    {
        case DRV_HALL_INSTALLATION_ANGLE60:
            table_point = table60;
            break;
        case DRV_HALL_INSTALLATION_ANGLE120:
            table_point = table120;
            break;
        default:
            table_point = table120;
            break;
    }  
    
    hall_sor->search_table[0] = 0;    
    for(int i=0; i<6; i++){
        hall_sor->order_table[i] =  table_point[i];
        hall_sor->search_table[table_point[i]] = i;
        hall_sor->search_table[0]++;        
    }
    //初始化 起始角度
    hall_sor->posi = DrvHall_GetSector(hall_sor)*PI_DIV3 + PI_DIV6;
    hall_sor->speed = 0;
    hall_sor->acc   = 0;
    hall_sor->init_flag = 1;//初始化
}




float DrvHall_GetSpeed(DRV_HallSenSor_t* hall_sor)
{    
    return hall_sor->speed;
}


float DrvHall_GetPosi(DRV_HallSenSor_t* hall_sor)
{
    return hall_sor->posi;             
}

float DrvHall_GetAcc(DRV_HallSenSor_t* hall_sor)
{
    return hall_sor->acc;        
}






