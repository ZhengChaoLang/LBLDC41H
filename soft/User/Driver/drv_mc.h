#ifndef __FOC_TASK_H_
#define __FOC_TASK_H_

#include "main.h"


#define DRV_EN  GPIO_PIN_SET
#define DRV_DE  GPIO_PIN_RESET
#define DRV_EN_PIN(state)     HAL_GPIO_WritePin(CLR_L_HD_GPIO_Port, CLR_L_HD_Pin, state)        //


int DrvMc_Init(void);


#endif
