#ifndef __DRV_HALL_H__
#define __DRV_HALL_H__

#include  <stdint.h>

#ifndef PI_DIV3
    #define PI_DIV3 (3.14159265f/3)
#endif
#define PI_DIV6 PI_DIV3/2

/**
 * @brief 霍尔传感器扇区枚举定义
 * 对应电机换相的6个有效状态，每个扇区对应特定的绕组导通组合。
 */
typedef enum{
    SECTOR_ERR,   // 错误扇区（无效霍尔状态），用于表示霍尔信号异常或未检测到有效状态
    SECTOR1,      // 第1扇区，对应霍尔传感器的第一个有效状态组合
    SECTOR2,      // 第2扇区，对应霍尔传感器的第二个有效状态组合
    SECTOR3,      // 第3扇区，对应霍尔传感器的第三个有效状态组合
    SECTOR4,      // 第4扇区，对应霍尔传感器的第四个有效状态组合
    SECTOR5,      // 第5扇区，对应霍尔传感器的第五个有效状态组合
    SECTOR6       // 第6扇区，对应霍尔传感器的第六个有效状态组合
}DRVHALL_SECTOR;

typedef struct{
    uint8_t init_flag;
    uint8_t installation_angle;
    uint8_t order_table[6];
    uint8_t search_table[7];
    
    float posi;
    float speed;
    float acc;
}DRV_HallSenSor_t;

 
#define DRV_HALL_INSTALLATION_ANGLE60       0
#define DRV_HALL_INSTALLATION_ANGLE120      1

/* @    
 * @{
 */
#define DRV_HALL_ANGLE120_ORDER {4,6,2,3,1,5}
#define DRV_HALL_ANGLE60_ORDER  {5,4,6,2,3,1}   
/* @}
 */    



void DrvHall_Init(DRV_HallSenSor_t* hall_sor);


extern DRV_HallSenSor_t hall_sor1;

#endif
