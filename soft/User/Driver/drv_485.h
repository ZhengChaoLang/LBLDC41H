#ifndef __DRV_485_H__
#define __DRV_485_H__

#include "main.h"

/** RS485通信配置宏 */
// 在drv_485.h中定义
#define RS485_TX_LEVEL  GPIO_PIN_SET  // 发送时DE置高
#define RS485_RX_LEVEL  GPIO_PIN_RESET // 接收时DE置低


#define RS485_UARTx         USART6            // 所用UART外设

// 切换RS485为接收模式
#define _RS485_DEPIN_READ()    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, RS485_RX_LEVEL)
// 切换RS485为发送模式
#define _RS485_DEPIN_Write()   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, RS485_TX_LEVEL)



#endif