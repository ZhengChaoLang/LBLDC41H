#include "drv_485.h"
#include "main.h"
#include "usart.h"
#include "rtthread.h"


// RS485 DMA收发宏定义
#define _RS485_SEND_DATA(tx_buf,size)   HAL_UART_Transmit_DMA(&huart6, tx_buf, size);  // DMA发送数据
#define _RS485_READ_DATA(rx_buf,size)   HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, size);  // DMA接收数据

// 缓冲区大小定义
#define RS485_RX_BUFSIZE 512  // 接收缓冲区大小
#define RS485_TX_BUFSIZE 512  // 发送缓冲区大小

// 收发缓冲区
uint8_t rs485_rx_buf[RS485_RX_BUFSIZE];  // 接收数据缓冲区
uint8_t rs485_tx_buf[RS485_TX_BUFSIZE];  // 发送数据缓冲区


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
     if(huart->Instance == RS485_UARTx)
     {
         
     }         
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
      void Drv_Rs485ErrFunc();
}

void Drv_Rs485ErrFunc()
{
     
}

void DrvRs485_Init()
{
//    rt_mq_create()
}
/**
 * @brief:
 * @param[in]:UART_PARITY_NONE
 * @return:
 */
void DrvRs485_ChangeBaudAndVerify(uint32_t baud, uint32_t verify)
{
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = RS485_UARTx;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = verify;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  
}


void DrvRs485_SendDateBuf(uint8_t *buff,uint16_t size)
{
     
}

void DrvRs485_RcvDate(void)
{
    
}


void DrvRs485_RcvProcess()
{
    
    
    
    
}



void DrvRs485_ThreadEnter()
{
    
    
    while(1){
        
        
        
        
        
    }
}






