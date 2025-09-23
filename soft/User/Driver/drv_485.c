#include "drv_485.h"
#include "main.h"
#include "usart.h"
#include "rtthread.h"


// RS485 DMA收发宏定义
#define _RS485_SEND_DATA(tx_buf,size)   HAL_UART_Transmit_DMA(&huart6, tx_buf, size);  // DMA发送数据
#define _RS485_READ_DATA(rx_buf,size)   HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, size);  // DMA接收数据

// 缓冲区大小定义
#define RS485_DATA_BUFSIZE 125  // 接收缓冲区大小
//#define RS485_DATA_BUFSIZE 125  // 发送缓冲区大小
#define RS485_BUF_SIZE   4      // 缓冲区个数

typedef struct{
    uint8_t buf[RS485_DATA_BUFSIZE];  //
    uint16_t size;
}rs485_buf_data_t;

typedef struct{
    rs485_buf_data_t rx_buf[RS485_BUF_SIZE];  // 接收数据缓冲区
    rs485_buf_data_t tx_buf[RS485_BUF_SIZE];  // 发送数据缓冲区
    uint8_t new_rx_index;
    uint8_t new_tx_index;
}rs485_buf_t;
rs485_buf_t rs485_buf;


#define RS485_EVENT_DE  ((rt_uint32_t)0x1)
#define RS485_EVENT_ERR  ((rt_uint32_t)0x2)
rt_mailbox_t rs485_mb_rx;
rt_mailbox_t rs485_mb_tx;
rt_event_t rs485_event_de;

// RS485错误处理：发送错误事件
void Drv_Rs485ErrFunc(void);

// RS485初始化：创建邮箱、事件集
int DrvRs485_Init(void);

// 修改RS485波特率与校验方式（baud：目标波特率；verify：校验方式，如UART_PARITY_NONE）
void DrvRs485_ChangeBaudAndVerify(uint32_t baud, uint32_t verify);

// 发送RS485数据：数据存入发送缓冲区并加入邮箱队列（buf：待发数据指针；size：待发数据长度）
void DrvRs485_SendDataBuf(uint8_t* buf, uint16_t size);

// RS485发送处理线程：循环从邮箱取数据并执行发送
void DrvRs485_TxProcess(void *arg);

// RS485发送完成回调：发送DE引脚控制事件
void DrvRs485_TcCallBack(void);

// RS485接收处理：存储接收数据、更新缓冲区并通知后续处理（size：接收数据长度）
void DrvRs485_RxProcess(uint16_t size);

// RS485 DE引脚控制线程：处理发送完成/错误事件，切换DE引脚状态
void DrvRs485_DeThread(void *arg);
void CMD_DEBUG_DrvRs485(void *arg);
//在app_finsh_thread.c
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//     if(huart->Instance == RS485_UARTx)
//     {
//         DrvRs485_RxProcess(Size);
//     }         
//}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
     if(huart->Instance == RS485_UARTx)
     {
        DrvRs485_TcCallBack();
     }     
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
     if(huart->Instance == RS485_UARTx){
        Drv_Rs485ErrFunc();
     }
}

void Drv_Rs485ErrFunc()
{
     rt_event_send(rs485_event_de, RS485_EVENT_ERR);
}

rt_thread_t thread_485send;
rt_thread_t thread_485de;
rt_thread_t thread_485Cmd;
int DrvRs485_Init()
{
    rs485_mb_rx = rt_mb_create("rs485_rx", RS485_BUF_SIZE, RT_IPC_FLAG_FIFO);
    rs485_mb_tx = rt_mb_create("rs485_tx", RS485_BUF_SIZE, RT_IPC_FLAG_FIFO);
    rs485_event_de = rt_event_create("rs485_de", RT_IPC_FLAG_PRIO);

    
    rs485_buf.new_rx_index = 0;  
    rs485_buf.new_tx_index = 0;    
    for(uint16_t i = 0; i < RS485_BUF_SIZE; i++)
    {   
        rt_memset(rs485_buf.rx_buf[i].buf, 0,RS485_DATA_BUFSIZE);
        rt_memset(rs485_buf.tx_buf[i].buf, 0,RS485_DATA_BUFSIZE);
        rs485_buf.rx_buf[i].size = 0;
        rs485_buf.tx_buf[i].size = 0;
    }
    thread_485send = rt_thread_create("485_SEND_TASK", DrvRs485_TxProcess, NULL, 512,25, 1);
    //thread_485recv = rt_thread_create("485_RECV_TASK", DrvRs485_RxProcess, NULL, 256,25, 1);  
    thread_485de = rt_thread_create("485_DE_TASK", DrvRs485_DeThread, NULL, 512,25, 1); 
    thread_485Cmd= rt_thread_create("485_CMD_TASK", CMD_DEBUG_DrvRs485, NULL, 512,25, 1);
    
    if(thread_485send&&thread_485de){
        rt_thread_startup(thread_485send);  
        rt_thread_startup(thread_485de);  
        rt_thread_startup(thread_485Cmd);
    } 
  
  
}
INIT_APP_EXPORT(DrvRs485_Init);
/**
 * @brief:
 * @param[in]:UART_PARITY_NONE
 * @return:
 */
void DrvRs485_ChangeBaudAndVerify(uint32_t baud, uint32_t verify)
{
  /* USER CODE END USART6_Init 1 */
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
void DrvRs485_SendDataBuf(uint8_t* buf, uint16_t size)
{
    uint8_t *send_buf = NULL;
    if(size > RS485_DATA_BUFSIZE) size = RS485_DATA_BUFSIZE;
    
    send_buf = rs485_buf.tx_buf[rs485_buf.new_tx_index].buf;
    rs485_buf.tx_buf[rs485_buf.new_tx_index].size = size;
    rt_memcpy(send_buf, buf, size);
    
    rt_mb_send(rs485_mb_tx,(rt_ubase_t)&(rs485_buf.tx_buf[rs485_buf.new_tx_index]));
    
    rs485_buf.new_tx_index++;
    rs485_buf.new_tx_index %= RS485_BUF_SIZE;
}

/**
 * @brief:
 * @param[in]:
 * @return:
 */
void DrvRs485_TxProcess(void *arg)
{
    
    rs485_buf_data_t *tx_buf =NULL;
    
    _RS485_READ_DATA(rs485_buf.rx_buf[0].buf, RS485_DATA_BUFSIZE);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart6.hdmatx, DMA_IT_HT);
    while(1){    
        rt_mb_recv(rs485_mb_tx, (rt_ubase_t*)&tx_buf, RT_WAITING_FOREVER); 
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, RS485_TX_LEVEL);
        rt_thread_mdelay(1);    //485发送间隔 modbus   
        _RS485_SEND_DATA(tx_buf->buf, tx_buf->size);
        //rt_thread_mdelay(2);    //485发送间隔 modbus     
    }
}

void DrvRs485_TcCallBack()
{
    rt_event_send(rs485_event_de , RS485_EVENT_DE);  
}

/**
 * @brief: RS485接收数据处理函数
 *         用于处理接收到的RS485数据，将数据存入接收缓冲区并通过邮箱通知，
 *         同时更新缓冲区索引并准备下一次接收
 * @param[in]: size - 接收到的数据长度
 * @return: 无
 */
void DrvRs485_RxProcess(uint16_t size)
{
    //uint8_t* rx_buf = rs485_buf.rx_buf[rs485_buf.new_rx_index].buf;  
    // 记录当前接收缓冲区的数据长度
    rs485_buf.rx_buf[rs485_buf.new_rx_index].size = size;
   
    // 将接收缓冲区地址通过邮箱发送，通知其他模块处理数据
    rt_mb_send(rs485_mb_rx, (rt_ubase_t)(&rs485_buf.rx_buf[rs485_buf.new_rx_index]));//放入数据队列    
    // 变更缓存位置（循环更新索引）
    rs485_buf.new_rx_index ++;
    rs485_buf.new_rx_index %= RS485_BUF_SIZE;
    // 准备下一次接收，启动读取数据到新的缓冲区
    _RS485_READ_DATA(rs485_buf.rx_buf[rs485_buf.new_rx_index].buf , RS485_DATA_BUFSIZE);
}


void DrvRs485_DeThread(void *arg)
{
    rt_uint32_t event_type = 0;
    while(1){
       rt_event_recv(rs485_event_de, RS485_EVENT_DE|RS485_EVENT_ERR, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &event_type);
       if(event_type == RS485_EVENT_DE){
            _RS485_DEPIN_READ();
       }
       else if(event_type&RS485_EVENT_ERR){
           rt_thread_mdelay(1);
           
            _RS485_DEPIN_READ();
           HAL_UART_Abort(&huart6);
           __HAL_UART_CLEAR_PEFLAG(&huart6);
           __HAL_UART_CLEAR_FEFLAG(&huart6);
           _RS485_READ_DATA(rs485_buf.rx_buf[rs485_buf.new_rx_index].buf, RS485_DATA_BUFSIZE);
           //复位串口
           
       }
    }
}

void CMD_DEBUG_DrvRs485(void * arg)
{
    rs485_buf_data_t * rx_buf = NULL; 
    uint8_t *data;
    uint8_t size;
    while(1){
        rt_mb_recv(rs485_mb_rx, (rt_ubase_t*)&rx_buf, RT_WAITING_FOREVER);
        data = rx_buf->buf;
        size = rx_buf->size;
        DrvRs485_SendDataBuf( data, size);
    }
}
    






