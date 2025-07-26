/***************************
*
*队列
*****************************/
#include "SimpleRingQueue.h"
#include "rtthread.h"
#include "usart.h"
#include "app_finsh_thread.h"


#define UARTR_MSH_BUFF_MAXSZIE		256    
char uart_rxbuff[UARTR_MSH_BUFF_MAXSZIE];
#define MSH_RX_FUNC(BUFF) 			        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)BUFF, UARTR_MSH_BUFF_MAXSZIE);



rt_mailbox_t msh_rx_buff_mb;
ring_queue_t  r_queue;

rt_thread_t msh_rx_thread;


/*
 * @brief:  创建msh环形缓存区处理线程
 * @param[in]:
 * @return:
 */
void msh_rxbuff_it_thread_init(void){
	
	msh_rx_thread = rt_thread_create("msh_rx", msh_rxbuff_it_thread, RT_NULL, 256, 25, 2);
	if(!msh_rx_thread){
		rt_kprintf("msh_rx_thread err...\r\n");
	}
	rt_thread_startup(msh_rx_thread);	
}
INIT_APP_EXPORT(msh_rxbuff_it_thread_init);

void msh_rxbuff_it_thread(void *arg){
	
	RingQueue_Init(&r_queue);
	int size =0;
	//开中断DMA 串口
	msh_rx_buff_mb = rt_mb_create("msh_rx", 4, RT_IPC_FLAG_FIFO);
	while(!msh_rx_buff_mb){
		rt_kprintf("msh_rx_buff_updeat creat err....\r\n");
		rt_thread_mdelay(1000);
		msh_rx_buff_mb = rt_mb_create("msh_rx", 0, RT_IPC_FLAG_FIFO);
	}
	MSH_RX_FUNC(uart_rxbuff);
	while(1){
		
		rt_mb_recv(msh_rx_buff_mb, (rt_ubase_t *)&size, RT_WAITING_FOREVER);
		if(size <UARTR_MSH_BUFF_MAXSZIE){
			for(int i =0;i < size; i++){		
				RingQueue_Write(&r_queue ,uart_rxbuff[i]);
			}
		}
		
		MSH_RX_FUNC(uart_rxbuff);
	}	
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	
	if(huart->Instance == USART1){
		rt_mb_send(msh_rx_buff_mb, Size);
	}
}

//#include "app_finsh_thread.h"
//char rt_hw_console_getchar(void)
//{
//    /* Note: the initial value of ch must < 0 */
//    int ch = -1;

////    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
////    {
////        ch = UartHandle.Instance->DR & 0xff;
////    }
//		ch = RingQueue_Read(&r_queue);
//		
//	
//    if(ch == -1)
//    {
//        rt_thread_mdelay(10);
//    }
//    return ch;
//}
