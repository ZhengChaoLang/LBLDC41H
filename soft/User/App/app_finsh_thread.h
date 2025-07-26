#ifndef __FINSH_THREAD_H__
#define __FINSH_THREAD_H__


#include "stddef.h"
#include "SimpleRingQueue.h"
     

void msh_rxbuff_it_thread_init(void);
void msh_rxbuff_it_thread(void *arg);



#endif
