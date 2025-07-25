
/* RT-Thread config file */

#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

// <<< Use Configuration Wizard in Context Menu >>>
// <h>Basic Configuration
// <o>Maximal level of thread priority <8-256>
//  <i>Default: 32
#define RT_THREAD_PRIORITY_MAX 32

// <o>OS tick per second
//  <i>Default: 1000   (1ms)
#define RT_TICK_PER_SECOND 1000

// <o>Alignment size for CPU architecture data access
//  <i>Default: 4
#define RT_ALIGN_SIZE 4

// <o>the max length of object name<2-16>
//  <i>Default: 8
#define RT_NAME_MAX 8

// <c1>Using RT-Thread components initialization
//  <i>Using RT-Thread components initialization
#define RT_USING_COMPONENTS_INIT
// </c>

// <c1>Using user main
//  <i>Using user main
#define RT_USING_USER_MAIN
// </c>

// <o>the size of main thread<1-4086>
//  <i>Default: 512
#define RT_MAIN_THREAD_STACK_SIZE 1024
// </h>

// <h>Debug Configuration
// <c1>enable kernel debug configuration
//  <i>Default: enable kernel debug configuration
//#define RT_DEBUG
// </c>

// <o>enable components initialization debug configuration<0-1>
//  <i>Default: 0
#define RT_DEBUG_INIT 0

// <c1>thread stack over flow detect
//  <i> Diable Thread stack over flow detect
//#define RT_USING_OVERFLOW_CHECK
// </c>
// </h>

// <h>Hook Configuration
// <c1>using hook
//  <i>using hook
//#define RT_USING_HOOK
// </c>

// <c1>using idle hook
//  <i>using idle hook
//#define RT_HOOK_USING_FUNC_PTR
// </c>
// </h>

// <h>Software timers Configuration
// <c1> Enables user timers
// <i> Enables user timers
//#define RT_USING_TIMER_SOFT
// </c>

// <o>The priority level of timer thread <0-31>
//  <i>Default: 4
#define RT_TIMER_THREAD_PRIO 4

// <o>The stack size of timer thread <0-8192>
//  <i>Default: 512
#define RT_TIMER_THREAD_STACK_SIZE 512
// </h>

// <h>IPC(Inter-process communication) Configuration
// <c1>Using Semaphore
//  <i>Using Semaphore
#define RT_USING_SEMAPHORE
// </c>

// <c1>Using Mutex
//  <i>Using Mutex
#define RT_USING_MUTEX
// </c>

// <c1>Using Signal
//  <i>Using Signal
//#define RT_USING_SIGNALS
// </c>

// <c1>Using Event
//  <i>Using Event
#define RT_USING_EVENT
// </c>

// <c1>Using MailBox
//  <i>Using MailBox
#define RT_USING_MAILBOX
// </c>

// <c1>Using Message Queue
//  <i>Using Message Queue
#define RT_USING_MESSAGEQUEUE
// </c>
// </h>

// <h>Memory Management Configuration
// <c1>Using Mempool Management
//  <i>Using Mempool Management
//#define RT_USING_MEMPOOL
// </c>

// <c1>Dynamic Heap Management
//  <i>Dynamic Heap Management
#define RT_USING_HEAP
// </c>

// <c1>using small memory
//  <i>using small memory
#define RT_USING_SMALL_MEM
// </c>

// <c1>Small Memory Algorithm
//  <i>Small Memory Algorithm
#define RT_USING_SMALL_MEM_AS_HEAP
// </c>
// </h>

// <h>Console Configuration
// <c1>Using console
//  <i>Using console
#define RT_USING_CONSOLE
// </c>
// <o>the buffer size of console <1-1024>
//  <i>the buffer size of console
//  <i>Default: 128  (128Byte)
#define RT_CONSOLEBUF_SIZE 128
// </h>

// <h>Enable FinSH Configuration
// <c1>include shell config
//  <i> Select this choice if you using FinSH
#define RT_USING_FINSH
// </c>
// </h>

#if defined(RT_USING_FINSH)
    #include "finsh_config.h"
#endif

// <h>Device Configuration
// <c1>using device framework
//  <i>using device framework
//#define RT_USING_DEVICE
// </c>
// </h>

// <<< end of configuration section >>>

#endif

