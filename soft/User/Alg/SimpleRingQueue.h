#ifndef __S_RINGQURUE_H__
#define __S_RINGQURUE_H__



#define RING_QUEUE_LEN_MAX 				256
typedef struct {
	char char_queue[RING_QUEUE_LEN_MAX];
	int write_index;
	int read_index;
	int len;
}ring_queue_t;



void RingQueue_Init(ring_queue_t * queue);
void RingQueue_Write(ring_queue_t * queue ,char ch);
int RingQueue_Read(ring_queue_t * queue );

#endif
