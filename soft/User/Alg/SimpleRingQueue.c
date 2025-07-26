#include "SimpleRingQueue.h"
#include "stddef.h"





static int RingQueue_isEmpty(ring_queue_t * queue)
{
		return queue->len;
}

static void RingQueue_ProcessSide(ring_queue_t * queue)
{
	queue->write_index = queue->write_index%RING_QUEUE_LEN_MAX;
	queue->read_index  = queue->read_index%RING_QUEUE_LEN_MAX;
}

void RingQueue_Init(ring_queue_t * queue)
{
	queue->write_index = 0;
	queue->read_index = 0;
	queue->len =0;
}


void RingQueue_Write(ring_queue_t * queue ,char ch)
{
	if(queue == NULL){
		return;
	}
	if(queue->len < RING_QUEUE_LEN_MAX -1){
		queue->char_queue[queue->write_index ++] = ch;
		queue->len++;
		RingQueue_ProcessSide(queue);
	}
}


int RingQueue_Read(ring_queue_t * queue )
{
	int ch =-1;
	if(queue == NULL){
		return -1;
	}
	if(RingQueue_isEmpty(queue)){
		ch = queue->char_queue[queue->read_index ++];
		queue->len--;
		RingQueue_ProcessSide(queue);
		return ch;
	}
	return -1;
}




