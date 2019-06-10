
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-10     suozhang      the first version
 *
 */
 
#ifndef _KFIFO_H_
#define _KFIFO_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

struct KFIFO
{   
    unsigned char *buffer;    /* the buffer holding the data */   
    unsigned int size;    		/* the size of the allocated buffer */   
    unsigned int in;    			/* data is added at offset (in % size) */   
    unsigned int out;    			/* data is extracted from off. (out % size) */
};

struct KFIFO *kfifo_alloc(unsigned int size);

unsigned int kfifo_put(struct KFIFO *fifo, unsigned char *buffer, unsigned int len);

unsigned int kfifo_get(struct KFIFO *fifo, unsigned char *buffer, unsigned int len);

unsigned int kfifo_get_data_len(struct KFIFO *fifo);

#endif /* _KFIFO_H_ */
