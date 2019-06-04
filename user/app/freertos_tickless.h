
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-04     suozhang      the first version
 *
 */


#ifndef FREERTOS_TICKLESS_H
#define FREERTOS_TICKLESS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void vlps_init(void);
	
void init_lptmr_tick_interrupt( TickType_t xExpectedIdleTime );


#endif /* FREERTOS_TICKLESS_H */

