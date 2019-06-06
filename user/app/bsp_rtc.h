
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-04     suozhang      the first version
 *
 */


#ifndef BSP_RTC_H
#define BSP_RTC_H

#include "time.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void init_rtc(time_t utc_time);
	
time_t get_rtc_utc_time( void );


#endif /* BSP_RTC_H */

