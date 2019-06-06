
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-04     suozhang      the first version
 *
 */


#ifndef BSP_LPTMR_H
#define BSP_LPTMR_H

#include "stdint.h"

#include "device_registers.h"
#include "system_S32K144.h"
#include "stdbool.h"

void init_lptmr_tick( uint32_t xExpectedIdleTime );
uint32_t get_lpmrt_counter( void );
bool get_lpmrt_time_interrupt_flag( void );
	
#endif /* BSP_LPTMR_H */

