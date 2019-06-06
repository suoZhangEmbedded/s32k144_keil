
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

#include "bsp_lptmr.h"


// 低功耗定时器初始化 xExpectedIdleTime 定时唤醒，接口重定向，方便移植
#define port_low_power_time_tick( xExpectedIdleTime )   init_lptmr_tick( xExpectedIdleTime )

// 获取低功耗定时器是否发生中断，接口重定向，方便移植
#define port_get_low_power_time_interrupt_flag( void )  get_lpmrt_time_interrupt_flag( void )

// 获取低功耗定时器计数值，接口重定向，方便移植
#define port_get_low_power_time_counter( void ) 				get_lpmrt_counter( void )

/* tickless 模式控制, 非0即关闭tickless模式 */

#define tickless_MODE_ON													0x00000000UL  /* tickless 模式强制打开 */

#define tickless_BIT_EN														0x01000000UL  /* tickless 总开关 标志位 */
#define tickless_BIT_USEING_DEVICE_UART1					0x02000000UL  /* 串口1外设正在使用中 标志位 */
#define tickless_BIT_USEING_DEVICE_CAN1						0x04000000UL  /* CAN1外设正在使用中 标志位 */

#endif /* FREERTOS_TICKLESS_H */

