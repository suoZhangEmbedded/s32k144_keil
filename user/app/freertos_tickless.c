
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-04     suozhang      the first version
 *
 */
 
#include "freertos_tickless.h"

#include "SEGGER_RTT.h"

volatile uint32_t tickless_mode_now = tickless_MODE_ON; /* 初始化 tickless 模式 */

void tickless_bit_on( uint32_t tickless_bit )
{
	vTaskSuspendAll(); /* 有必要吗，因为是位控制，感觉没必要 */

		tickless_mode_now |= tickless_bit;
	
	xTaskResumeAll();

}

void tickless_bit_off( uint32_t tickless_bit )
{

	vTaskSuspendAll(); /* 有必要吗，因为是位控制，感觉没必要 */

		tickless_mode_now &= ~tickless_bit;
	
	xTaskResumeAll();

}

void application_sleep_enter_before( TickType_t xExpectedIdleTime )
{

	SEGGER_RTT_printf( 0, "application_sleep_enter_before:\r\n" );
	
}

void application_sleep_enter_later( TickType_t xExpectedIdleTime )
{

	SEGGER_RTT_printf( 0, "application_sleep_enter_later:\r\n" );
	
}


#ifndef configSYSTICK_CLOCK_HZ
	#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
	/* Ensure the SysTick is clocked at the same frequency as the core. */
	#define portNVIC_SYSTICK_CLK_BIT_SUOZHANG	( 1UL << 2UL )
#else
	/* The way the SysTick is clocked is not modified in case it is not the same
	as the core. */
	#define portNVIC_SYSTICK_CLK_BIT	( 0 )
#endif

#define portNVIC_SYSTICK_INT_BIT_SUOZHANG			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT_SUOZHANG			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT_SUOZHANG		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT_SUOZHANG 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT_SUOZHANG		( 1UL << 25UL )
/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 16 bit resolution of the Low Power Timer.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t xMaximumPossibleSuppressedTicks = 60000;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The number of SysTick increments that make up one tick period.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t ulTimerCountsForOneTick = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ );
#endif /* configUSE_TICKLESS_IDLE */

#define portNVIC_SYSTICK_CTRL_REG_SUOZHANG					( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG_SUOZHANG					( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG_SUOZHANG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG_SUOZHANG								( * ( ( volatile uint32_t * ) 0xe000ed20 ) )

#if( configUSE_TICKLESS_IDLE == 1 )

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
	uint32_t ulReloadValue, ulCompleteTickPeriods;

	TickType_t xModifiableIdleTime;
	
		/* 判定是否进入 tickless 模式 */
		if( tickless_mode_now ) 
		{
			// 终止进入 tickless 模式
			return;
		}

		/* Make sure the SysTick reload value does not overflow the counter. */
		if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
		{
			xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
		}

		/* Stop the SysTick momentarily.  The time the SysTick is stopped for
		is accounted for as best it can be, but using the tickless mode will
		inevitably result in some tiny drift of the time maintained by the
		kernel with respect to calendar time. */
		portNVIC_SYSTICK_CTRL_REG_SUOZHANG &= ~portNVIC_SYSTICK_ENABLE_BIT_SUOZHANG;

		/* Calculate the reload value required to wait xExpectedIdleTime
		tick periods.  -1 is used because this code will execute part way
		through one of the tick periods. */
		ulReloadValue = xExpectedIdleTime - 1UL;

		/* Enter a critical section but don't use the taskENTER_CRITICAL()
		method as that will mask interrupts that should exit sleep mode. */
		__disable_irq();
		__dsb( portSY_FULL_READ_WRITE );
		__isb( portSY_FULL_READ_WRITE );

		/* If a context switch is pending or a task is waiting for the scheduler
		to be unsuspended then abandon the low power entry. */
		if( eTaskConfirmSleepModeStatus() == eAbortSleep )
		{
			/* Restart from whatever is left in the count register to complete
			this tick period. */
			portNVIC_SYSTICK_LOAD_REG_SUOZHANG = portNVIC_SYSTICK_CURRENT_VALUE_REG_SUOZHANG;

			/* Restart SysTick. */
			portNVIC_SYSTICK_CTRL_REG_SUOZHANG |= portNVIC_SYSTICK_ENABLE_BIT_SUOZHANG;

			/* Reset the reload register to the value required for normal tick
			periods. */
			portNVIC_SYSTICK_LOAD_REG_SUOZHANG = ulTimerCountsForOneTick - 1UL;

			/* Re-enable interrupts - see comments above __disable_irq() call
			above. */
			__enable_irq();
		}
		else
		{
			/* Set low power timer the new reload value. */
			port_low_power_time_tick( ulReloadValue );

			/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
			set its parameter to 0 to indicate that its implementation contains
			its own wait for interrupt or wait for event instruction, and so wfi
			should not be executed again.  However, the original expected idle
			time variable must remain unmodified, so a copy is taken. */
			xModifiableIdleTime = xExpectedIdleTime;
			configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
			if( xModifiableIdleTime > 0 )
			{
				__dsb( portSY_FULL_READ_WRITE );
				STANDBY(); /* suozhang, s32k144 enter VLPS mode */
				__isb( portSY_FULL_READ_WRITE );
			}
			configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

			/* Re-enable interrupts to allow the interrupt that brought the MCU
			out of sleep mode to execute immediately.  see comments above
			__disable_interrupt() call above. */
			__enable_irq();
			__dsb( portSY_FULL_READ_WRITE );
			__isb( portSY_FULL_READ_WRITE );

			/* Disable interrupts again because the clock is about to be stopped
			and interrupts that execute while the clock is stopped will increase
			any slippage between the time maintained by the RTOS and calendar
			time. */
			__disable_irq();
			__dsb( portSY_FULL_READ_WRITE );
			__isb( portSY_FULL_READ_WRITE );

			if( get_lpmrt_time_interrupt_flag() )
			{/* 是低功耗定时器唤醒的CPU */

				SEGGER_RTT_printf( 0, "lptmr wake up.\r\n" );
				
				/* As the pending tick will be processed as soon as this
				function exits, the tick value maintained by the tick is stepped
				forward by one less than the time spent waiting. */
				ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
				
			}
			else
			{/* 其他中断唤醒的CPU */

				SEGGER_RTT_printf( 0, "not lptmr wake up.\r\n" );

				/* 读取 LPTMR Counter Register，用于 system tick 补偿 */
				/* 这里直接读取寄存器 会卡死，不知为啥 */
				ulCompleteTickPeriods = port_get_low_power_time_counter();
				
				// 这里做保护, 定时器计数如果大于任务挂起时间，FreeRTOS 会有断言卡死，因此做保护
				if( ulCompleteTickPeriods >= xExpectedIdleTime )
				{
					SEGGER_RTT_printf( 0, "ulCompleteTickPeriods :%d.\r\n", ulCompleteTickPeriods );
					
					ulCompleteTickPeriods = xExpectedIdleTime;
				}


			}

			/* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
			again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
			value. */
			portNVIC_SYSTICK_CURRENT_VALUE_REG_SUOZHANG = 0UL;
			portNVIC_SYSTICK_CTRL_REG_SUOZHANG |= portNVIC_SYSTICK_ENABLE_BIT_SUOZHANG;
			vTaskStepTick( ulCompleteTickPeriods );
			portNVIC_SYSTICK_LOAD_REG_SUOZHANG = ulTimerCountsForOneTick - 1UL;

			/* Exit with interrpts enabled. */
			__enable_irq();
		}
	}
	
#endif /* configUSE_TICKLESS_IDLE */


















