
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-04     suozhang      the first version
 *
 */
 
#include "bsp_rtc.h"

#include "device_registers.h"
#include "system_S32K144.h"
#include "stdbool.h"

#include "SEGGER_RTT.h"

/*******************************************************************************
Function: RTC_IRQHandler
*******************************************************************************/
void RTC_IRQHandler(void)
{

	RTC->TAR += 3; // Writing to the TAR clears the SR[TAF] Time Alarm Flag.
	
	// Next alarm in 3s
}

/*******************************************************************************
Function: RTC_Seconds_IRQHandler
*******************************************************************************/
void RTC_Seconds_IRQHandler(void)
{

}

/*******************************************************************************
Function: init_rtc
Notes   : 初始化 RTC ，utc_time: 当前 UTC 时间
				: 32kHz RTC clock derived from 128kHz internal LPO
*******************************************************************************/
void init_rtc(time_t utc_time)
{

    // Power-on cycle the MCU before writing to this register
	
		/* RTC 时钟源有2种: 1: RTC_CLK, 2: LPO1K_CLK */
		/* 这里选择 RTC_CLK,需要配置RTC_CLK时钟源，即寄存器 SIM->LPOCLKS[3-2] LPOCLKSEL,选择 LPO32K_CLK */
		/* 因此这里 LPO32k_CLK 输出需要使能 */

		/* 错误警告：LPO32k_CLK 和 LPO1k_CLK 不能同时使能，否则会选择 1K输出，请注意 */

    // [5-4] RTCCLKSEL = 1 choose LPO32k_CLK
    // [1] LPO32KCLKEN = 1 Enable LPO32k_CLK output
    // [0] LPO1KCLKEN =  0 Disable LPO1k_CLK  output
		SIM->LPOCLKS = SIM_LPOCLKS_RTCCLKSEL(1) | SIM_LPOCLKS_LPO32KCLKEN(1) | SIM_LPOCLKS_LPO1KCLKEN(0);

    // Peripheral Clock Control for RTC
		// 使能 RTC 外设的接口时钟，允许访问 RTC 模块
    PCC->PCCn[PCC_RTC_INDEX] = PCC_PCCn_CGC_MASK; // Enable clock to RTC module - BUS_CLK
	
	  RTC->SR = RTC_SR_TCE(0); // 失能秒计数器

    RTC->IER = RTC_IER_TAIE(0);
    // [18-16] TSIC = 0x000 1 Hz second interrupt，配置成1S一次中断
    // [4] TSIE = 0 秒中断关闭，Time Seconds Interrupt Disable
    // [2] TAIE = 0 闹钟中断关闭， Time alarm flag does generate an interrupt
    // [1] TOIE = 0 计数器溢出中断关闭，Time overflow flag does not generate an interrupt

    // Writing to RTC_TSR when the time counter is disabled will clear Time Invalid Flag
		// 时间秒寄存器 初始化
    RTC->TSR = utc_time;

    // RTC_TAR Alarm Register
		// 时间闹钟寄存器设置为3
    RTC->TAR = 3;    // Alarm in 3s

    // The prescaler output clock is output on RTC_CLKOUT pin
		// 时钟源选择 RTC_CLK，进行32K分频
    RTC->CR = RTC_CR_CPE(0);
    // [24] CPE = 0 RTC_CLKOUT is Disabled
    // [9] CLKO = 0 The 32 kHz clock is not output to other peripherals.
    // [7] LPOS = 0 RTC prescaler increments using 32kHz
    // [5] CPS =  0 The prescaler output clock (as configured by TSIC) is output on RTC_CLKOUT
		
		// 秒计数器使能
    RTC->SR = RTC_SR_TCE(1);
    // [4] TCE = 1 (Time Counter Enable)

}

/**
 * @ingroup get_rtc_utc_time
 * 获取 RTC UTC 时间
 * @return 当前 RTC UTC 时间
 */
time_t get_rtc_utc_time( void )
{
	
	return RTC->TSR;

}















