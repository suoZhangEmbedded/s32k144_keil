
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-04     suozhang      the first version
 *
 */
 
#include "bsp_lptmr.h"

#include "SEGGER_RTT.h"

// 用于提前保存CSR寄存器中断状态标志位，否则在中断函数中进行中断失能，标志位会被清除，不知为啥
// 目的是判断 定时中断是否产生
static volatile uint32_t lptmr_csr = 0;

/*******************************************************************************
Function: LPTMR0_IRQHandler
*******************************************************************************/
void LPTMR0_IRQHandler(void)
{

	// 使能 LPTMR 外设的接口时钟，允许访问 LPTMR 模块
	PCC->PCCn[PCC_LPTMR0_INDEX] = PCC_PCCn_CGC_MASK; // Enable clock to LPTMR module - BUS_CLK

	// 提前保存状态标志位，否则下面中断失能，标志位会被清除，不知为啥
	lptmr_csr = LPTMR0->CSR;
	
		SEGGER_RTT_printf( 0, "LPTMR0->CNR:%u.\r\n", LPTMR0->CNR );
		SEGGER_RTT_printf( 0, "LPTMR0->CSR:%X.\r\n", LPTMR0->CSR );

	// 定时器中断失能
	LPTMR0->CSR &= ~LPTMR_CSR_TIE(0);

}

/*******************************************************************************
Function: init_lptmr_tick
Notes   : 32kHz RTC clock derived from 128kHz internal LPO
        : Time counter mode, interrupt enabled
				: 1 tick == 1ms
*******************************************************************************/
void init_lptmr_tick( uint32_t xExpectedIdleTime )
{
		SEGGER_RTT_printf( 0, "init_lptmr_tick_interrupt xExpectedIdleTime tick:%u.\r\n", xExpectedIdleTime );
	
    // Peripheral Clock Control for LPTMR
		// 使能 LPTMR 外设的接口时钟，允许访问 LPTMR 模块
    PCC->PCCn[PCC_LPTMR0_INDEX] = PCC_PCCn_CGC_MASK; // Enable clock to LPTMR module - BUS_CLK

		LPTMR0->CNR |= 0XFFFF0000; 
	
		lptmr_csr = 0; //清0
	
		// [6] TIE  = 1 enable time interrupt
    // [2] TFC  = 1 CNR is reset on overflow.
    // [1] TMS  = 0 时间计数器模式
    // [0] TEN  = 0 LPTMR 定时器关闭
    LPTMR0->CSR = LPTMR_CSR_TFC(1) | LPTMR_CSR_TIE(1);
	
		/* LPTMR 时钟源有4种: 1: SIRCDIV2_CLK, 2: LPO1K_CLK, 3: RTC_CLK, 4: BUS_CLK */
		/* FreeRTOS 是 1ms 一个 tick, 因此需要配置 LPTMR 产生 单位是 1ms 的中断 */
		/* S32K144 VLPS模式下，只能选择 LPO1K_CLK, 或者 RTC_CLK  */
		/* 这里选择 RTC_CLK,需要配置RTC_CLK时钟源，即寄存器 SIM->LPOCLKS[3-2] LPOCLKSEL,选择 LPO32K_CLK */
		/* 因此这里 LPO32k_CLK 输出需要使能 */
	
		/* 错误警告：这里配置了 RTC 的时钟源，会影响RTC 的时钟，两者可能会冲突，请注意 */
		/* 错误警告：LPO32k_CLK 和 LPO1k_CLK 不能同时使能，否则会选择 1K输出，请注意 */
	
		/* 第二种时钟方案：预分频器还可以旁路掉，也就是说 时钟源直接到 低功耗定时器计数寄存器，因此可以选择 LPO1K_CLK */
	
    // [5-4] RTCCLKSEL = 1 choose LPO32k_CLK
    // [1] LPO32KCLKEN = 1 Enable LPO32k_CLK output
    // [0] LPO1KCLKEN =  0 Disable LPO1k_CLK  output
		SIM->LPOCLKS = SIM_LPOCLKS_RTCCLKSEL(1) | SIM_LPOCLKS_LPO32KCLKEN(1) | SIM_LPOCLKS_LPO1KCLKEN(0);

    // LPO32K_CLK 时钟 到预分频器进行32分频，每16个上升沿，计数寄存器+1
		// 这样 计数寄存器 ＋1 就是 1ms
		// [6-3] PRESCALE  = 4 选择 32 分频
    // [2] 	 PBYP      = 0 Enable 预分频器
    // [1-0] PCS       = 2 根据手册2018.9.9 table 27-9 , 这里选择RTC_CLK
		LPTMR0->PSR = LPTMR_PSR_PRESCALE(4) | LPTMR_PSR_PBYP(0) | LPTMR_PSR_PCS(2);
		
    // 设定低功耗定时器比较寄存器
    LPTMR0->CMR = xExpectedIdleTime; //定时 xExpectedIdleTime 个 tick 后 产生中断 
		
		// LPTMR0_Interrupt
    S32_NVIC->ICPR[1] = (1 << (LPTMR0_IRQn % 32));
    S32_NVIC->ISER[1] = (1 << (LPTMR0_IRQn % 32));
    S32_NVIC->IP[LPTMR0_IRQn]  = 15;  // Priority level 15

    // [0] TEN  = 1 LPTMR 启动定时器
	  LPTMR0->CSR |= LPTMR_CSR_TEN(1);

}

uint32_t get_lpmrt_counter( void )
{
	return LPTMR0->CNR;
}

// 返回定时器是否产生中断
// true: 发生中断
// false: 未发生中断
bool get_lpmrt_time_interrupt_flag( void )
{
	return (lptmr_csr & LPTMR_CSR_TCF_MASK);
}





