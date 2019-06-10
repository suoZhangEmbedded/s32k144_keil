
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-6-10     suozhang      the first version
 *
 */
 
#include "bsp_lpuart.h"

#include "device_registers.h"
#include "system_S32K144.h"
#include "stdbool.h"

#include "SEGGER_RTT.h"

#include "kfifo.h"

/* lpuatr1 接收环形缓冲区句柄 */
static struct KFIFO *lpuart1_rx_kfifo_handle = NULL;

void lpuatr1_send_char(uint8_t send)
{    /* Function to Transmit single Char */
  while((LPUART1->STAT & LPUART_STAT_TDRE_MASK)>>LPUART_STAT_TDRE_SHIFT == 0 );
                                   /* Wait for transmit buffer to be empty */
  LPUART1->DATA=send;              /* Send data */
}

void lpuatr1_send_buff( uint8_t *buff, uint32_t len )
{
	uint32_t i = 0;
	
	for( i=0; i<len; i++ )
	{
		lpuatr1_send_char( buff[i] );
	}
}

void LPUART1_RxTx_IRQHandler(void)
{
	uint8_t recive_ch = 0;
	
	uint32_t len = 0, i=0;
	
	//RDRF 接收数据寄存器满标志
	if (LPUART1->STAT & LPUART_STAT_RDRF_MASK )
	{
		recive_ch = LPUART1->DATA;

		if( lpuart1_rx_kfifo_handle )
			kfifo_put( lpuart1_rx_kfifo_handle, &recive_ch, 1 );
		
	}
	
	//IDLE 串口空闲标志位
	if (LPUART1->STAT & LPUART_STAT_IDLE_MASK )
	{
		/* 清除接收总线空闲中断标志位 */
		LPUART1->STAT |= LPUART_STAT_IDLE_MASK;
				
		if( lpuart1_rx_kfifo_handle )
		{
			len = kfifo_get_data_len( lpuart1_rx_kfifo_handle );
			
			SEGGER_RTT_printf( 0, "LPUART1 IRQHandler IDLE recive len:%d.\r\n", len );
			
			for( i=0; i<len; i++ )
			{
				kfifo_get( lpuart1_rx_kfifo_handle, &recive_ch, 1 );
				SEGGER_RTT_printf( 0, " 0x%X", recive_ch );
			}			
		}
	}
	
	//TDRE 发送数据寄存器空标志
	if( LPUART1->STAT & LPUART_STAT_TDRE_MASK ) 
	{
		LPUART1->CTRL &= ~LPUART_CTRL_TIE_MASK; // 关掉发送数据寄存器为空中断
	}
	
	//TC 传输完成标志
	if( LPUART1->STAT & LPUART_STAT_TC_MASK ) 
	{
		LPUART1->CTRL &= ~LPUART_CTRL_TCIE_MASK; // 关掉发送数据寄存器完成中断
		
	}
}

void lpuart1_port_init(void)
{
  PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;  /* Enable clock for PORTC */
  PORTC->PCR[6] |= PORT_PCR_MUX(2);           			/* Port C6: MUX = ALT2,UART1 TX */
  PORTC->PCR[7] |= PORT_PCR_MUX(2);           			/* Port C7: MUX = ALT2,UART1 RX */
}

void lpuart1_init(void)  /* Init. summary: 115200 baud, 1 stop bit, 8 bit format, 15 parity */
{
	/* 初始化接收环形缓冲区 */
	lpuart1_rx_kfifo_handle = kfifo_alloc( 256 );
	
	if( NULL == lpuart1_rx_kfifo_handle )
		while(1);
	
	lpuart1_port_init(); /* 引脚初始化 */
	
  PCC->PCCn[PCC_LPUART1_INDEX] &= ~PCC_PCCn_CGC_MASK;    /* Ensure clk disabled for config */
  PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_PCS(0x001)    /* Clock Src= 1 (SOSCDIV2_CLK) */
                               |  PCC_PCCn_CGC_MASK;     /* Enable clock for LPUART1 regs */

	/* Initialize for 115200 baud, 1 stop: */
  LPUART1->BAUD = LPUART_BAUD_SBR(5)  |  /* SBR=52 (0x34): baud divisor = 8M/115200/14 = ~5 */
									LPUART_BAUD_OSR(13) ;  /* OSR=14: Over sampling ratio = 13+1 =14 */  
																				 /* SBNS=0: One stop bit */
																				 /* BOTHEDGE=0: - Receiver samples input data using the rising edge 
																												of the baud rate clock. */
																				 /* M10=0: Rx and Tx use 7 to 9 bit data characters */
																				 /* RESYNCDIS=0: Resync during rec'd data word supported */
																				 /* LBKDIE, RXEDGIE=0: interrupts disable */
																				 /* TDMAE, RDMAE, TDMAE=0: DMA requests disabled */
																				 /* MAEN1, MAEN2,  MATCFG=0: Match disabled */

/* Enable transmitter & receiver, no parity, 8 bit char: */
  LPUART1->CTRL = LPUART_CTRL_RE(1)  | /* RE=1:   接收使能 */
									LPUART_CTRL_TE(1)  | /* TE=1:   发送使能 */
									LPUART_CTRL_RIE(1) | /* RIE=1:  接收中断使能 */
									LPUART_CTRL_TCIE(1)| /* TCIE=1: 传输完成中断使能 */
									LPUART_CTRL_TIE(0) | /* TIE=0:  发送寄存器为空中断失能 */
									LPUART_CTRL_ILT(1) | /* ILT=1:  Idle char bit count starts after STOP bit */
									LPUART_CTRL_ILIE(1); /* ILIE=1: 总线空闲使能 */	
																			 /* PE,PT=0: No hw parity generation or checking */
																			 /* M7,M,R8T9,R9T8=0: 8-bit data characters*/
																			 /* DOZEEN=0: LPUART enabled in Doze mode */
																			 /* ORIE,NEIE,FEIE,PEIE,MA1IE,MA2IE=0: no IRQ*/
																			 /* TxDIR=0: TxD pin is input if in single-wire mode */
																			 /* TXINV=0: TRansmit data not inverted */
																			 /* RWU,WAKE=0: normal operation; rcvr not in statndby */
																			 /* IDLCFG=0: one idle character */
																			 /* SBK=0: Normal transmitter operation - no break char */
																			 /* LOOPS,RSRC=0: no loop back */
																			
		// LPUART1_Interrupt
    S32_NVIC->ICPR[1] = (1 << (LPUART1_RxTx_IRQn % 32));
    S32_NVIC->ISER[1] = (1 << (LPUART1_RxTx_IRQn % 32));
    S32_NVIC->IP[LPUART1_RxTx_IRQn]  = 15; // Priority level 15
		
}


















