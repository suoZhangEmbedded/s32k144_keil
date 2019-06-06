
/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-5-21     suozhang      the first version
 *
 */
 
#include "device_registers.h"
#include "system_S32K144.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "SEGGER_RTT.h"

#include "freertos_tickless.h"

#include "bsp_vlps.h"

#include "bsp_rtc.h"

void vLedTask( void *pvParameters );

#define  LPTMR_COUNTER  ( * ( ( volatile uint32_t * ) 0x4004000Cu ) )

/*******************************************************************************
Function: delay
*******************************************************************************/
void delay(uint32_t d)
{
	while(d--);
}

void sosc_8mhz_init(void)
{
  SCG->SOSCDIV=0x00000101;  /* SOSCDIV1 & SOSCDIV2 =1: divide by 1 */
  SCG->SOSCCFG=0x00000024;  /* Range=2: Medium freq (SOSC betw 1MHz-8MHz)*/
                            /* HGO=0:   Config xtal osc for low power */
                            /* EREFS=1: Input is external XTAL */
  while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK); /* Ensure SOSCCSR unlocked */
  SCG->SOSCCSR=0x00000001;  /* LK=0:          SOSCCSR can be written */
                            /* SOSCCMRE=0:    OSC CLK monitor IRQ if enabled */
                            /* SOSCCM=0:      OSC CLK monitor disabled */
                            /* SOSCERCLKEN=0: Sys OSC 3V ERCLK output clk disabled */
                            /* SOSCLPEN=0:    Sys OSC disabled in VLP modes */
                            /* SOSCSTEN=0:    Sys OSC disabled in Stop modes */
                            /* SOSCEN=1:      Enable oscillator */
  while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)); /* Wait for sys OSC clk valid */
}

void spp_160mhz_init(void)
{
  while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); /* Ensure SPLLCSR unlocked */
  SCG->SPLLCSR = 0x00000000;  /* SPLLEN=0: SPLL is disabled (default) */
  SCG->SPLLDIV = 0x00000302;  /* SPLLDIV1 divide by 2; SPLLDIV2 divide by 4 */
  SCG->SPLLCFG = 0x00180000;  /* PREDIV=0: Divide SOSC_CLK by 0+1=1 */
                              /* MULT=24:  Multiply sys pll by 4+24=40 */
                              /* SPLL_CLK = 8MHz / 1 * 40 / 2 = 160 MHz */
  while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); /* Ensure SPLLCSR unlocked */
  SCG->SPLLCSR = 0x00000001; /* LK=0:        SPLLCSR can be written */
                             /* SPLLCMRE=0:  SPLL CLK monitor IRQ if enabled */
                             /* SPLLCM=0:    SPLL CLK monitor disabled */
                             /* SPLLSTEN=0:  SPLL disabled in Stop modes */
                             /* SPLLEN=1:    Enable SPLL */
  while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)); /* Wait for SPLL valid */
}

/* Change to normal RUN mode with 8MHz SOSC, 80 MHz PLL*/
void normal_80mhz_mode_run_init(void)
{  
  SCG->RCCR=SCG_RCCR_SCS(6)      /* PLL as clock source*/
    |SCG_RCCR_DIVCORE(0x01)      /* DIVCORE=1, div. by 2: Core clock = 160/2 MHz = 80 MHz*/
    |SCG_RCCR_DIVBUS(0x01)       /* DIVBUS=1, div. by 2: bus clock = 40 MHz*/
    |SCG_RCCR_DIVSLOW(0x02);     /* DIVSLOW=2, div. by 2: SCG slow, flash clock= 26 2/3 MHz*/
  
	while (((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT ) != 6){}
                                 /* Wait for sys clk src = SPLL */
}

void LED_PORT_init (void) 
{
  PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT D */
	
  /* Configure port D0 as GPIO output (BLUE LED on EVB) */
  PTD->PDDR |= 1<<0;       		/* Port D0: Data Direction= output */
  PORTD->PCR[0] = 0x00000100; /* Port D0: MUX = GPIO */
	
	PTD-> PCOR |= 1<<0;		/* Clear Output on port D0 (LED on) */
	
}

void led_triggle(int leds)
{
	int led = 0;

	static uint8_t led_status = 0;
	
	led = leds;
	
	switch(led)
	{
		case 0:
			
			if(led_status == 1)
			{
				led_status = 0;
				
				PTD-> PCOR |= 1<<0;		/* Clear Output on port D0 (LED on) */
			}
			else
			{
				led_status = 1;
				
				PTD-> PSOR |= 1<<0;   /* Set Output on port D0 (LED off) */
			}
			
			break;
			
		default:
			break;
	}
}

int main( void )
{

	vlps_init();
	
	init_rtc( 0 );  // 2019/6/4 17:33:00 = 1559640780 + 8*60*60
	
	sosc_8mhz_init();       			 /* Initialize system oscilator for 8 MHz xtal */
	spp_160mhz_init();     				 /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	normal_80mhz_mode_run_init();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	
	LED_PORT_init();	/* Configure port D0 as GPIO output (BLUE LED on EVB) */

	xTaskCreate( vLedTask, "vLedTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1);

}

void vLedTask( void *pvParameters )
{
	
	time_t utc_time_now = 0;
	 
	for(;;)
	{

		led_triggle( 0 );
		
		SEGGER_RTT_printf( 0, "\r\nsystem tick :%u.\r\n", xTaskGetTickCount()/1000 );

		utc_time_now = get_rtc_utc_time();
		
//		SEGGER_RTT_printf( 0, "RTC time now :%s", ctime( &utc_time_now ) );
		
		SEGGER_RTT_printf( 0, "RTC counter :%u.\r\n", utc_time_now );
		
		vTaskDelay( 1000 );
	
	}
}
	


























