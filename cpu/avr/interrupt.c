#include "contiki.h"
#include "interrupt.h"

#include <avr/io.h>
#include <avr/interrupt.h>


/*
 * \file
 * 			This file provides basic routines 
 * 			for interrupt setup and intialization
 * 	\author
 *			jacopo mondi <mondi@cs.unibo.it>
 *
 */

void 
interrupt_init(PMIC_CTRL_INTLVL_t int_level)
{
//if we have an XMEGA
#if defined(PMIC_CTRL)
	uint8_t temp;
	/* 
	 * Setup here interrupt enabled interrupt
	 * level
	 */
	PMIC.CTRL = int_level;

	/*disable or enable Round-Robin interrupt scheduling*/
#if USE_RR
	PMIC.CTRL |= PMIC_RREN_bm;
#else
	PMIC.CTRL &= ~PMIC_RREN_bm;
#endif

	/*move interrupt vector to application section*/
	temp = PMIC.CTRL & ~PMIC_IVSEL_bm;
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = temp;
#endif /* __AVR_ATxmega256A3__*/
}

uint8_t
interrupt_get_status(void){
#if defined(__AVR_ATxmega256A3__)
	uint8_t temp = PMIC.STATUS;
	do{
	}while(0);
	return temp;
#endif /* __AVR_ATxmega256A3__*/
}
