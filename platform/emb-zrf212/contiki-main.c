/*
 * Copyright (c) 2006, Technical University of Munich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/*
 * \file 
 * 				Main Loop for Contiki OS on EMB-ZRF212 plat.
 * 				This Module initialize low-level
 * 				subsystems and starts main loop, where auto-start
 * 				processes are launched and event dispatched
 *
 * \author
 * 				jacopo mondi <mondi@cs.unibo.it>
 */
#include <stdio.h>
#include <string.h>
#include <util/delay.h>

#include "lib/mmem.h"
#include "loader/symbols-def.h"
#include "loader/symtab.h"

#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "dev/leds.h"
#include "dev/rs232.h"
#include "interrupt.h"

/*-------------------------Low level initialization------------------------*/
void initialize_lowlevel(void)
{
	/* --- USART DATA ---
	 * baud, parity, chsize, stop bit and communication mode*/
	uint16_t baud=212; /*baud 9600 @32760 KHz*/
	USART_CHSIZE_t chsize = USART_CHSIZE_8BIT_gc;
	USART_PMODE_t cmode = USART_CMODE_ASYNCHRONOUS_gc;
	USART_CMODE_t pmode = USART_PMODE_DISABLED_gc;
	USART_SMODE_t smode = USART_SMODE_1BIT_gc;
	/* --- INTERRUPT DATA ---
	 * enable 3 level interrupt management (high, med and low)*/
	PMIC_CTRL_INTLVL_t int_level = PMIC_CTRL_HML_gm;

	/*
	 *  --- Initialize Low-Level --- 
	 */

	/*setup and configure global interrupts*/
	interrupt_init(int_level);
/*- Setup led module -- */
#if defined(__USE_LEDS__)
	leds_init();
	leds_on(0x01);
	leds_off(0x01);
#endif /* __USE_LEDS__ */
	/*--- Setup system clock (if required) and start timer */
#if defined(__SYSTEM_CLOCK_SETUP__)
	system_clock_init();
#endif /* __SYSTEM_CLOCK_SETUP__*/
	clock_init();

	/*setup serial port on USARTD0*/
	rs232_init(RS232_PORT_0, baud, 
			(chsize | pmode | cmode | smode) );
	rs232_print(RS232_PORT_0, "Setup Serial Channel\n\0");

	rs232_print(RS232_PORT_0, "Setup LED module\n\0");

	/* start global interrupt  vector*/
	interrupt_start();
	rs232_print(RS232_PORT_0, "Startup global interrupt vector\n\0");
	rs232_print(RS232_PORT_0, "BOOTING CONTIKI OS...\n\0");
	clock_wait(70);
	rs232_print(RS232_PORT_0, "DONE\n\0");

	return;

}
/*-------------------------------------------------------------------------*/
/*------------------------- Main Scheduler loop----------------------------*/
/*-------------------------------------------------------------------------*/
int
main(void)
{
	/*--- Setup platform and cpu specific subsystems ---*/
	initialize_lowlevel();

//	leds_init();
//	clock_init();
	

//	leds_on(0x03);
//	leds_off(0x03);
//	process_start(&etimer_process, NULL);
//	ctimer_init();

	//autostart_start(autostart_processes);
	while (1){
		;

		leds_on(0x01);
		clock_wait(125);
		leds_off(0x01);
		clock_wait(125);
		rs232_print(RS232_PORT_0,
				"A led is blinking... we are alive and well\n\0"); 
		//process_run();
		//watchdog_periodic();
	}

	return 0;

}

