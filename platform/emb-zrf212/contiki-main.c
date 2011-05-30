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
#include "autostart.h"

#include "dev/leds.h"
#include "dev/rs232.h"
#include "interrupt.h"
#include "watchdog.h"

#include	"apps/common.h"



#define ANNOUNCE_BOOT 1
#define DEBUG 1
#if DEBUG
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#define PRINTSHORT(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...)
#define PRINTSHORT(...)
#endif

/*-------------------------Low level initialization------------------------*/
void initialize_lowlevel(void)
{
	/* --- USART DATA ---
	 * parity, chsize, stop bit and communication mode*/
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

	/*--- Setup and configure global interrupts ---*/
	interrupt_init(int_level);

	/*--- Setup system clock (if required) and start timer --- */
#if defined(__SYSTEM_CLOCK_SETUP__)
	system_clock_init();
#endif /* __SYSTEM_CLOCK_SETUP__*/
	clock_init();

	/*--- Setup serial port on USARTD0 ---*/
	rs232_init(RS232_PORT_0, XMEGA_USART_CALC(9600), 
			(chsize | pmode | cmode | smode) );
	rs232_redirect_stdout(RS232_PORT_0);

	/*--- Setup led module ---*/
#if defined(__USE_LEDS__)
	leds_init();
	leds_on(0x08);
	//leds_off(0x01);
#endif /* __USE_LEDS__ */
	PRINTF("Setup LED module\n\0");

	/*--- Watchdog init ---*/
	PRINTF("Setup Watchdog module\n\0");
	watchdog_init();

	/*--- start global interrupt  vector ---*/
	PRINTF("Setup global interrupt vector\n\0");
	interrupt_start();

	/*process subsystem start*/
	PRINTF("Starting process subsystem..\n\n\0");
  process_init();

#if ANNOUNCE_BOOT	
	clock_wait(70);
  printf_P(PSTR("\n*******Booting %s*******\n"),CONTIKI_VERSION_STRING);
#endif
	leds_off(0x08);
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
  process_start(&etimer_process, NULL);
	process_start(&blink_process, NULL);
	process_start(&hello_world_process, NULL);
 // autostart_start(autostart_processes);

	while (1){
		watchdog_periodic();
		process_run();
	}

	return 0;

}

