/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * $Id: hello-world.c,v 1.1 2006/10/02 21:46:46 adamdunkels Exp $
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include <dev/rs232.h>

#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{

	static struct etimer our_ev_timer;

	PROCESS_BEGIN();

	dprintf("Hello, world\n");

	struct process *p = PROCESS_CURRENT();
	
	dprintf("Process=%p Thread=%p  Name=\"%s\" \n",p,p->thread,PROCESS_NAME_STRING(p));

	etimer_set(&our_ev_timer, 1 * CLOCK_SECOND);
 
	PROCESS_WAIT_UNTIL(etimer_expired( &our_ev_timer) );

	dprintf("Waited 1 seconds, start main loop!\n");

	while(1)
	{

		PROCESS_WAIT_UNTIL(etimer_expired( &our_ev_timer) );
		if ( etimer_expired(&our_ev_timer) ) 
		{
			etimer_set(&our_ev_timer, CLOCK_SECOND * 1);
    	}
			
		dprintf("Periodic entry into routine performed!\n");
	  dprintf("Process=%p Thread=%p  Name=\"%s\" \n",p,p->thread,PROCESS_NAME_STRING(p));

	}
 
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
