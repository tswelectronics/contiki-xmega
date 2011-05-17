/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * @(#)$Id: rs232.c,v 1.7 2010/10/27 14:51:20 dak664 Exp $
 * modified on 2011/05/17 jacopo mondi mondi@cs.unibo.it
 */

/* \file
 * 				RS232 implementation for various AVR (currently
 * 				ATmega, ATxmega and AV90USB).
 * 				Features the rs232_port structure where each device maps
 * 				its register in order to provide an uniform point
 * 				of view
 * \author
 * 				2010/10/27 dak664
 * 				2011/5/17 jacopo mondi <mondi@cs.unibo.it>	
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "contiki-conf.h"
#include "contiki.h"

#include "dev/slip.h"
#include "dev/rs232.h"

#ifdef RS232_CONF_PRINTF_BUFFER_LENGTH
#define RS232_PRINTF_BUFFER_LENGTH RS232_CONF_PRINTF_BUFFER_LENGTH
#else
#define RS232_PRINTF_BUFFER_LENGTH 64
#endif

#ifndef ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
#define ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT 1
#endif


/* This is a generic UART structure.
 * Each platform maps corresponding registers into
 * structure elements.
 * This convention (hopefully) works for atmega, atxmega, and at90usb
 */
typedef struct {
  volatile uint8_t * DATA;
  volatile uint8_t * BAUDH;
  volatile uint8_t * BAUDL;
  volatile uint8_t * FUNCTION;
  volatile uint8_t * INTERRUPT;
  volatile uint8_t * FORMAT;
  volatile uint8_t txwait;
  int (* input_handler)(unsigned char);
} rs232_t;

#if defined (__AVR_ATmega128__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega128RFA1__)

static rs232_t rs232_ports[2] = {
  {   // UART0
    &UDR0,
    &UBRR0H,
    &UBRR0L,
    &UCSR0B,
    &UCSR0B,
    &UCSR0C,
    0,
    NULL
  },

  {  // UART1
    &UDR1,
    &UBRR1H,
    &UBRR1L,
    &UCSR1B,
    &UCSR1B,
    &UCSR1C,
    0,
    NULL
  }
};
/*---------------------------------------------------------------------------*/
ISR(USART0_TX_vect)
{
  rs232_ports[RS232_PORT_0].txwait = 0;
}

/*---------------------------------------------------------------------------*/
ISR(USART0_RX_vect)
{
  unsigned char c;

  c = *(rs232_ports[RS232_PORT_0].DATA);

  if(rs232_ports[RS232_PORT_0].input_handler != NULL) {
    rs232_ports[RS232_PORT_0].input_handler(c);
  }
}
/*---------------------------------------------------------------------------*/
ISR(USART1_TX_vect)
{
  rs232_ports[RS232_PORT_1].txwait = 0;
}

/*---------------------------------------------------------------------------*/
ISR(USART1_RX_vect)
{
  unsigned char c;

  c = *(rs232_ports[RS232_PORT_1].DATA);

  if(rs232_ports[RS232_PORT_1].input_handler != NULL) {
    rs232_ports[RS232_PORT_1].input_handler(c);
  }
}

#elif defined (__AVR_AT90USB1287__)
/* Has only UART1, map it to port 0 */
static rs232_t rs232_ports[1] = {
  {  // UART1
    &UDR1,
    &UBRR1H,
    &UBRR1L,
    &UCSR1B,
    &UCSR1B,
    &UCSR1C,
    0,
    NULL
  }
};
/*---------------------------------------------------------------------------*/
ISR(USART1_TX_vect)
{
  rs232_ports[RS232_PORT_0].txwait = 0;
}

/*---------------------------------------------------------------------------*/
ISR(USART1_RX_vect)
{
  unsigned char c;

  c = *(rs232_ports[RS232_PORT_0].DATA);

  if(rs232_ports[RS232_PORT_0].input_handler != NULL) {
    rs232_ports[RS232_PORT_0].input_handler(c);
  }
}
#elif defined (__AVR_ATxmega256A3__)
	/* The Xmega header file already contains the USART_t 
	 * type definitio*/

static rs232_t rs232_ports[8] = {
	{
		&USARTC0.DATA,
		&USARTC0.BAUDCTRLA,
		&USARTC0.BAUDCTRLB,
		&USARTC0.CTRLB,
		&USARTC0.CTRLA,
		&USARTC0.CTRLC,
		0,
		NULL
	},
	{
		&USARTC1.DATA,
		&USARTC1.BAUDCTRLA,
		&USARTC1.BAUDCTRLB,
		&USARTC1.CTRLB,
		&USARTC1.CTRLA,
		&USARTC1.CTRLC,
		0,
		NULL
	},
	{
		&USARTD0.DATA,
		&USARTD0.BAUDCTRLA,
		&USARTD0.BAUDCTRLB,
		&USARTD0.CTRLB,
		&USARTD0.CTRLA,
		&USARTD0.CTRLC,
		0,
		NULL
	},
	{
		&USARTD1.DATA,
		&USARTD1.BAUDCTRLA,
		&USARTD1.BAUDCTRLB,
		&USARTD1.CTRLB,
		&USARTD1.CTRLA,
		&USARTD1.CTRLC,
		0,
		NULL
	},
	{
		&USARTE0.DATA,
		&USARTE0.BAUDCTRLA,
		&USARTE0.BAUDCTRLB,
		&USARTE0.CTRLB,
		&USARTE0.CTRLA,
		&USARTE0.CTRLC,
		0,
		NULL
	},
	{
		&USARTE1.DATA,
		&USARTE1.BAUDCTRLA,
		&USARTE1.BAUDCTRLB,
		&USARTE1.CTRLB,
		&USARTE1.CTRLA,
		&USARTE1.CTRLC,
		0,
		NULL
	},

	{
		&USARTF0.DATA,
		&USARTF0.BAUDCTRLA,
		&USARTF0.BAUDCTRLB,
		&USARTF0.CTRLB,
		&USARTF0.CTRLA,
		&USARTF0.CTRLC,
		0,
		NULL
	},
	{
		&USARTF1.DATA,
		&USARTF1.BAUDCTRLA,
		&USARTF1.BAUDCTRLB,
		&USARTF1.CTRLB,
		&USARTF1.CTRLA,
		&USARTF1.CTRLC,
		0,
		NULL
	},
};

/*-- interrupt level HIGH for transmit, empty and receive --*/
USART_RXCINTLVL_t USART_INTERRUPT_RX_COMPLETE = USART_RXCINTLVL_HI_gc;
USART_TXCINTLVL_t USART_INTERRUPT_TX_COMPLETE = USART_TXCINTLVL_HI_gc;

/*=== Interrupt Routines ===*/
/*--USARTC0------------------------------------------------------------------*/
ISR(USARTC0_TXC_vect)
{
  rs232_ports[USARTc0].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTC0_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTc0].DATA);

  if(rs232_ports[USARTc0].input_handler != NULL) {
    rs232_ports[USARTc0].input_handler(c);
  }
}

/*---USARTC1-----------------------------------------------------------------*/
ISR(USARTC1_TXC_vect)
{
  rs232_ports[USARTc1].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTC1_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTc1].DATA);

  if(rs232_ports[USARTc1].input_handler != NULL) {
    rs232_ports[USARTc1].input_handler(c);
  }
}

/*---USARTD0-----------------------------------------------------------------*/
ISR(USARTD0_TXC_vect)
{
  rs232_ports[USARTd0].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTD0_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTd0].DATA);

  if(rs232_ports[USARTd0].input_handler != NULL) {
    rs232_ports[USARTd0].input_handler(c);
  }
}
/*---USARTD1-----------------------------------------------------------------*/
ISR(USARTD1_TXC_vect)
{
  rs232_ports[USARTd1].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTD1_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTd1].DATA);

  if(rs232_ports[USARTd1].input_handler != NULL) {
    rs232_ports[USARTd1].input_handler(c);
  }
}

/*---USARTE0-----------------------------------------------------------------*/
ISR(USARTE0_TXC_vect)
{
  rs232_ports[USARTe0].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTE0_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTe0].DATA);

  if(rs232_ports[USARTe0].input_handler != NULL) {
    rs232_ports[USARTe0].input_handler(c);
  }
}

/*---USARTE1-----------------------------------------------------------------*/
ISR(USARTE1_TXC_vect)
{
  rs232_ports[USARTe1].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTE1_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTe1].DATA);

  if(rs232_ports[USARTe1].input_handler != NULL) {
    rs232_ports[USARTe1].input_handler(c);
  }
}

/*---USARTF0-----------------------------------------------------------------*/
ISR(USARTF0_TXC_vect)
{
  rs232_ports[USARTf0].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTF0_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTf0].DATA);

  if(rs232_ports[USARTf0].input_handler != NULL) {
    rs232_ports[USARTf0].input_handler(c);
  }
}

#if 0
/*---USARTF1-----------------------------------------------------------------*/
ISR(USARTF1_TXC_vect)
{
  rs232_ports[USARTf1].txwait = 0;
}
/*---------------------------------------------------------------------------*/
ISR(USARTF1_RXC_vect)
{
  unsigned char c;

  c = *(rs232_ports[USARTf1].DATA);

  if(rs232_ports[USARTf1].input_handler != NULL) {
    rs232_ports[USARTf1].input_handler(c);
  }
}
#endif /*#if 0*/

#else
#error Please define the UART registers for your MCU!
#endif

/*---------------------------------------------------------------------------*/
void
rs232_init (uint8_t port, uint16_t bd, uint8_t ffmt)
{
	rs232_t *rs232 = &rs232_ports[port];

	/*
	 * Setup baudrate
	 */
  (*(*rs232).BAUDH) = (uint8_t)(bd>>8);
  (*(*rs232).BAUDL) = (uint8_t)bd;

  /*
   * - Enable receiver and transmitter,
   * - Enable interrupts for receiver and transmitter
	 *   (high priority interrupts for xmega)
   */
	(*(*rs232).FUNCTION) |= 
					USART_RECEIVER_ENABLE |
					USART_TRANSMITTER_ENABLE;
	(*(*rs232).INTERRUPT) |= 
	 				USART_INTERRUPT_RX_COMPLETE |
					USART_INTERRUPT_TX_COMPLETE;

  /*
   * - mode (sync. / async)
   * - Parity
   * - Stop bits
   * - charater size (9 bits are currently not supported)
   * - clock polarity
   */
	(*(*rs232).FORMAT) = ffmt;

  (*rs232).txwait = 0;
  (*rs232).input_handler = NULL;
}

void
rs232_print_p(uint8_t port, prog_char *buf)
{
  while(pgm_read_byte(buf)) {
    rs232_send(port, pgm_read_byte(buf));
    ++buf;
  }
}
/*---------------------------------------------------------------------------*/
void
rs232_print(uint8_t port, char *buf)
{
  while(*buf) {
#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
    if(*buf=='\n') rs232_send(port, '\r');
	if(*buf=='\r') buf++; else rs232_send(port, *buf++);
#else
    rs232_send(port, *buf++);
#endif
  }
}
/*---------------------------------------------------------------------------*/
void
rs232_printf(uint8_t port, const char *fmt, ...)
{
  va_list ap;
  static char buf[RS232_PRINTF_BUFFER_LENGTH];

  va_start (ap, fmt);
  vsnprintf (buf, RS232_PRINTF_BUFFER_LENGTH, fmt, ap);
  va_end(ap);

  rs232_print (port, buf);
}
/*---------------------------------------------------------------------------*/
void
rs232_send(uint8_t port, unsigned char c)
{
  rs232_ports[port].txwait = 1;
  *(rs232_ports[port].DATA) = c;
  while(rs232_ports[port].txwait);
}
/*---------------------------------------------------------------------------*/
void
rs232_set_input(uint8_t port, int (*f)(unsigned char))
{
  rs232_ports[port].input_handler = f;
}
/*---------------------------------------------------------------------------*/
void
slip_arch_writeb(unsigned char c)
{
  rs232_send(SLIP_PORT, c);
}
/*---------------------------------------------------------------------------*/
int rs232_stdout_putchar(char c, FILE *stream);
static uint8_t stdout_rs232_port=RS232_PORT_0;
static FILE rs232_stdout = FDEV_SETUP_STREAM(rs232_stdout_putchar,
					     NULL,
					     _FDEV_SETUP_WRITE);

int rs232_stdout_putchar(char c, FILE *stream)
{
#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
  if(c=='\n') rs232_send(stdout_rs232_port, '\r');
  if(c!='\r') rs232_send (stdout_rs232_port, c);
#else
  rs232_send (stdout_rs232_port, c);
#endif
  return 0;
}
/*---------------------------------------------------------------------------*/
void rs232_redirect_stdout (uint8_t port) {
  stdout_rs232_port = port;
  stdout = &rs232_stdout;
}
