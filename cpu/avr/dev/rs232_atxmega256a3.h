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
 * @(#)$$
 */
/**
 * \file
 *         AVR atxmega specific definitions for the UART ports.
 *         This header is much more lighter than mega's one, since
 *         io.h provides enum and constant for all the funcionalities.
 * \author
 *         jacopo mondi <mondi@cs.unibo.it>
 */

#ifndef __RS232_ATXMEGA256a3__
#define __RS232_ATXMEGA256a3__
/******************************************************************************/
/***   Includes                                                               */
/******************************************************************************/
#include <avr/io.h>


/******************************************************************************/
/***   USART ports                                                            */
/******************************************************************************/
#define USARTc0  0
#define USARTc1  1
#define USARTd0  2
#define USARTd1  3
#define USARTe0  4
#define USARTe1  5
#define USARTf0  6
#define USARTf1  7

//XXX MISSING BAUDRATES

/******************************************************************************/
/***   Interrupt settings                                                     */
/******************************************************************************/
//USART_RXCINTLVL_t USART_INTERRUPT_RX_COMPLETE;
//USART_TXCINTLVL_t USART_INTERRUPT_TX_COMPLETE;

/******************************************************************************/
/***   Receiver / transmitter                                                 */
/******************************************************************************/
#define USART_RECEIVER_ENABLE USART_RXEN_bm
#define USART_TRANSMITTER_ENABLE USART_RXEN_bm

/******************************************************************************/
/***   Mode select                                                            */
/******************************************************************************/
USART_CMODE_t USART_CMODE;

/******************************************************************************/
/***   Stop bit select                                                            */
/******************************************************************************/
typedef enum USART_SMODE_enum
{
	USART_SMODE_1BIT_gc = 0x00,
	USART_SMODE_2BIT_gc = (0x01<<3),
} USART_SMODE_t;

/******************************************************************************/
/***   Parity                                                                 */
/******************************************************************************/
USART_PMODE_t USART_PMODE;
/******************************************************************************/
/***   Character size                                                         */
/******************************************************************************/
USART_CHSIZE_t USART_DATA_BITS;

#endif /* #ifndef __RS232_ATMEGA128__ */
