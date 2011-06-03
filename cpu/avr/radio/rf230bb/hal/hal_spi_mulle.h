
/*   Copyright (c) 2008, Swedish Institute of Computer Science
 *  All rights reserved. 
 *
 *  Additional fixes for AVR contributed by:
 *
 *	Colin O'Flynn coflynn@newae.com
 *	Eric Gnoske egnoske@gmail.com
 *	Blake Leverett bleverett@gmail.com
 *	Mike Vidales mavida404@gmail.com
 *	Kevin Brown kbrown3@uccs.edu
 *	Nate Bohlmann nate@elfwerks.com
 *	Jacopo Mondi mondi@cs.unibo.it
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __AVR_HAL_MULLE_SPI__
#define __AVR_HAL_MULLE_SPI__

/*
 *  Mulle 5.2 platform
 */
#if CONTIKI_TARGET_MULLE
#   define SSPORT     3
#   define SSPIN      5
#   define MOSIPORT   1
#   define MOSIPIN    1
#   define MISOPORT   1
#   define MISOPIN    0
#   define SCKPORT    3
#   define SCKPIN     3
#   define RSTPORT    4
#   define RSTPIN     3
#   define IRQPORT    8
#   define IRQPIN     3
#   define SLPTRPORT  0
#   define SLPTRPIN   7
#   define HAS_SPARE_TIMER
#else
#error "Mulle platform undefined in hal.h"
#endif


/**
 * \name 
 * 			Macros used to generate read register names 
 * 			from platform-specific definitions of ports.
 *
 * \brief
 * 		  The various CAT macros (DDR, PORT, and PIN) are 
 * 		  used to assign port/pin/DDR names to various 
 * 		  macro variables.  The variables are assigned 
 * 		  based on the specific connections made in
 * 			the hardware.  For example TCCR(TICKTIMER,A)
 * 			can be used in place of TCCR0A if TICKTIMER is 
 * 			defined as 0.
 * \{
 */
#define CAT(x, y)      x##y.BYTE
#define CAT2(x, y, z)  x##y##z.BYTE
#define DDR(x)         CAT(PD,  x)
#define PORT(x)        CAT(P, x)
#define PIN(x)         CAT(P, x)
#define UCSR(num, let) CAT2(UCSR,num,let)
#define RXEN(x)        CAT(RXEN,x)
#define TXEN(x)        CAT(TXEN,x)
#define TXC(x)         CAT(TXC,x)
#define RXC(x)         CAT(RXC,x)
#define RXCIE(x)       CAT(RXCIE,x)
#define UCSZ(x,y)      CAT2(UCSZ,x,y)
#define UBRR(x,y)      CAT2(UBRR,x,y)
#define UDRE(x)        CAT(UDRE,x)
#define UDRIE(x)       CAT(UDRIE,x)
#define UDR(x)         CAT(UDR,x)
#define TCNT(x)        CAT(TCNT,x)
#define TIMSK(x)       CAT(TIMSK,x)
#define TCCR(x,y)      CAT2(TCCR,x,y)
#define COM(x,y)       CAT2(COM,x,y)
#define OCR(x,y)       CAT2(OCR,x,y)
#define CS(x,y)        CAT2(CS,x,y)
#define WGM(x,y)       CAT2(WGM,x,y)
#define OCIE(x,y)      CAT2(OCIE,x,y)
#define COMPVECT(x)    CAT2(TIMER,x,_COMPA_vect)
#define UDREVECT(x)    CAT2(USART,x,_UDRE_vect)
#define RXVECT(x)      CAT2(USART,x,_RX_vect)
/** \} */
/* 
 * === 
 * Define SPI control registers mapping
 * ===
 */
#define HAL_SPI_MODE  SPCR
#define HAL_SPI_SPEED SPSR
#define HAL_SPI_DATA  SPDR
#define HAL_SPI_STATUS SPSR
#define SPI_ENABLE_bm (1<<SPE)
#define SPI_MASTER_bm (1<<MSTR)
#define SPI_CLK2X_bm  (1<<SPI2X)
#define SPI_IF_bm (1<<SPIF)

#endif
