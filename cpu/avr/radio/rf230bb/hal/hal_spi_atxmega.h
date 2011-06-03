

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

#ifndef __AVR_HAL_ATXMEGA_SPI__
#define __AVR_HAL_ATXMEGA_SPI__

/* 
 *  ATXmega based platforms
 */

#if defined(__EMB_ZRF212__)
	/* Embit EMB-ZRF212 platform*/
	#define SSPORT     C
	#define SSPIN      (0x04)
	#define SPIPORT    C
	#define MOSIPIN    (0x05)
	#define MISOPIN    (0x06)
	#define SCKPIN     (0x07)
	#define RSTPORT    C
	#define RSTPIN     (0x00)
	#define IRQPORT    C
	#define IRQPIN     (0x02)
	#define SLPTRPORT  C
	#define SLPTRPIN   (0x03)
	#define FRONTEND_PORT C
	#define FRONTEND_PIN (0x00)
#if 0
	#define TXCWPORT   B
	#define TXCWPIN    (0x00)
	#define USART      1
	#define USARTVECT  USART1_RX_vect
	#define TICKTIMER  3
	#define HAS_CW_MODE
	#define HAS_SPARE_TIMER
#endif
#else
#error "define your xmega based platform"
#endif /*PLATFORM_EMB_ZRF212*/

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
#define CAT(x, y)      x##y
#define CAT2(x, y, z)  x##y##z
#define DDR(x) 			 CAT2(PORT,x,_DIR)	
#define PORT(x)			 CAT2(PORT,x,_OUT)
#define PIN(x)       CAT2(PORT,x,_IN)
#if 0
#define UCSR(num, let) 
#define RXEN(x)        
#define TXEN(x)        
#define TXC(x)         
#define RXC(x)         
#define RXCIE(x)       
#define UCSZ(x,y)      
#define UBRR(x,y)      
#define UDRE(x)        
#define UDRIE(x)       
#define UDR(x)         
#define TCNT(x)        
#define TIMSK(x)       
#define TCCR(x,y)      
#define COM(x,y)       
#define OCR(x,y)       
#define CS(x,y)        
#define WGM(x,y)       
#define OCIE(x,y)      
#define COMPVECT(x)    
#define UDREVECT(x)    
#define RXVECT(x)    
#endif
/** \} */

/* 
 * === 
 * Define SPI control registers mapping
 * ===
 */
#define HAL_SPI_RF_PORT SPIC
#define HAL_SPI_MODE  HAL_SPI_RF_PORT.CTRL
#define HAL_SPI_SPEED HAL_SPI_RF_PORT.CTRL
#define HAL_SPI_DATA HAL_SPI_RF_PORT.DATA
#define HAL_SPI_STATUS HAL_SPI_RF_PORT.STATUS

#endif
