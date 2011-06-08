
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

#ifndef __HAL_AVR_INT__h
#define __HAL_AVR_INT__h
/*
 * ===
 * Radio interrupt platform specific macros
 * ===
 */ 
/**
 * \name Low Level radio interrupt macros
 * \brief Platform specific macros used to set 
 * 				up interrupt from rf chip 
 * \{
 */
#if defined(__AVR__)

	#if HARWARE_REVISION == ZIGBIT
		#define RADIO_VECT 										INT5_vect// IRQ E5 for Zigbit example
		#define TIMER_OVF_VECT 								TIMER1_OVF_vect
		#define HAL_ENABLE_RADIO_INTERRUPT( ) \
				{ ( EIMSK |= ( 1 << INT5 ) ) ;\
 				EICRB |= 0x0C ;\
 				PORTE &= ~(1<<PE5);\
				DDRE &= ~(1<<DDE5); }
		#define HAL_DISABLE_RADIO_INTERRUPT() 	 (EIMSK &= ~( 1 << INT5 ) )

	#elif HW_PLATFORM == XMEGA
		#define  TIMER_OVF_VECT 								TCC1_OVF_vect	
		#if defined(__EMB_ZRF212__)
			#define  RF_PORT 												PORTC
			#define	 RF_PIN_bm											(1<<2)	
			#define  RADIO_VECT											PORTC_INT0_vect
			#define  HAL_ENABLE_RADIO_INTERRUPT() {\
						/*make RF_PIN input*/\
						RF_PORT.DIR &= ~RF_PIN_bm;\
						/*trigger rising edge and set as pulldown*/\
						RF_PORT.PIN2CTRL = PORT_ISC0_bm | PORT_OPC_PULLDOWN_gc;\
						/*clear intflags*/\
					  RF_PORT.INTFLAGS = PORT_INT0IF_bm;\
						/*enable INT0 on PIN2*/\
						RF_PORT.INT0MASK = PIN2_bm;\
						/*set input as high level*/\
					  RF_PORT.INTCTRL |= PORT_INT0LVL_HI_gc;}
			#define HAL_DISABLE_RADIO_INTERRUPT() {\
						/*clean up everything*/\
						RF_PORT.INT0MASK &= ~PIN2_bm;\
						RF_PORT.INTCTRL = 0;\
						RF_PORT.PIN2CTRL = 0;}
		#else
			#error "define rf interrupt pin for your xmega platform"
		#endif
	#else /* ! ZIGBIT ! XMEGA */
		#define RADIO_VECT 		 										TIMER1_CAPT_vect
		#define TIMER_OVF_VECT 										TIMER1_OVF_vect
		#define HAL_ENABLE_RADIO_INTERRUPT()	 		( TIMSK1 |= ( 1 << ICIE1 ) )
		#define HAL_DISABLE_RADIO_INTERRUPT() 		( TIMSK1 &= ~( 1 << ICIE1 ) )
	#endif /* HARDWARE_REVISION == ZIGBIT */

#else /* MULLE */
	#define HAL_ENABLE_RADIO_INTERRUPT( ) ( INT1IC.BYTE |= 1 )
	#define HAL_DISABLE_RADIO_INTERRUPT( ) ( INT1IC.BYTE &= ~(1) )
#endif /* !__AVR__ */
/** \} */

/**  \brief  Enable the interrupt from the radio transceiver. */
#define hal_enable_txrx_interrupt( ) HAL_ENABLE_RADIO_INTERRUPT( )
/** \brief  Disable the interrupt from the radio transceiver.
 *  \retval 0 if the pin is low, 1 if the pin is high. */
#define hal_disable_trx_interrupt( ) HAL_DISABLE_RADIO_INTERRUPT( )
#endif
