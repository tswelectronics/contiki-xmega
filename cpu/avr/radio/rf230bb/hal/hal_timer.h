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
#ifndef __HAL_AVR_TIMER__H
#define __HAL_AVR_TIMER__H
/**
 *    \addtogroup hal timer
 *    @{
 */
/*
 * ===
 */

/** 
 * \brief Macros defined for HAL_TIMER1.
 *
 *  These macros are used to define the correct 
 *  setupt of the AVR's Timer1, and to ensure that
 *  the hal_get_system_time function returns the 
 *  system time in symbols (16 us ticks).
 */
#if defined(XMEGA)
		#define HAL_RFTIMER TCC1
		#define HAL_RFTIMER_PRE			  	HAL_RFTIMER.CTRLA
		#define HAL_RFTIMER_INT	 				HAL_RFTIMER.INTCTRLA
		#define HAL_RFTIMER_OVF_bm 			0x03
		#define HAL_RFTIMER_CAPTURE	 		HAL_RFTIMER.INTCTRLB
		#define HAL_RFTIMER_CAPTURE_bm 	0xff

	#if	( F_CPU == 32768000UL )
		#define HAL_TIMER_PRE_CONFIG (0x07) /*prescaler 1024*/
		#define HAL_US_PER_SYMBOL ( 2 ) 
		#define HAL_SYMBOL_MASK 	( 0x7FFFffff )
	#else /* ! 32768000 */
	#error "Your Xmega clock is unsupported"
	#endif

#else /* ! XMEGA */
		#define HAL_RFTIMER_PRE 					TCCR1B
		#define HAL_RFTIMER_INT 					TIFR1 
		#define HAL_RFTIMER_OVF_bm 				(1<<TOIE1)
		#define HAL_RFTIMER_CAPTURE 			TIFR1 
		#define HAL_RFTIMER_CAPTURE_bm 		(1 << ICF1);             

	#if ( F_CPU == 16000000UL )
    #define HAL_TIMER_PRE_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS12 ) )
    #define HAL_US_PER_SYMBOL ( 1 )
    #define HAL_SYMBOL_MASK   ( 0xFFFFffff )
	#elif ( F_CPU == 8000000UL )
    #define HAL_TIMER_PRE_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
	#elif ( F_CPU == 4000000UL )
    #define HAL_TIMER_PRE_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 1 )
    #define HAL_SYMBOL_MASK   ( 0xFFFFffff )
	#elif ( F_CPU == 1000000UL )
    #define HAL_TIMER_PRE_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
	#else
    #error "Clock speed not supported."
	#endif /* F_CPU == 16000000UL */
#endif /* defined(XMEGA) */

/* 
 * ===
 * Timer1 wrapper functions
 * ===
 *
 * Those macros expand to platform specific functions 
 */
#if defined(__AVR__)
/** \brief Enable interrupt for timer overflow  */
#define hal_timer_enable_overflow_interrupt()\
 	(HAL_RFTIMER_INT |= HAL_RFTIMER_OVF_bm)
/** \brief Disable interrupt for timer overflow  */
#define hal_timer_disable_overflow_interrupt()\
	(HAL_RFTIMER_INT &= ~HAL_RFTIMER_OVF_bm)
/** \brief set the prescaler using macro based on F_CPU */
#define	hal_timer_set_prescaler()\
	(HAL_RFTIMER_PRE |= HAL_TIMER_PRE_CONFIG)
/** \brief Clear the input capture flag  */
#define hal_timer_clear_input_capture()\
	(HAL_RFTIMER_CAPTURE &= ~HAL_RFTIMER_CAPTURE_bm)	
#else /* MULLE */
#define HAL_ENABLE_OVERFLOW_INTERRUPT( ) ( TB4IC.BYTE = 1 )
#define HAL_DISABLE_OVERFLOW_INTERRUPT( ) ( TB4IC.BYTE = 0 )
#define hal_timer_enable_overflow_interrupt() HAL_ENABLE_OVERFLOW_INTERRUPT
#define hal_timer_disable_overflow_interrupt() HAL_DISABLE_OVERFLOW_INTERRUPT
#endif /* __AVR__*/

/** \} */
#endif
