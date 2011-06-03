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

/**
 *    \addtogroup hal
 *    @{
 */

/**
 *  \file
 *  \brief This file contains low-level radio driver code.
 *
 *   $Id: hal.h,v 1.5 2010/12/03 20:42:01 dak664 Exp $
*/

#ifndef HAL_AVR_H
#define HAL_AVR_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
//#include <util/crc16.h>
#include "contiki-conf.h"
/*============================ MACROS ========================================*/

// TEST CODE
#define TRIG1 DDRB |= 0x04, PINB |= 0x04
#define TRIG2 DDRD |= 0x80, PIND |= 0x80

/** \name This is the list of pin configurations needed for a given platform.
 * \brief Change these values to port to other platforms.
 * \{
 */
/* Define all possible revisions here */
// Don't use zero, it will match if undefined!
// RAVEN_D : Raven kit with LCD display
// RAVENUSB_C : used for USB key or Raven card 
// RCB_B : RZ200 kit from Atmel based on 1281V
// ZIGBIT : Zigbit module from Meshnetics
// ATMEGA128RFA1 : Bare chip with internal radio
#define RAVEN_D	    4
#define RAVENUSB_C  1
#define RCB_B	    	2
#define ZIGBIT			3
#define ATMEGA128RFA1   4
#define XMEGA 			5


/*
 * === 
 * GPIO registers for SPI pins 
 * ===
 */
/* TODO: Move to platform (or CPU specific) */
#if RCB_REVISION == RCB_B
/* 1281 rcb */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define IRQPORT    D
#   define IRQPIN     (0x04)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_SPARE_TIMER

#elif HARWARE_REVISION == ZIGBIT
/* 1281V Zigbit */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    A
#   define RSTPIN     (0x07)
#   define IRQPORT    E
#   define IRQPIN     (0x05)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define TXCWPORT   B
#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
//#   define TICKTIMER  3
//#   define HAS_SPARE_TIMER // Not used

#elif HW_PLATFORM == XMEGA
/* Xmega based platforms*/
#if defined(__EMB_ZRF212__)
	/* Embit EMB-ZRF212 platform*/
#   define SSPORT     C
#   define SSPIN      (0x04)
#   define SPIPORT    C
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPIN     (0x07)
#   define RSTPORT    C
#   define RSTPIN     (0x00)
#   define IRQPORT    C
#   define IRQPIN     (0x02)
#   define SLPTRPORT  C
#   define SLPTRPIN   (0x03)
#		define FRONTEND_PORT C
#		define FRONTEND_PIN (0x00)
#if 0
#   define TXCWPORT   B
#   define TXCWPIN    (0x00)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER
#endif
#else
#error "define your xmega based platform"
#endif /*PLATFORM_EMB_ZRF212*/


#elif RAVEN_REVISION == RAVEN_D
/* 1284 raven */
#   define SSPORT     B
#   define SSPIN      (0x04)
#   define SPIPORT    B
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPIN     (0x07)
#   define RSTPORT    B
#   define RSTPIN     (0x01)
#   define IRQPORT    D
#   define IRQPIN     (0x06)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x03)
#   define TXCWPORT   B
#   define TXCWPIN    (0x00)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER

#elif RAVEN_REVISION == RAVENUSB_C
/* 1287USB raven */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define IRQPORT    D
#   define IRQPIN     (0x04)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define TXCWPORT   B
#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER

#elif HARWARE_REVISION == ATMEGA128RFA1
/* ATmega1281 with internal AT86RF231 radio */
#if 0
#   define SSPORT     B
#   define SSPIN      (0x04)
#   define SPIPORT    B
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPIN     (0x07)
#   define RSTPORT    B
#   define RSTPIN     (0x01)
#   define IRQPORT    D
#   define IRQPIN     (0x06)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x03)
#   define TXCWPORT   B
#   define TXCWPIN    (0x00)
#endif
#   define SLPTRPORT  TRXPR
#   define SLPTRPIN   1
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER

#elif CONTIKI_TARGET_MULLE
/* mulle 5.2 (TODO: move to platform specific) */

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
#error "Platform undefined in hal.h"
#endif

/* For architectures that have all SPI signals on the same port */
#ifndef SSPORT
#define SSPORT SPIPORT
#endif

#ifndef SCKPORT
#define SCKPORT SPIPORT
#endif

#ifndef MOSIPORT
#define MOSIPORT SPIPORT
#endif

#ifndef MISOPORT
#define MISOPORT SPIPORT
#endif
/** \} */

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
#if defined(__AVR__)
#define CAT(x, y)      x##y
#define CAT2(x, y, z)  x##y##z
#if HW_PLATFORM == XMEGA
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
#else /*! XMEGA */
#define DDR(x)         CAT(DDR,  x)
#define PORT(x)        CAT(PORT, x)
#define PIN(x)         CAT(PIN,  x)
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
#endif /* XMEGA */
#endif /* __AVR__ */

/* TODO: Move to CPU specific */
#if defined(CONTIKI_TARGET_MULLE)
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
#endif
/** \} */

/*
 * ===
 * SPI PORT pin macros
 * ===
 */
/**
 * \name  Pin macros
 * \brief These macros convert the platform-specific pin 
 *  			defines into names and functions that the 
 *  			source code can directly use.
 *
 * \{
 */
#if defined(__AVR_ATmega128RFA1__)

#define hal_set_rst_low( )    ( TRXPR &= ~( 1 << TRXRST ) ) /**< This macro pulls the RST pin low. */
#define hal_set_rst_high( )   ( TRXPR |= ( 1 << TRXRST ) ) /**< This macro pulls the RST pin high. */
#define hal_set_slptr_high( ) ( TRXPR |= ( 1 << SLPTR ) )      /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low( )  ( TRXPR &= ~( 1 << SLPTR ) )     /**< This macro pulls the SLP_TR pin low. */
#define hal_get_slptr( ) (    ( TRXPR & ( 1 << SLPTR ) ) >> SLPTR )  /**< Read current state of the SLP_TR pin (High/Low). */

#else /* ! __AVR_ATmega128RFA1__) */
#define SLP_TR                SLPTRPIN            /**< Pin number that corresponds to the SLP_TR pin. */
#define DDR_SLP_TR            DDR( SLPTRPORT )    /**< Data Direction Register that corresponds to the port where SLP_TR is connected. */
#define PORT_SLP_TR           PORT( SLPTRPORT )   /**< Port (Write Access) where SLP_TR is connected. */
#define PIN_SLP_TR            PIN( SLPTRPORT )    /**< Pin (Read Access) where SLP_TR is connected. */
#define hal_set_slptr_high( ) ( PORT_SLP_TR |= ( 1 << SLP_TR ) )      /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low( )  ( PORT_SLP_TR &= ~( 1 << SLP_TR ) )     /**< This macro pulls the SLP_TR pin low. */
#define hal_get_slptr( ) (    ( PIN_SLP_TR & ( 1 << SLP_TR ) ) >> SLP_TR )  /**< Read current state of the SLP_TR pin (High/Low). */
/*XXX undefine lined 2551 iox256a3.h*/
#undef RST
#define RST                   RSTPIN              /**< Pin number that corresponds to the RST pin. */
#define DDR_RST               DDR( RSTPORT )      /**< Data Direction Register that corresponds to the port where RST is */
#define PORT_RST              PORT( RSTPORT )     /**< Port (Write Access) where RST is connected. */
#define PIN_RST               PIN( RSTPORT /* BUG? */)      /**< Pin (Read Access) where RST is connected. */
#define hal_set_rst_high( )   ( PORT_RST |= ( 1 << RST ) )  /**< This macro pulls the RST pin high. */
#define hal_set_rst_low( )    ( PORT_RST &= ~( 1 << RST ) ) /**< This macro pulls the RST pin low. */
#define hal_get_rst( )        ( ( PIN_RST & ( 1 << RST )  ) >> RST )  /**< Read current state of the RST pin (High/Low). */
#define HAL_SS_PIN            SSPIN               /**< The slave select pin. */
#define HAL_SCK_PIN           SCKPIN              /**< Data bit for SCK. */
#define HAL_MOSI_PIN          MOSIPIN
#define HAL_MISO_PIN          MISOPIN
#define HAL_PORT_SPI          PORT( SPIPORT )     /**< The SPI module is located on PORTB. (Not true, just for megas) */
#define HAL_PORT_SS            PORT( SSPORT )
#define HAL_PORT_SCK           PORT( SCKPORT )
#define HAL_PORT_MOSI          PORT( MOSIPORT )     /**< The SPI module uses GPIO might be split on different ports. */
#define HAL_PORT_MISO          PORT( MISOPORT )     /**< The SPI module uses GPIO might be split on different ports. */
#define HAL_DDR_SPI           DDR( SPIPORT )      /**< Data Direction Register for PORTB. */
#define HAL_DDR_SS             DDR( SSPORT )      /**< Data Direction Register for MISO GPIO pin. */
#define HAL_DDR_SCK            DDR( SCKPORT )      /**< Data Direction Register for MISO GPIO pin. */
#define HAL_DDR_MOSI           DDR( MOSIPORT )      /**< Data Direction Register for MISO GPIO pin. */
#define HAL_DDR_MISO           DDR( MISOPORT )      /**< Data Direction Register for MOSI GPIO pin. */
#define HAL_DD_SS             SSPIN               /**< Data Direction bit for SS. */
#define HAL_DD_SCK            SCKPIN              /**< Data Direction bit for SCK. */
#define HAL_DD_MOSI           MOSIPIN             /**< Data Direction bit for MOSI. */
#define HAL_DD_MISO           MISOPIN             /**< Data Direction bit for MISO. */
#endif /* defined(__AVR_ATmega128RFA1__) */
/** \} */

/**< MACRO for pulling SS high. */
#define HAL_SS_HIGH( ) \
	(HAL_PORT_SS |= ( 1 << HAL_SS_PIN )) 
/**< MACRO for pulling SS low. */
#define HAL_SS_LOW( )\
	(HAL_PORT_SS &= ~( 1 << HAL_SS_PIN )) 

/* 
 * === 
 * Define SPI control registers mapping
 * ===
 */
#if defined(XMEGA)
#define HAL_SPI_RF_PORT SPIC
#define HAL_SPI_MODE  HAL_SPI_RF_PORT.CTRL
#define HAL_SPI_SPEED HAL_SPI_RF_PORT.CTRL
#define HAL_SPI_DATA HAL_SPI_RF_PORT.DATA
#define HAL_SPI_STATUS HAL_SPI_RF_PORT.STATUS
#else /* ! XMEGA */
#define HAL_SPI_MODE  SPCR
#define HAL_SPI_SPEED SPSR
#define HAL_SPI_DATA  SPDR
#define HAL_SPI_STATUS SPSR
/* Map bitmasks onto xmega ones, from avr-libc.
 * This way no additional definitions are required for xmegas*/
#define SPI_ENABLE_bm (1<<SPE)
#define SPI_MASTER_bm (1<<MSTR)
#define SPI_CLK2X_bm  (1<<SPI2X)
#define SPI_IF_bm (1<<SPIF)
#endif /*XMEGA*/

/*
 * ===
 * SPI control wrapper functions
 * ===
 */
#define	hal_set_sleep() \
    DDR_SLP_TR |= (1 << SLP_TR);
#define	hal_set_reset() \
   DDR_RST    |= (1 << RST);

#define	hal_spi_clear()\
		HAL_SPI_MODE = 0; HAL_SPI_SPEED = 0;
#define	hal_spi_set_direction() {\
    HAL_DDR_SPI   |= (1 << HAL_DD_SS)\
								  | (1 << HAL_DD_SCK)\
								  | (1 << HAL_DD_MOSI);}
#define		hal_spi_pin_init() \
    HAL_PORT_SPI  |= (1 << HAL_DD_SS) | (1 << HAL_DD_SCK);
#define		hal_spi_set_mode() \
    HAL_SPI_MODE  |= SPI_ENABLE_bm | SPI_MASTER_bm; 
#define		hal_spi_set_speed() \
    HAL_SPI_SPEED |= SPI_CLK2X_bm;

/*
 * ===
 * HAL_TIMER1 MACROS
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
#if defined(__AVR__)
#if defined(XMEGA)
		#define HAL_RFTIMER TCC1
		#define HAL_RFTIMER_PRE			  	HAL_RFTIMER1.CTRLA
		#define HAL_RFTIMER_INT	 				HAL_TIMER1.INTCTRLA
		#define HAL_RFTIMER_OVF_bm 			0x03
		#define HAL_RFTIMER_CAPTURE	 		HAL_TIMER1.INTCTRLB
		#define HAL_RFTIMER_CAPTURE_bm 	0xff

#if	( F_CPU == 32768000UL )
		#define HAL_TIMER_PRE_CONFIG (0x07) /*prescaler 1024*/
		#define HAL_US_PER_SYMBOL ( 2 ) 
		#define HAL_SYMBOL_MASK 	( 0x7FFFffff )
#else /* ! 32768000 */
#error "Your Xmega clock is unsupported"

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
#endif /* defined(XMEGA) */

/* 
 * ===
 * Timer1 wrapper functions
 * ===
 *
 * Those macros expand to platform specific functions 
 */
#define hal_timer_clear_input_capture() HAL_CLEAR_INPUT_CAPUTURE()
/** \brief Enable interrupt for timer overflow  */
#define hal_timer_enable_overflow_interrupt() (HAL_RFTIMER_INT |= HAL_TIMER_OVF_bm)
/** \brief Disable interrupt for timer overflow  */
#define hal_timer_disable_overflow_interrupt() (HAL_RFTIMER_INT &= ~HAL_TIMER_OVF_bm)
/** \brief set the prescaler using macro based on F_CPU */
#define	hal_timer_set_prescaler()				 (HAL_RFTIMER1_PRE |= HAL_TIMER_PRE_CONFIG)
/** \brief Clear the input capture flag  */
#define hal_clear_input_capture()		 	 (HAL_RFTIMER_CAPTURE &= ~HAL_RFTIMER_CAPTURE_bm)	

/*
 * ===
 * Radio interrupt platform specific macros
 * ===
 */ 
/**
 *  Low Level radio interrupt macros
 * \brief Platform specific macros used to set 
 * 				up interrupt from rf chip 
 * \{
 */
/**  \brief  Enable the interrupt from the radio transceiver. */
#define hal_enable_txrx_interrupt( ) HAL_ENABLE_RADIO_INTERRUPT( )
/** \brief  Disable the interrupt from the radio transceiver.
 *  \retval 0 if the pin is low, 1 if the pin is high. */
#define hal_disable_trx_interrupt( ) HAL_DISABLE_RADIO_INTERRUPT( )


#if HARWARE_REVISION == ZIGBIT
#define RADIO_VECT 										INT5_vect// IRQ E5 for Zigbit example
#define TIMER_OVF_VECT 								TIMER1_OVF_vect
#define HAL_ENABLE_RADIO_INTERRUPT( ) \
				{ ( EIMSK |= ( 1 << INT5 ) ) ;\
 				EICRB |= 0x0C ;\
 				PORTE &= ~(1<<PE5);\
				DDRE &= ~(1<<DDE5); }
#define HAL_DISABLE_RADIO_INTERRUPT() 	 (EIMSK &= ~( 1 << INT5 ) )

//#define HAL_ENABLE_OVERFLOW_INTERRUPT()(TIMSK1 |= ( 1 << TOIE1 ) )

#elif HW_PLATFORM == XMEGA
#warning "Still to be defined"
#define  RADIO_VECT											 asda
#define  TIMER_OVF_VECT 								 asdd
#define  HAL_ENABLE_RADIO_INTERRUPT()
#define  HAL_DISABLE_RADIO_INTERRUPT()	 	

#else /* ! ZIGBIT ! XMEGA */
#define RADIO_VECT 		 										TIMER1_CAPT_vect
#define TIMER_OVF_VECT 										TIMER1_OVF_vect
#define HAL_ENABLE_RADIO_INTERRUPT()	 		( TIMSK1 |= ( 1 << ICIE1 ) )
#define HAL_DISABLE_RADIO_INTERRUPT() 		( TIMSK1 &= ~( 1 << ICIE1 ) )
#endif /* HARDWARE_REVISION == ZIGBIT */

/** \} */
/** This macro will protect the following code from interrupts.*/
#define HAL_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
/** This macro must always be used in conjunction with HAL_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define HAL_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

#else /* MULLE */
#define HAL_ENABLE_RADIO_INTERRUPT( ) ( INT1IC.BYTE |= 1 )
#define HAL_DISABLE_RADIO_INTERRUPT( ) ( INT1IC.BYTE &= ~(1) )
#define HAL_ENABLE_OVERFLOW_INTERRUPT( ) ( TB4IC.BYTE = 1 )
#define HAL_DISABLE_OVERFLOW_INTERRUPT( ) ( TB4IC.BYTE = 0 )
/** This macro will protect the following code from interrupts.*/
#define HAL_ENTER_CRITICAL_REGION( ) MULLE_ENTER_CRITICAL_REGION( )
/** This macro must always be used in conjunction with HAL_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define HAL_LEAVE_CRITICAL_REGION( ) MULLE_LEAVE_CRITICAL_REGION( )

#endif /* !__AVR__ */


/*============================ TYPDEFS =======================================*/
/*============================ PROTOTYPES ====================================*/
/*============================ MACROS ========================================*/
/** \name Macros for radio operation.
 * \{ 
 */
#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */
/** \} */
/*============================ TYPDEFS =======================================*/
/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

/** RX_START event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_rx_start_event_handler(). */
typedef void (*hal_rx_start_isr_event_handler_t)(uint32_t const isr_timestamp, uint8_t const frame_length);

/** RRX_END event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_trx_end_event_handler(). */
typedef void (*hal_trx_end_isr_event_handler_t)(uint32_t const isr_timestamp);

typedef void (*rx_callback_t) (uint16_t data);

/*============================ PROTOTYPES ====================================*/
void hal_init( void );

void hal_reset_flags( void );
uint8_t hal_get_bat_low_flag( void );
void hal_clear_bat_low_flag( void );

hal_trx_end_isr_event_handler_t hal_get_trx_end_event_handler( void );
void hal_set_trx_end_event_handler( hal_trx_end_isr_event_handler_t trx_end_callback_handle );
void hal_clear_trx_end_event_handler( void );

hal_rx_start_isr_event_handler_t hal_get_rx_start_event_handler( void );
void hal_set_rx_start_event_handler( hal_rx_start_isr_event_handler_t rx_start_callback_handle );
void hal_clear_rx_start_event_handler( void );

uint8_t hal_get_pll_lock_flag( void );
void hal_clear_pll_lock_flag( void );

/* Hack for atmega128rfa1 with integrated radio. Access registers directly, not through SPI */
#if defined(__AVR_ATmega128RFA1__)
//#define hal_register_read(address) _SFR_MEM8((uint16_t)address)
#define hal_register_read(address) address
uint8_t hal_subregister_read( uint16_t address, uint8_t mask, uint8_t position );
void hal_subregister_write( uint16_t address, uint8_t mask, uint8_t position,
                            uint8_t value );

//#define hal_register_write(address, value) _SFR_MEM8((uint16_t)address)=value
#define hal_register_write(address, value) address=value
//#define hal_subregister_read( address, mask, position ) (_SFR_MEM8((uint16_t)address)&mask)>>position
//#define hal_subregister_read1( address, mask, position ) (address&mask)>>position
//#define hal_subregister_write( address, mask, position, value ) address=(address<<position)&mask
#else
uint8_t hal_register_read( uint8_t address );
void hal_register_write( uint8_t address, uint8_t value );
uint8_t hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position );
void hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value );
#endif



//void hal_frame_read(hal_rx_frame_t *rx_frame, rx_callback_t rx_callback);
/* For speed RF230BB does not use a callback */
void hal_frame_read(hal_rx_frame_t *rx_frame);
void hal_frame_write( uint8_t *write_buffer, uint8_t length );
void hal_sram_read( uint8_t address, uint8_t length, uint8_t *data );
void hal_sram_write( uint8_t address, uint8_t length, uint8_t *data );
/* Number of receive buffers in RAM. */
#ifndef RF230_CONF_RX_BUFFERS
#define RF230_CONF_RX_BUFFERS 1
#endif

#endif
/** @} */
/*EOF*/
