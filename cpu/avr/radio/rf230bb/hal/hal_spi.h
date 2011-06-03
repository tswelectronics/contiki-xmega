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
 *    \addtogroup hal spi
 *    @{
 */

/**
 *  \file
 *  \brief This file contains low-level radio driver code.
 *
 *   $Id: hal.h,v 1.5 2010/12/03 20:42:01 dak664 Exp $
*/

#ifndef __HAL_AVR_SPI__H
#define __HAL_AVR_SPI__H
/* Define all possible revisions here */
// Don't use zero, it will match if undefined!
// RAVEN_D : Raven kit with LCD display
// RAVENUSB_C : used for USB key or Raven card 
// RCB_B : RZ200 kit from Atmel based on 1281V
// ZIGBIT : Zigbit module from Meshnetics
// ATMEGA128RFA1 : Bare chip with internal radio

#define RAVENUSB_C  	1
#define RCB_B	    		2
#define ZIGBIT				3
#define ATMEGA128RFA1 4
#define RAVEN_D	    	4
#define XMEGA 				5

/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
//#include <util/crc16.h>
#include "contiki-conf.h"

/** \name This is the list of supported hardware configuration
 * \brief Change values defined in the headers to port to other platforms.
 * \{
 */

#if RCB_REVISION == RCB_B
/* 1281 rcb */
#include "hal_spi_atmega.h"

#elif HARWARE_REVISION == ZIGBIT
#include "hal_spi_atmega.h"

#elif HW_PLATFORM == XMEGA
/* Xmega based platforms*/
#include "hal_spi_atxmega.h"

#elif RAVEN_REVISION == RAVEN_D
/* 1284 raven */
#include "hal_spi_atmega.h"

#elif RAVEN_REVISION == RAVENUSB_C
#include "hal_spi_atmega.h"

#elif HARWARE_REVISION == ATMEGA128RFA1
/* ATmega1281 with internal AT86RF231 radio */
#include "hal_spi_atmega.h"

#elif CONTIKI_TARGET_MULLE
#include "hal_spi_mulle.h"

#else
#error "Platform undefined in hal.h"
#endif

/*============================ MACROS ========================================*/

// TEST CODE
#define TRIG1 DDRB |= 0x04, PINB |= 0x04
#define TRIG2 DDRD |= 0x80, PIND |= 0x80

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
#undef  RST
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

#define HAL_SS_HIGH( ) (HAL_PORT_SS |= ( 1 << HAL_SS_PIN )) /**< MACRO for pulling SS high. */
#define HAL_SS_LOW( )  (HAL_PORT_SS &= ~( 1 << HAL_SS_PIN )) /**< MACRO for pulling SS low. */
/** \} */

/*
 * ===
 * SPI control wrapper functions
 * ===
 * \name spi wrapper macros
 * \brief Those macros use platform indipendent definitions to 
 * 				provide a more generci abstraction
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
/** \} */
#endif
