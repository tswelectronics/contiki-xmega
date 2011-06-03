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
#include "hal/hal_spi.h"
#include "hal/hal_timer.h"
#include "hal/hal_interrupt.h"
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

/** \name Critical section operations 
 * \{
 */
#if defined(__AVR__)
/** This macro will protect the following code from interrupts.*/
#define HAL_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
/** This macro must always be used in conjunction with HAL_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define HAL_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}
#else /* MULLE */
/** This macro will protect the following code from interrupts.*/
#define HAL_ENTER_CRITICAL_REGION( ) MULLE_ENTER_CRITICAL_REGION( )
/** This macro must always be used in conjunction with HAL_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define HAL_LEAVE_CRITICAL_REGION( ) MULLE_LEAVE_CRITICAL_REGION( )
#endif
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

/** RX_START event handler callback type. 
 *  Is called with timestamp in IEEE 802.15.4 symbols and frame length.
 *  See hal_set_rx_start_event_handler(). */
typedef void (*hal_rx_start_isr_event_handler_t)
	(uint32_t const isr_timestamp, uint8_t const frame_length);

/** RRX_END event handler callback type.
 *  Is called with timestamp in IEEE 802.15.4 symbols and frame length.
 *  See hal_set_trx_end_event_handler(). */
typedef void (*hal_trx_end_isr_event_handler_t)
	(uint32_t const isr_timestamp);

typedef void (*rx_callback_t)
 	(uint16_t data);

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
