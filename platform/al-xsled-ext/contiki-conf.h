/**
 *
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>
#include <avrdef.h>
#include <avr/io.h>

/* MCU and clock rate */
#define MCU_MHZ 32.0
#define PLATFORM PLATFORM_AVR

/* Clock ticks per second */
#define CLOCK_CONF_SECOND 100

/* Maximum time interval (used for timers) */
#define INFINITE_TIME 0xffff

/* Pre-allocated memory for loadable modules heap space (in bytes)*/
#define MMEM_CONF_SIZE 256

/* LED's */
#define __USE_LEDS__
#define LEDPORT PORTF
#define LED1 PIN7_bm
#define LED2 PIN6_bm
#define LED3 PIN5_bm
#define LED4 PIN4_bm
#define LEDS_CONF_ALL (LED1 | LED2 | LED3 | LED4)
#define LED_STATUS LED1
#define LED_ALERT LED2
/* The following defs satisfy the LEDS api, and do nothing. */
#define LEDS_GREEN PIN3_bm
#define LEDS_YELLOW PIN3_bm
#define LEDS_RED PIN3_bm
#define LEDS_BLUE PIN3_bm

/* Watchdog */
#define WATCHDOG_CONF_TIMEOUT WDT_PER_8KCLK_gc

/* ? */
#define CCIF
#define CLIF


#endif /* __CONTIKI_CONF_H__ */
