/**
 *
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>
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


/* ? */
#define CCIF
#define CLIF



typedef int32_t s32_t;
typedef unsigned short clock_time_t;
typedef unsigned char u8_t;
typedef unsigned short u16_t;
typedef unsigned long u32_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

void clock_delay(unsigned int us2);
void clock_wait(int ms10);
void clock_set_seconds(unsigned long s);
unsigned long clock_seconds(void);




#endif /* __CONTIKI_CONF_H__ */
