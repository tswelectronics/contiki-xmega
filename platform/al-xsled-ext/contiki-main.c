/**
 *
 */

#include <stdio.h>
#include <contiki.h>
#include <autostart.h>
#include <interrupt.h>
#include <dev/rs232.h>
#include <dev/leds.h>


#define ANNOUNCE_BOOT 1
#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif



/**
 *
 */
static void initalize(void)
{
	/* Leds */
	leds_init();
	leds_on(LED_STATUS | LED_ALERT);

	/* Interrupts */
	interrupt_init(PMIC_CTRL_HML_gm);
	interrupt_start();

	/* Console */
	rs232_init(RS232_USARTD1, XMEGA_BAUD_ASYNC_115200,
			USART_MODE_ASYNC | USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);
	rs232_redirect_stdout(RS232_USARTD1);



#if ANNOUNCE_BOOT
	printf_P(PSTR("\n*******Booting %s*******\n"), CONTIKI_VERSION_STRING);
#endif
	/* Clock */
#if defined(__SYSTEM_CLOCK_SETUP__)
	//system_clock_init();
#endif
	clock_init();

	/* Watchdog */


	/* Process subsystem */
	dprintf("Starting process subsystem\n");
	process_init();
	autostart_start(autostart_processes);

	/* Init done, turn of Alert LED. */
	leds_off(LED_ALERT);
}

/**
 *
 */
int main(void)
{
	initalize();

	dprintf("Starting main loop...\n");
	while (1) {
		// watchdog_periodic();
		process_run();
	}

	return 0;
}
