/*
 * Copyright (c) 2012, Timothy Rule <trule.github@nym.hush.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * @file
 * 		Platform for AL-XSLED-EXT.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdio.h>
#include <contiki.h>
#include <autostart.h>
#include <interrupt.h>
#include <sd.h>
#include <spi-xmega.h>
#include <dev/watchdog.h>
#include <dev/rs232.h>
#include <dev/leds.h>


#define ANNOUNCE_BOOT 1
#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif


static spi_xmega_slave_t spi_slaves [] = {
	{
		&PORTC,
		&SPIC.DATA,
		&SPIC.CTRL,
		&SPIC.STATUS,
		&PORTC,
		PIN4_bm,
		SPI_CLK2X_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV16_gc, /* 250KHz */
		0
	},
};

int8_t sd_fd  = -1;

/**
 *
 */
static void sd_init(void)
{
	int rc;

	sd_fd = spi_open(&spi_slaves[0]);
	if (sd_fd < 0) {
		dprintf("spi open failed\n");
		return;
	}

	rc = sd_initialize(&spi_slaves[0]);
	if (rc != 0) {
		dprintf("SD init result %d\n", rc);
		return;
	}
}



/**
 *
 */
static void initalize(void)
{
	/* Leds */
	leds_init();
	leds_on(LED_STATUS);

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

	/* Watchdog */
	if (RST.STATUS & RST_WDRF_bm) {
		printf_P(PSTR("Watchdog caused reset!\n"));
		leds_on(LED_ALERT);
	}
	watchdog_init();
	watchdog_start();

	/* Clock */
	clock_init();

	/* SPI Busses */
	spi_init_multi(spi_slaves, 1);
	sd_init();

	/* Process subsystem */
	dprintf("Starting process subsystem\n");
	process_init();
	process_start(&etimer_process, NULL);
	autostart_start(autostart_processes);
}

/**
 *
 */
int main(void)
{
	initalize();

	dprintf("Starting main loop...\n");
	while (1) {
		watchdog_periodic();
		process_run();
	}

	return 0;
}
