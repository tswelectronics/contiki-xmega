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
#include <spi_xmega.h>
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
		SPI_MODE_0_gc | SPI_PRESCALER_DIV64_gc,
		0
	},
};

static int8_t sd_fd;


/**
 * Test SD interface.
 */
#define SD_TRANSACTION_ATTEMPTS		512
#define SD_READ_RESPONSE_ATTEMPTS	8
#define SPI_IDLE	0xff
#define GO_IDLE_STATE		0
#define R1			1
#define R2			2
#define R3			5
#define R7			5

void send_command(uint8_t cmd, uint32_t argument)
{
	uint8_t req[8];

	req[0] = SPI_IDLE;
	req[1] = 0x40 | cmd;
	req[2] = argument >> 24;
	req[3] = argument >> 16;
	req[4] = argument >> 8;
	req[5] = argument;
	req[6] = 0x95;
	req[7] = SPI_IDLE;

	spi_write(sd_fd, req, 8);
}

static uint8_t * get_response(int length)
{
	int i;
	static uint8_t r[R7];

	for (i = 0; i < SD_READ_RESPONSE_ATTEMPTS; i++) {
		spi_read(sd_fd, r, 1);

		if((r[0] & 0x80) == 0) {
			/* A get_response byte is indicated by the MSB being 0. */
			break;
		}
	}

	if (i == SD_READ_RESPONSE_ATTEMPTS) {
		return NULL;
	}

	spi_read(sd_fd, &r[1], length - 1);

	return r;
}

static unsigned char *transaction(int command, unsigned long argument,
	int response_type, unsigned attempts)
{
	unsigned i;
	unsigned char *r;

	if (spi_lock(sd_fd)) {
		dprintf("spi busy\n");
		return NULL;
	}

	r = NULL;
	for (i = 0; i < attempts; i++) {
		send_command(command, argument);
		r = get_response(response_type);
		if(r != NULL) {
			break;
		}
	}

	spi_unlock(sd_fd);

	return r;
}

static void sd_init(void)
{
	if (PORTC.IN & PIN3_bm) {
		dprintf("microSD card is not inserted\n");
		return;
	}

	sd_fd = spi_open(&spi_slaves[0]);
	if (sd_fd < 0) {
		dprintf("spi open failed\n");
		return;
	}

	/* Send go idle. */
	uint8_t *r;
	r = transaction(GO_IDLE_STATE, 0, R1, SD_TRANSACTION_ATTEMPTS);
	if (r != NULL) {
		dprintf("Go-idle result: %d\n", r[0]);
	} else {
		dprintf("Failed to get go-idle response\n");
	}

	spi_close(sd_fd);
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

	/* SPI */
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
