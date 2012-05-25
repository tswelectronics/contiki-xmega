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
#include <antelope.h>
#include <cfs.h>
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

#define CFS_STATUS_FILE "/cfs_status.txt"

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
static int sd_init(void)
{
	int rc;

	sd_fd = spi_open(&spi_slaves[0]);
	if (sd_fd < 0) {
		dprintf("spi open failed\n");
		return -1;
	}

	rc = sd_initialize(&spi_slaves[0]);
	if (rc != 0) {
		dprintf("SD init result %d\n", rc);
		return rc;
	}

	return 0;
}

/**
 *
 */
static int cfs_init(void)
{
	int fd;

	if (sd_fd < 0) {
		dprintf("CFS not initialised, no SD Card\n");
		return -1;
	}

#if 0
	dprintf("Formating SD Card for CFS...");
	cfs_coffee_format();
	dprintf("OK\n");
#endif

	fd = cfs_open(CFS_STATUS_FILE, CFS_WRITE);
	if (fd < 0) {
		dprintf("Formating SD Card for CFS...");
		cfs_coffee_format();
		dprintf("OK\n");

		fd = cfs_open(CFS_STATUS_FILE, CFS_WRITE);
		if (fd < 0) {
			dprintf("CFS not initialised, could not open status file\n");
			return -2;
		}
		cfs_write(fd, "CFS OK\n", 8);
	}
	cfs_close(fd);

	return 0;
}

/**
 *
 */
static int sensor_db_init(void)
{
	db_handle_t handle;
	db_result_t result;

	result = db_query(&handle, "SELECT id, name, unit, scale FROM sensor;");
	db_free(&handle);
	if (DB_ERROR(result) != 0) {
		dprintf("DB creating sensor table...\n");

		db_query(NULL, "REMOVE RELATION sensor;");
		db_query(NULL, "CREATE RELATION sensor;");
		db_query(NULL, "CREATE ATTRIBUTE id DOMAIN INT IN sensor;");
		db_query(NULL, "CREATE ATTRIBUTE name DOMAIN STRING(10) IN sensor;");
		db_query(NULL, "CREATE ATTRIBUTE unit DOMAIN STRING(10) IN sensor;");
		db_query(NULL, "CREATE ATTRIBUTE scale DOMAIN INT IN sensor;");
		db_query(NULL, "CREATE INDEX sensor.id TYPE INLINE;");
		db_query(NULL, "CREATE INDEX sensor.name TYPE INLINE;");

		result = db_query(&handle, "SELECT id, name, unit, scale FROM sensor;");
		db_free(&handle);
		if (DB_ERROR(result) != 0) {
			dprintf("DB table init failed with reason : %s\n",
								db_get_result_message(result));
			return -1;
		}
	}

	result = db_query(&handle, "SELECT sensor_id, time, value FROM sample;");
	db_free(&handle);
	if (DB_ERROR(result) != 0) {
		dprintf("DB creating sample table...\n");

		db_query(NULL, "REMOVE RELATION sample;");
		db_query(NULL, "CREATE RELATION sample;");
		db_query(NULL, "CREATE ATTRIBUTE sensor_id DOMAIN INT IN sample;");
		db_query(NULL, "CREATE ATTRIBUTE time DOMAIN INT IN sample;");
		db_query(NULL, "CREATE ATTRIBUTE value DOMAIN INT IN sample;");
		db_query(NULL, "CREATE INDEX sample.sensor_id TYPE INLINE;");
		db_query(NULL, "CREATE INDEX sample.time TYPE INLINE;");

		result = db_query(&handle, "SELECT sensor_id, time, value FROM sample;");
		db_free(&handle);
		if (DB_ERROR(result) != 0) {
			dprintf("DB table init failed with reason : %s\n",
					db_get_result_message(result));
			return -2;
		}
	}

	return 0;
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
	if (sd_init() != 0) {
		printf_P(PSTR("Boot halted, SD-card failed init!\n"));
		leds_on(LED_ALERT);
		while (1);
	}
	if (cfs_init() != 0) {
		printf_P(PSTR("Boot halted, Coffee FS failed init!\n"));
		leds_on(LED_ALERT);
		while (1);
	}
	if (sensor_db_init() != 0) {
		printf_P(PSTR("Boot halted, Sensor DB failed init!\n"));
		leds_on(LED_ALERT);
		while (1);
	}

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
