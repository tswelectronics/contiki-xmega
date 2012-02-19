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
 * 		Platform for AL-XSLED-EXT, sensors.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdio.h>
#include <stddef.h>
#include <contiki.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>


#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif


/* Sensor values are read in to these globals. */
uint16_t calibration_temp;
uint16_t calibration_adc;
uint16_t on_chip_temp;
uint16_t on_chip_vcc;
uint16_t on_chip_bandgap;

static struct etimer timer_monitor;
static struct etimer timer_display;

static uint16_t
adc_read_internal(uint8_t muxctrl, uint16_t calibration)
{
	uint16_t result;

	if (ADCA.CTRLA & ADC_ENABLE_bm) {
		dprintf("ADC is in use!\n");
		return 0;
	}

	/* Setup and enable the ADC. */
	ADCA.CALL = calibration & 0xff;
	ADCA.CALH = (calibration >> 8) & 0xff;
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm | ADC_TEMPREF_bm;
	ADCA.CTRLA |= ADC_ENABLE_bm;

	/* Setup CH0 and start measurement. */
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;
	ADCA.CH0.MUXCTRL = muxctrl;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;

	/* Wait for complete, get result. */
	while (!ADCA.CH0.INTFLAGS);
	ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
	result = ADCA.CH0RES;

	/* Disable the ADC*/
	ADCA.CTRLA = 0;

	return result;
}

PROCESS(on_chip_sensors_monitor_process, "On Chip Sensors");
PROCESS_THREAD(on_chip_sensors_monitor_process, ev, data)
{
	PROCESS_BEGIN();

	dprintf("On Chip Sensors ready to read ...\n");

	/* Load calibration settings. */
	calibration_temp = xmega_read_calibration_byte(
			offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE0)) & 0xff;
	calibration_temp |= (xmega_read_calibration_byte(
			offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE1)) << 8);
	calibration_adc = xmega_read_calibration_byte(
			offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0)) & 0xff;
	calibration_adc |= (xmega_read_calibration_byte(
			offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1)) << 8);

	dprintf("ADC calibration (temp/adc) %04x : %04x\n",
			calibration_temp, calibration_adc);

	/* Start the measurement loop, each 10 seconds. */
	etimer_set(&timer_monitor, CLOCK_SECOND * 10);

	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		etimer_reset(&timer_monitor);

		/* Measure */
		on_chip_temp = adc_read_internal
				(ADC_CH_MUXINT_TEMP_gc, calibration_temp);
		on_chip_vcc = adc_read_internal(
				ADC_CH_MUXINT_SCALEDVCC_gc, calibration_adc);
		on_chip_bandgap = adc_read_internal(
				ADC_CH_MUXINT_BANDGAP_gc, calibration_adc);
	}

	PROCESS_END();
}

PROCESS(on_chip_sensors_display_process, "On Chip Sensors Display");
PROCESS_THREAD(on_chip_sensors_display_process, ev, data)
{
	PROCESS_BEGIN();

	/* Start the display loop, each 10 seconds. */
	etimer_set(&timer_display, CLOCK_SECOND * 10);

	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		etimer_reset(&timer_display);

		dprintf("ADC result (temp/vcc/bandgap) %04x : %04x : %04x\n",
				on_chip_temp, on_chip_vcc, on_chip_bandgap);

		/*
		 * Cal temp is at 85¡C, but using the value as if at 85 ¡F gives
		 * conversions that make more sense. So use 29.4¡C (== 85¡C).
		 */
		int32_t temp = on_chip_temp;
		temp = ((29.4 + 273.0)/calibration_temp) * temp - 273;
		dprintf("ADC temp (degC) %d\n", temp);

		int32_t vcc = on_chip_vcc;
		vcc = (vcc - 200)*10*1000/4095;
		dprintf("ADC vcc (mV) %d\n", vcc);
	}

	PROCESS_END();
}
