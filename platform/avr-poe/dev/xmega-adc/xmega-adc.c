#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "xmega-signatures.h"
#include "xmega-adc.h"
#include <stdio.h>

#ifndef ADCACAL0_offset

#define ADCACAL0_offset 0x20
#define ADCACAL1_offset 0x21
#define TEMPSENSE0_offset 0x2E
#define TEMPSENSE1_offset 0x2F

#endif



void adc_init(void) {
    #define ADC_CAL 1
    
    #if defined(ADC_CAL)
    uint16_t temp_cal;
    temp_cal =
        prod_read( PROD_SIGNATURES_START + ADCACAL0_offset) +
        (prod_read( PROD_SIGNATURES_START + ADCACAL1_offset) << 8);
    ADCA.CAL = temp_cal;
    #endif
    
    ADCA.CTRLA = ADC_ENABLE_bm; //enable the ADC
    ADCA.REFCTRL = ADC_BANDGAP_bm | ADC_TEMPREF_bm | ADC_REFSEL_VCC_gc; //enable the bandgap as a reference
    ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc; //64 div prescaler
    //ADC.CALL = cal

}

/* run the ADC on the internal temperature sensor (blocking) */
uint16_t adc_sample_temperature(void) {
    uint16_t ret;


    ADCA.CH0.MUXCTRL = 0; //set channel to be temp.
    ADCA.CH0.CTRL |= ADC_CH_START_bm; //start ch0 conversion

    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)); //wait for conversion complete
    ret = ADCA.CH0.RES; //get result
    ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm; //reset flag bit for next time

    return ret;
}

//returns VCC in v
uint16_t adc_sample_vcc(void) {
    uint16_t ret;

    ADCA.CH0.MUXCTRL = ADC_CH_MUXINT_BANDGAP_gc; //set channel to be temp.
    ADCA.CH0.CTRL |= ADC_CH_START_bm; //start ch0 conversion

    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)); //wait for conversion complete
    ret = ADCA.CH0.RES; //get result
    ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm; //reset flag bit for next time

    return ret ;
}