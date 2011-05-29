#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"
#include <util/delay.h>
#include "xmega-adc.h"


/* run the ADC on the internal temperature sensor (blocking) */
uint16_t adc_sample_temperature(void) {
    uint16_t ret;
    ADCA.CTRLA = ADC_ENABLE_bm; //enable the ADC
    ADCA.REFCTRL = ADC_BANDGAP_bm; //enable the bandgap as a reference
    ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc; //64 div prescaler
    //ADC.CALL = cal

    ADCA.CH0.MUXCTRL = 0; //set channel to be temp.
    ADCA.CH0.CTRL |= ADC_CH_START_bm; //start ch0 conversion

    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)); //wait for conversion complete
    ret = ADCA.CH0.RES; //get result
    ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm; //reset flag bit for next time

    return ret;
}

//returns VCC in v
float adc_sample_vcc(void) {
    float ret;
    ADCA.CTRLA = ADC_ENABLE_bm; //enable the ADC
    ADCA.REFCTRL = ADC_BANDGAP_bm; //enable the bandgap as a reference
    ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc; //64 div prescaler
    //ADC.CALL = cal

    ADCA.CH0.MUXCTRL = ADC_CH_MUXINT_SCALEDVCC_gc; //set channel to be temp.
    ADCA.CH0.CTRL |= ADC_CH_START_bm; //start ch0 conversion

    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)); //wait for conversion complete
    ret = ADCA.CH0.RES; //get result
    ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm; //reset flag bit for next time

    return ret / 10;
}