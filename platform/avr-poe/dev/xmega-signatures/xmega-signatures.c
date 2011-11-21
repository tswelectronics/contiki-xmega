#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "xmega-signatures.h"
#include <stdio.h>



uint8_t prod_read( uint8_t index ) {
    uint8_t ret;
    /*set up to read from the prod calibration row*/
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    ret = pgm_read_byte(index);

    /* set back */
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;

    return ret;
}


//copy the last  _len_ bytes of the serial number into _dest_
void get_serial_number(char* dest, uint8_t len) {
    if (len > 11)
        len = 11;

    while (len > 0) {
        len--;

        if (len > 6) // 10, 9, 8, 7
            *dest = prod_read(PROD_SIGNATURES_START + 0x12 + len - 7);
        else if (len > 5) // 6
            *dest = prod_read(PROD_SIGNATURES_START + 0x10);
        else if (len==5)
            *dest = 0x00;
        else if (len==4)
            *dest = 0x12;
        else if (len==3)
            *dest = 0x02;
        else // 5 4 3 2 1 0
            *dest = prod_read(PROD_SIGNATURES_START + 0x12 + len);

        dest++;
    }
}

void print_serial_number(void) {
    uint8_t i;

    printf("serial number: ");
    for (i=0; i<6; i++) {
        printf("%2X ", prod_read(PROD_SIGNATURES_START + 0x8 + i));
    }
    printf("%2X ", prod_read(PROD_SIGNATURES_START + 0x10));
    for (i=0; i<4; i++) {
        printf("%2X ", prod_read(PROD_SIGNATURES_START + 0x12 + i));
    }
    printf("\n");
}

