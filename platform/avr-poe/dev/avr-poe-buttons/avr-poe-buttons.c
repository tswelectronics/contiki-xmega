#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"
#include <util/delay.h>
#include "avr-poe-buttons.h"



//function to read the currently pressed button number, 1-10, or 0 if none.
uint8_t avr_poe_buttons_read (void) {
    // PNLBTN_ROW_PORT PORTA
    // PNLBTN_ROW_START 1
    // PNLBTN_COL_A_PORT PORTD
    // PNLBTN_COL_A_MASK (1<<3)
    // PNLBTN_COL_B_PORT PORTA
    //  PNLBTN_COL_B_MASK (1<<6)
    //enable pulldowns on row pins
    PNLBTN_ROW_PORT.PIN1CTRL |= PORT_OPC_PULLDOWN_gc;
    PNLBTN_ROW_PORT.PIN2CTRL |= PORT_OPC_PULLDOWN_gc;
    PNLBTN_ROW_PORT.PIN3CTRL |= PORT_OPC_PULLDOWN_gc;
    PNLBTN_ROW_PORT.PIN4CTRL |= PORT_OPC_PULLDOWN_gc;
    PNLBTN_ROW_PORT.PIN5CTRL |= PORT_OPC_PULLDOWN_gc;



    //make them both output
    PNLBTN_COL_A_PORT.DIRSET = PNLBTN_COL_A_MASK;
    PNLBTN_COL_B_PORT.DIRSET = PNLBTN_COL_B_MASK;

    //raise A, lower B
    PNLBTN_COL_A_PORT.OUTSET = PNLBTN_COL_A_MASK;
    PNLBTN_COL_B_PORT.OUTCLR = PNLBTN_COL_B_MASK;
    _delay_us(10);
    uint8_t ret = 0;
    //read rows once
    uint8_t read = PNLBTN_ROW_PORT.IN & 0b00111110;
    if (read) {
        uint8_t i;
        for (i=0; i<PNLBTN_ROWS; i++) {
            if (read == (1<<(i+PNLBTN_ROW_START))) {
                ret = i+1;
            }
        }
    }

    //raise B, lower A
    PNLBTN_COL_A_PORT.OUTCLR = PNLBTN_COL_A_MASK;
    PNLBTN_COL_B_PORT.OUTSET = PNLBTN_COL_B_MASK;
    _delay_us(10);
    //read rows again
    read = PNLBTN_ROW_PORT.IN & 0b00111110;
    if (read) {
        //check if we've detected two buttons
        if (ret) {
            return 0;
        }
        uint8_t i;
        for (i=0; i<PNLBTN_ROWS; i++) {
            if (read == (1<<(i+PNLBTN_ROW_START))) {
                ret = i+PNLBTN_ROWS+1;
            }
        }
    }

    //float all pins.
    PNLBTN_COL_A_PORT.DIRCLR = PNLBTN_COL_A_MASK;
    PNLBTN_COL_B_PORT.DIRCLR = PNLBTN_COL_B_MASK;

    return ret;
}