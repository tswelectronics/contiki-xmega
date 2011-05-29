#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"
#include <util/delay.h>
#include "lcd.h"

uint8_t image[8] = {0b00001100,
                    0b00011100,
                    0b00011100,
                    0b00111100,
                    0b00111100,
                    0b01101100,
                    0b01101100,
                    0b11101100,
                   };





void bblcd_initialize(void) {

    BBLCD_DATA_PORT.DIRSET = 0xff;

    BBLCD_CTRL_PORT.DIRSET = BBLCD_CTRL_RD | BBLCD_CTRL_WR | BBLCD_CTRL_A0 | BBLCD_CTRL_RESET;
    BBLCD_CS_PORT.DIRSET = BBLCD_CS;
    BBLCD_LIGHT_PORT.DIRSET =  BBLCD_LIGHT_BL ;//| BBLCD_LIGHT_LED;
    //BBLCD_CTRL_DDR = 0xff;
    //BBLCD_DATA_DDR = 0xff;

    uint8_t reg = MCU.MCUCR; //we'll need to set the jtag disable bit
    reg |= MCU_JTAGD_bm; //pre-calculate
    CPU_CCP = CCP_IOREG_gc; //unlock for writing
    MCU.MCUCR = reg;

    //initialize
    //raise !RD, and !WR
    BBLCD_CTRL_PORT.OUTSET = BBLCD_CTRL_RD;
    BBLCD_CTRL_PORT.OUTSET = BBLCD_CTRL_WR;

    //raise !reset
    BBLCD_CTRL_PORT.OUTCLR = BBLCD_CTRL_RESET;
    _delay_ms(10);
    BBLCD_CTRL_PORT.OUTSET = BBLCD_CTRL_RESET;
    _delay_ms(10);

    //send initialize cmds
    bblcd_cmd(BBCMD_CNTCLR);
    bblcd_cmd(BBCMD_PCCLR);
    bblcd_cmd(BBCMD_CCCLR);

    //set to 1/160 duty cycle (160 rows)
    bblcd_cmd(BBCMD_DTYSET);
    bblcd_data((180-1)/4);

    bblcd_cmd(BBCMD_PATSET);
    bblcd_data(0b01);

    bblcd_cmd(BBCMD_CKSET);
    bblcd_data(0b01);
    
    //turn off sleep
    bblcd_cmd(BBCMD_SLPOFF);
    _delay_ms(100);

    bblcd_cmd(BBCMD_DON);

    bblcd_cmd(BBCMD_CDIR);


    
}

uint8_t bblcd_read(void) {
    uint8_t ret = 0;
    BBLCD_DATA_PORT.DIR = 0;
 //   _delay_us(1);
    BBLCD_CS_PORT.OUTCLR = BBLCD_CS;
//    _delay_us(1);
    BBLCD_CTRL_PORT.OUTSET = (BBLCD_CTRL_WR) ;
    BBLCD_CTRL_PORT.OUTCLR = (BBLCD_CTRL_RD | BBLCD_CTRL_A0 );
    //BBLCD_CTRL_PORT &= ~(1<<BBLCD_CTRL_A0);
    //BBLCD_CTRL_PORT |= (1<<BBLCD_CTRL_WR);//raise
    //BBLCD_CTRL_PORT &= ~(1<<BBLCD_CTRL_CS);//lower
    //BBLCD_CTRL_PORT &= ~(1<<BBLCD_CTRL_RD);
//    _delay_us(1);
    ret = BBLCD_DATA_PORT.IN;
 //   _delay_us(1);
    BBLCD_CTRL_PORT.OUTSET =  (BBLCD_CTRL_RD);
    BBLCD_CTRL_PORT.OUTCLR = (BBLCD_CTRL_A0);
    BBLCD_CS_PORT.OUTSET =  BBLCD_CS;
    BBLCD_DATA_PORT.DIR = 0xFF;
    return ret;
}

void bblcd_cmd(uint8_t cmd) {
    uint8_t ret;
    ret = bblcd_read();

    //while (! (ret & 0x80) ) {
    //    ret = bblcd_read();
    //} //wait while still not ready

    BBLCD_CTRL_PORT.OUTCLR = BBLCD_CTRL_A0; //lower
    BBLCD_CS_PORT.OUTCLR &= BBLCD_CS;//lower
    BBLCD_DATA_PORT.OUT = cmd;
//    _delay_us(1);
    BBLCD_CTRL_PORT.OUTCLR = BBLCD_CTRL_WR;//lower
//    _delay_us(1);
    BBLCD_CTRL_PORT.OUTSET = BBLCD_CTRL_WR;//raise
    BBLCD_CS_PORT.OUTSET = BBLCD_CS;//raise


}

void bblcd_data(uint8_t data) {

    uint8_t ret;
    ret = bblcd_read();
    while (! (ret & 0x80) ) {
        ret = bblcd_read();
    }

    BBLCD_CTRL_PORT.OUTSET = BBLCD_CTRL_A0;

    BBLCD_CS_PORT.OUTCLR = BBLCD_CS;//lower

    BBLCD_CTRL_PORT.OUTCLR = BBLCD_CTRL_WR;

    BBLCD_DATA_PORT.OUT = data;
//_delay_us(1);
    BBLCD_CTRL_PORT.OUTSET = BBLCD_CTRL_WR;
//    _delay_us(1);
    BBLCD_CTRL_PORT.OUTCLR = BBLCD_CTRL_A0;

    BBLCD_CS_PORT.OUTSET = BBLCD_CS;//raise


}

void bblcd_display_n(uint8_t * data, uint8_t n) {
    bblcd_cmd(BBCMD_MWRITE);
    uint8_t c;
    for (c=0; c<n; c++) {
        bblcd_data(data[c]);
    }
}

void change_image(uint8_t seed) {
    image[(seed/8)%8] |= (1<<(seed%8));
    image[((seed/8) + 4)%8 ] &= ~(1<<(seed%8));

}

void bblcd_checkerboard(uint8_t offset) {
    //write out a checkerboard pattern
    bblcd_cmd(BBCMD_CNTCLR);
    bblcd_cmd(BBCMD_PCCLR);
    bblcd_cmd(BBCMD_CCCLR);
    uint8_t y;
    for (y=0; y<20; y++) {
        uint8_t line[160];
        uint8_t i;
        for (i=0; i<160; i++) {
            if ((i)%2 == (offset%2))
                line[i] = 0b01010101;
            else
                line[i] = 0b10101010;

        }
        bblcd_display_n(line,160);

        bblcd_cmd(BBCMD_RETURN);
    }
}