#ifndef __AVR_POE_LCD_H__
#define __AVR_POE_LCD_H__



//connections here
#define BBLCD_DATA_PORT PORTC

#define BBLCD_CTRL_PORT PORTB
#define BBLCD_CS_PORT PORTA
#define BBLCD_LIGHT_PORT PORTD

#define BBLCD_CTRL_RD (1<<3)
#define BBLCD_CTRL_WR (1<<2)
#define BBLCD_CTRL_A0 (1<<1)
#define BBLCD_CTRL_RESET (1<<0)

#define BBLCD_CS (1<<7)

#define BBLCD_LIGHT_BL (1<<0)
#define BBLCD_LIGHT_LED (1<<1)

//now some of the command defines

#define BBCMD_VOLCTL 0xC6
#define BBCMD_VOLRD 0xB6

#define BBCMD_DON 0xAF
#define BBCMD_DOFF 0xAE

#define BBCMD_DISNOR 0xA6
#define BBCMD_DISINV 0xA7

#define BBCMD_DTYSET 0xA8
#define BBCMD_PATSET 0xCC
#define BBCMD_CKSET 0xBF

#define BBCMD_SLPON 0x95
#define BBCMD_SLPOFF 0x94

#define BBCMD_PDNOR 0x6A
#define BBCMD_PDINV 0x6B

#define BBCMD_PDIR 0x4C
#define BBCMD_CDIR 0x4D

#define BBCMD_CNTCLR 0x56
#define BBCMD_PCCLR 0x0A
#define BBCMD_CCCLR 0x05


#define BBCMD_MWRITE 0x5C
#define BBCMD_CKSET 0xBF
#define BBCMD_RETURN 0xBE



// //function to read the currently pressed button number, 1-10, or 0 if none.
// uint8_t read_buttons (void) ;

void bblcd_initialize(void) ;

uint8_t bblcd_read(void) ;

void bblcd_cmd(uint8_t cmd) ;

void bblcd_data(uint8_t data) ;

void bblcd_display_n(uint8_t * data, uint8_t n);
void change_image(uint8_t seed);

void bblcd_checkerboard(uint8_t offset);


#endif