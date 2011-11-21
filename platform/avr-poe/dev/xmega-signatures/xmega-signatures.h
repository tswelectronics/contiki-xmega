#ifndef __XMEGA_SIGNATURES_H__
#define __XMEGA_SIGNATURES_H__

uint8_t prod_read( uint8_t index );
void get_serial_number(char* dest, uint8_t len);
void print_serial_number(void);


#endif
