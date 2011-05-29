#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"
#include <util/delay.h>

// defines for the buttons

#define PNLBTN_ROW_PORT PORTA
#define PNLBTN_ROW_START 1
#define PNLBTN_ROWS 5

#define PNLBTN_COL_A_PORT PORTD
#define PNLBTN_COL_A_MASK (1<<3)

#define PNLBTN_COL_B_PORT PORTA
#define PNLBTN_COL_B_MASK (1<<6)


//function to read the currently pressed button number, 1-10, or 0 if none.
uint8_t avr_poe_buttons_read (void);
