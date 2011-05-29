#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef __CONTIKI_INTERRUPT_AVR__
#define __CONTIKI_INTERRUPT_AVR__

/* all ATXMEGA's should have defined*/
#if defined(PMIC_CTRL)
#define USE_RR 0
typedef enum PMIC_CTRL_INTLVL_enum {
	PMIC_CTRL_H_gm   = 0x04,
	PMIC_CTRL_M_gm   = 0x02,
	PMIC_CTRL_HML_gm = 0x07,
	PMIC_CTRL_ML_gm  = 0x03,
	PMIC_CTRL_L_gm   = 0x01
}PMIC_CTRL_INTLVL_t;

#endif /* __AVR_ATxmega__*/
/* prototypes */
/*
 * \brief       Initialize interrupt vector
 *				
 *			  	    This function has to be called from
 *			  	    low level init function, before using
 *				      any kind of interrupt driven module
 *
 *	\param			the enabled interrupt level mask.
 *							use the PMIC_CTRL_INTLVL_t type to define the interrupt
 *							levels to be enabled in Programmable Multilevel
 *							Interrupt Contol Module.
 *							
 *
 */ 			
void 
interrupt_init(PMIC_CTRL_INTLVL_t int_level);

/*
 * \brief			A wrapper around the sei macro (assembly SEI)
 */
static void inline
interrupt_start(void) {sei();}

/*
 * \brief 			Return the global interrupt vector status
 *
 * \return			The STATUS register, a OR combination of:
 * 							0x80: Non-Maskable interrupt (NMI) executing
 * 							0x04: High-Level interrupt executing (or interrupted by NMI)
 * 							0x02: Mid-Level interrupt executing (or interrupted by higer level interrupt)
 * 							0x01: Low-Level interrupt executing (or interrupted by higer level interrupt)
 */
uint8_t
interrupt_get_status(void);

#endif
