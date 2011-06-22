#include "common.h"

PROCESS(blink_process, "Blink");
//AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_EXITHANDLER(goto exit;)
  PROCESS_BEGIN();

	leds_on(0x02);
	printf("Hello I'm \"Blink\" process:\nstart blinking\n");
  while(1) {
    static struct etimer et;
    etimer_set(&et, 125);//CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    leds_on(0x01);
		printf(".\n\0");
    etimer_set(&et,125);// CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    leds_off(0x01);
		printf(".\n\0");
  }

 exit:
  leds_off(LEDS_ALL);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
